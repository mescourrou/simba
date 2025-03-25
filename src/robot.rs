/*!
Module providing the main robot manager, [`Robot`], along with the configuration
[`RobotConfig`] and the record [`RobotRecord`] structures.
*/

use std::sync::{Arc, Condvar, Mutex, RwLock};

use super::navigators::navigator::{Navigator, NavigatorConfig, NavigatorRecord};
use super::navigators::trajectory_follower;

use crate::constants::TIME_ROUND;
use crate::controllers::controller::{self, Controller, ControllerConfig, ControllerRecord};
use crate::controllers::pid;

use crate::navigators::navigator;
use crate::networking::message_handler::MessageHandler;
use crate::networking::network::{Network, NetworkConfig};
use crate::networking::service_manager::ServiceManager;
use crate::physics::physic::{Physic, PhysicConfig, PhysicRecord};
use crate::physics::{perfect_physic, physic};

use crate::simulator::{SimulatorConfig, TimeCvData};
use crate::state_estimators::state_estimator::{
    StateEstimator, StateEstimatorConfig, StateEstimatorRecord,
};
use crate::state_estimators::{perfect_estimator, state_estimator};

use crate::sensors::sensor_manager::{SensorManager, SensorManagerConfig, SensorManagerRecord};

use crate::plugin_api::PluginAPI;
use crate::stateful::Stateful;
use crate::time_analysis;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::maths::round_precision;
use crate::utils::time_ordered_data::TimeOrderedData;

#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct BenchStateEstimatorConfig {
    name: String,
    #[check]
    config: StateEstimatorConfig,
}

impl Default for BenchStateEstimatorConfig {
    fn default() -> Self {
        Self {
            name: String::from("bench_state_estimator"),
            config: StateEstimatorConfig::Perfect(Box::new(
                perfect_estimator::PerfectEstimatorConfig::default(),
            )),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BenchStateEstimatorRecord {
    name: String,
    record: StateEstimatorRecord,
}

#[derive(Debug)]
pub struct BenchStateEstimator {
    name: String,
    state_estimator: Arc<RwLock<Box<dyn StateEstimator>>>,
}

// Configuration for Robot
extern crate confy;
use config_checker::macros::Check;
use log::{debug, info, warn};
use serde_derive::{Deserialize, Serialize};
use serde_json::Value;

/// Configuration of the [`Robot`].
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct RobotConfig {
    /// Name of the robot.
    pub name: String,
    /// [`Navigator`] to use, and its configuration.
    #[check]
    pub navigator: NavigatorConfig,
    /// [`Controller`] to use, and its configuration.
    #[check]
    pub controller: ControllerConfig,
    /// [`Physic`] to use, and its configuration.
    #[check]
    pub physic: PhysicConfig,
    /// [`StateEstimator`] to use, and its configuration.
    #[check]
    pub state_estimator: StateEstimatorConfig,
    /// [`SensorManager`] configuration, which defines the [`Sensor`]s used.
    #[check]
    pub sensor_manager: SensorManagerConfig,
    /// [`Network`] configuration.
    #[check]
    pub network: NetworkConfig,

    /// Additional [`StateEstimator`] to be evaluated but without a feedback
    /// loop with the [`Navigator`]
    #[check]
    pub state_estimator_bench: Vec<BenchStateEstimatorConfig>,
}

impl Default for RobotConfig {
    /// Default configuration, using:
    /// * Default [`TrajectoryFollower`](trajectory_follower::TrajectoryFollower) navigator.
    /// * Default [`PID`](pid::PID) controller.
    /// * Default [`PerfectPhysic`](perfect_physic::PerfectPhysic) physics.
    /// * Default [`PerfectEstimator`](perfect_estimator::PerfectEstimator) state estimator.
    /// * Default [`SensorManager`] config (no sensors).
    /// * Default [`Network`] config.
    fn default() -> Self {
        RobotConfig {
            name: String::from("NoName"),
            navigator: NavigatorConfig::TrajectoryFollower(Box::new(
                trajectory_follower::TrajectoryFollowerConfig::default(),
            )),
            controller: ControllerConfig::PID(Box::new(pid::PIDConfig::default())),
            physic: PhysicConfig::Perfect(Box::new(perfect_physic::PerfectPhysicConfig::default())),
            state_estimator: StateEstimatorConfig::Perfect(Box::new(
                perfect_estimator::PerfectEstimatorConfig::default(),
            )),
            sensor_manager: SensorManagerConfig::default(),
            network: NetworkConfig::default(),
            state_estimator_bench: Vec::new(),
        }
    }
}

/// State record of [`Robot`].
///
/// It contains the dynamic elements and the elements we want to save.
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct RobotRecord {
    /// Name of the robot.
    pub name: String,
    /// Record of the [`Navigator`] module.
    pub navigator: NavigatorRecord,
    /// Record of the [`Controller`] module.
    pub controller: ControllerRecord,
    /// Record of the [`Physic`] module.
    pub physic: PhysicRecord,
    /// Record of the [`StateEstimator`] module.
    pub state_estimator: StateEstimatorRecord,
    /// Record of the additionnal [`StateEstimator`]s, only to evaluate them.
    pub state_estimator_bench: Vec<BenchStateEstimatorRecord>,

    pub sensors: SensorManagerRecord,
}

/// Structure to manage the messages for no specific modules of the robot.
#[derive(Clone, Debug)]
pub struct RobotGenericMessageHandler {}

impl RobotGenericMessageHandler {
    /// Create a new instance of [`RobotGenericMessageHandler`].
    pub fn new() -> Self {
        RobotGenericMessageHandler {}
    }
}

impl MessageHandler for RobotGenericMessageHandler {
    /// Handle the message or raise an error (to send the message to the next handler).
    ///
    /// See [`MessageHandler::handle_message`] for trait specification.
    fn handle_message(
        &mut self,
        robot: &mut Robot,
        _from: &String,
        message: &Value,
        _time: f32,
    ) -> Result<(), ()> {
        if robot.message_callback_str(&message).is_ok() {
            return Ok(());
        } else if robot.message_callback_number(&message).is_ok() {
            return Ok(());
        }
        Err(())
    }
}

// Robot itself

/// Structure managing one robot.
///
/// It is composed of modules to manage different aspects:
/// * `navigator` is of [`Navigator`] trait, and defines the error to be sent
/// to the [`Controller`] to follow the required trajectory.
/// * `controller` is of [`Controller`] trait, it defines the command to be sent
/// to the [`Physic`] module.
/// * `physic` is of [`Physic`] trait. It simulates the robot behaviour, its real
/// state. It contains a ground truth to evaluate the [`StateEstimator`].
/// * `state_estimator` is of [`StateEstimator`] trait. It estimates the robot
/// state, and send it to the [`Navigator`].
///
/// * `sensor_manager`, of type [`SensorManager`], manages the [`Sensor`]s. The
/// observations of the sensors are sent to the [`StateEstimator`].
/// * `network` is the robot [`Network`] interface. It manages the reception and
/// the send of messages to other robots.
/// * `message_handler` manages the messages that are not sent to a specific module.
///
/// The [`Robot`] internally manages a history of its states, using [`TimeOrderedData`].
/// In this way, it can get back to a past state, in order to treat a message sent
/// from the past. [`Robot::run_next_time_step`] does the necessary
/// so that the required time is reached taking into account all past messages.
#[derive(Debug, Clone)]
pub struct Robot {
    /// Name of the robot. Should be unique among all [`Simulator`](crate::simulator::Simulator)
    /// robots.
    name: String,
    /// [`Navigator`] module, implementing the navigation strategy.
    navigator: Arc<RwLock<Box<dyn Navigator>>>,
    /// [`Controller`] module, implementing the control strategy.
    controller: Arc<RwLock<Box<dyn Controller>>>,
    /// [`Physic`] module, implementing the physics strategy.
    physic: Arc<RwLock<Box<dyn Physic>>>,
    /// [`StateEstimator`] module, implementing the state estimation strategy.
    state_estimator: Arc<RwLock<Box<dyn StateEstimator>>>,
    /// Manages all the [`Sensor`]s and send the observations to `state_estimator`.
    sensor_manager: Arc<RwLock<SensorManager>>,
    /// [`Network`] interface to receive and send messages with other robots.
    network: Arc<RwLock<Network>>,
    /// [`MessageHandler`] for messages which are not sent to a specific module.
    message_handler: Arc<RwLock<RobotGenericMessageHandler>>,
    /// History of the states ([`RobotRecord`]) of the robot, to set the [`Robot`]
    /// in a past state.
    state_history: TimeOrderedData<RobotRecord>,
    /// Additional [`StateEstimator`] to be evaluated.
    state_estimator_bench: Arc<RwLock<Vec<BenchStateEstimator>>>,

    // services: Vec<Arc<RwLock<Box<dyn ServiceInterface>>>>,
    /// Not really an option, but for delayed initialization
    service_manager: Option<ServiceManager>,
}

impl Robot {
    /// Creates a new [`Robot`] with the given name.
    pub fn new(name: String, time_cv: Arc<(Mutex<TimeCvData>, Condvar)>) -> Arc<RwLock<Self>> {
        let robot = Arc::new(RwLock::new(Self {
            name: name.clone(),
            navigator: Arc::new(RwLock::new(Box::new(
                trajectory_follower::TrajectoryFollower::new(),
            ))),
            controller: Arc::new(RwLock::new(Box::new(pid::PID::new()))),
            physic: Arc::new(RwLock::new(Box::new(perfect_physic::PerfectPhysic::new()))),
            state_estimator: Arc::new(RwLock::new(Box::new(
                perfect_estimator::PerfectEstimator::new(),
            ))),
            sensor_manager: Arc::new(RwLock::new(SensorManager::new())),
            network: Arc::new(RwLock::new(Network::new(name.clone(), time_cv.clone()))),
            message_handler: Arc::new(RwLock::new(RobotGenericMessageHandler::new())),
            state_history: TimeOrderedData::new(),
            state_estimator_bench: Arc::new(RwLock::new(Vec::new())),
            // services: Vec::new(),
            service_manager: None,
        }));
        robot.write().unwrap().save_state(0.);
        let service_manager = Some(ServiceManager::initialize(robot.clone(), time_cv));
        robot.write().unwrap().service_manager = service_manager;
        robot
    }

    /// Creates a new [`Robot`] with the given configuration.
    ///
    /// During the creation of the new robot, each module is initialized with its own config.
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the robot, including all modules. See [`RobotConfig`]
    /// * `plugin_api` -- Optional [`PluginAPI`] implementation required to use external modules.
    /// * `meta_config` -- Simulator config.
    ///
    /// ## Return
    /// The new robot is return as a reference counter to be shared among threads.
    pub fn from_config(
        config: &RobotConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &DeterministRandomVariableFactory,
        time_cv: Arc<(Mutex<TimeCvData>, Condvar)>,
    ) -> Arc<RwLock<Self>> {
        let robot = Arc::new(RwLock::new(Self {
            name: config.name.clone(),
            navigator: navigator::make_navigator_from_config(
                &config.navigator,
                plugin_api,
                global_config,
                va_factory,
            ),
            controller: controller::make_controller_from_config(
                &config.controller,
                plugin_api,
                global_config,
                va_factory,
            ),
            physic: physic::make_physic_from_config(
                &config.physic,
                plugin_api,
                global_config,
                va_factory,
            ),
            state_estimator: state_estimator::make_state_estimator_from_config(
                &config.state_estimator,
                plugin_api,
                global_config,
                va_factory,
            ),
            sensor_manager: Arc::new(RwLock::new(SensorManager::from_config(
                &config.sensor_manager,
                plugin_api,
                global_config,
                &config.name,
                va_factory,
            ))),
            network: Arc::new(RwLock::new(Network::from_config(
                config.name.clone(),
                &config.network,
                global_config,
                va_factory,
                time_cv.clone(),
            ))),
            message_handler: Arc::new(RwLock::new(RobotGenericMessageHandler::new())),
            state_history: TimeOrderedData::new(),
            state_estimator_bench: Arc::new(RwLock::new(Vec::with_capacity(
                config.state_estimator_bench.len(),
            ))),
            // services: Vec::new(),
            service_manager: None,
        }));

        for state_estimator_config in &config.state_estimator_bench {
            robot
                .write()
                .unwrap()
                .state_estimator_bench
                .write()
                .unwrap()
                .push(BenchStateEstimator {
                    name: state_estimator_config.name.clone(),
                    state_estimator: state_estimator::make_state_estimator_from_config(
                        &state_estimator_config.config,
                        plugin_api,
                        global_config,
                        va_factory,
                    ),
                })
        }

        let service_manager = Some(ServiceManager::initialize(robot.clone(), time_cv));
        {
            let mut writable_robot = robot.write().unwrap();
            writable_robot.network.write().unwrap().subscribe(Arc::<
                RwLock<RobotGenericMessageHandler>,
            >::clone(
                &writable_robot.message_handler
            ));
            writable_robot.save_state(0.);

            // Services
            debug!("Setup services");
            writable_robot.service_manager = service_manager;
            // let service = Arc::new(RwLock::new(Box::new(Service::<GetRealStateReq, GetRealStateResp, dyn Physic>::new(time_cv.clone(), physics)) as _ ));
            // writable_robot.services.push(service);
        }

        robot
    }

    /// Initialize the robot after its creation.
    ///
    /// It is used to initialize the sensor manager, which need to know the list of all robots.
    pub fn post_creation_init(
        &mut self,
        robot_list: &Arc<RwLock<Vec<Arc<RwLock<Robot>>>>>,
        robot_idx: usize,
    ) {
        let sensor_manager = self.sensor_manager();
        sensor_manager
            .write()
            .unwrap()
            .init(self, robot_list, robot_idx);
    }

    /// Run the robot to reach the given time.
    ///
    /// It will go back in time if needed by old messages or other asynchronous operations.
    ///
    /// ## Arguments
    /// * `time` -- Time to reach.
    ///
    /// ## Return
    /// Next time step.
    pub fn run_next_time_step(&mut self, time: f32, read_only: bool) {
        self.process_messages();
        self.run_time_step(time, read_only);
    }

    /// Process all the messages: one-way (network) and two-way (services).
    pub fn process_messages(&self) -> usize {
        self.network().write().unwrap().process_messages()
            + self.service_manager.as_ref().unwrap().process_requests()
    }

    /// Run only one time step.
    ///
    /// To run the given `time` step, the robot sets itself in the state of this moment
    /// using [`Robot::set_in_state`].
    ///
    /// The update step is done in this order:
    /// 1. Update the physics
    /// 2. Generate the observations
    /// 3. Correction step of the state estimator
    /// 4. If it is the time for the state estimator to do its prediction step:
    ///     1. The prediction step is done
    ///     2. The navigator computes the error from the state estimation
    ///     3. The command is computed by the Controller
    ///     4. The command is applied to the Physics.
    /// 5. The network messages are handled
    ///
    /// Then, the robot state is saved.
    pub fn run_time_step(&mut self, time: f32, read_only: bool) {
        info!("Run time {}", time);
        self.set_in_state(time);
        // Update the true state
        self.physic.write().unwrap().update_state(time);

        // Make observations (if it is the right time)
        let observations = self
            .sensor_manager()
            .write()
            .unwrap()
            .get_observations(self, time);

        debug!("Got {} observations", observations.len());
        if observations.len() > 0 {
            // Treat the observations
            let ta = time_analysis::time_analysis(
                time,
                "control_loop_state_estimator_correction_step".to_string(),
            );
            self.state_estimator()
                .write()
                .unwrap()
                .correction_step(self, &observations, time);
            time_analysis::finished_time_analysis(ta);

            for state_estimator in self.state_estimator_bench().read().unwrap().iter() {
                let ta = time_analysis::time_analysis(
                    time,
                    state_estimator.name.clone() + "_correction_step",
                );
                state_estimator
                    .state_estimator
                    .write()
                    .unwrap()
                    .correction_step(self, &observations, time);
                time_analysis::finished_time_analysis(ta);
            }
        }

        // If it is time for the state estimator to do the prediction
        if time >= self.state_estimator.read().unwrap().next_time_step() {
            // Prediction step
            let ta = time_analysis::time_analysis(
                time,
                "control_loop_state_estimator_prediction_step".to_string(),
            );
            self.state_estimator()
                .write()
                .unwrap()
                .prediction_step(self, time);
            time_analysis::finished_time_analysis(ta);
            let state = self.state_estimator.read().unwrap().state();

            // Compute the error to the planned path
            let ta = time_analysis::time_analysis(
                time,
                "control_loop_navigator_compute_error".to_string(),
            );
            let error = self.navigator().write().unwrap().compute_error(self, state);
            time_analysis::finished_time_analysis(ta);

            // Compute the command from the error
            let ta = time_analysis::time_analysis(
                time,
                "control_loop_controller_make_command".to_string(),
            );
            let command = self
                .controller()
                .write()
                .unwrap()
                .make_command(self, &error, time);
            time_analysis::finished_time_analysis(ta);

            // Apply the command to the physics
            self.physic.write().unwrap().apply_command(&command, time);
        }
        for state_estimator in self.state_estimator_bench().read().unwrap().iter() {
            if time
                >= state_estimator
                    .state_estimator
                    .read()
                    .unwrap()
                    .next_time_step()
            {
                let ta = time_analysis::time_analysis(
                    time,
                    state_estimator.name.clone() + "_prediction_step",
                );
                state_estimator
                    .state_estimator
                    .write()
                    .unwrap()
                    .prediction_step(self, time);
                time_analysis::finished_time_analysis(ta);
            }
        }

        // Treat messages synchronously
        self.network()
            .write()
            .unwrap()
            .handle_message_at_time(self, time);

        // for service in &self.services {
        //     service.write().unwrap().handle_requests(time);
        // }

        self.service_manager.as_ref().unwrap().handle_requests(time);

        if !read_only {
            // Save state (replace if needed)
            self.save_state(time);
        }
    }

    /// Computes the next time step, using state estimator, sensors and received messages.
    pub fn next_time_step(&self) -> (f32, bool) {
        let mut next_time_step = self.state_estimator.read().unwrap().next_time_step().min(
            self.sensor_manager
                .read()
                .unwrap()
                .next_time_step()
                .unwrap_or(f32::INFINITY),
        );
        let mut read_only = false;

        let message_next_time = self.network().read().unwrap().next_message_time();
        debug!(
            "In robot: message_next_time: {}",
            match message_next_time {
                Some((time, _)) => time,
                None => -1.,
            }
        );
        if let Some(msg_next_time) = message_next_time {
            if next_time_step > msg_next_time.0 {
                read_only = msg_next_time.1;
                next_time_step = msg_next_time.0;
            }
            debug!("Time step changed with message: {}", next_time_step);
        }
        for state_estimator in self.state_estimator_bench.read().unwrap().iter() {
            next_time_step = next_time_step.min(
                state_estimator
                    .state_estimator
                    .read()
                    .unwrap()
                    .next_time_step(),
            );
        }
        // for service in &self.services {
        //     let tpl = service.read().unwrap().next_time();
        //     if next_time_step > tpl.0 {
        //         read_only = tpl.1;
        //         next_time_step = tpl.0;
        //     }
        // }

        let tpl = self.service_manager.as_ref().unwrap().next_time();
        if next_time_step > tpl.0 {
            read_only = tpl.1;
            next_time_step = tpl.0;
        }
        next_time_step = round_precision(next_time_step, TIME_ROUND).unwrap();
        debug!(
            "next_time_step: {} (read only: {read_only})",
            next_time_step
        );
        (next_time_step, read_only)
    }

    /// Save the current state to the given `time`.
    fn save_state(&mut self, time: f32) {
        self.state_history.insert(time, self.record(), true);
    }

    /// Set the robot in the state just before `time` (but different).
    ///
    /// It should be called for the minimal time before using [`Robot::save_state`].
    pub fn set_in_state(&mut self, time: f32) {
        let state_at_time = self.state_history.get_data_before_time(time);
        if state_at_time.is_none() {
            warn!("No state to be set in at time {time}");
            return;
        }
        let state_at_time = state_at_time.unwrap().1;
        self.from_record(state_at_time.clone());
    }

    /// Returs the current state history.
    pub fn record_history(&self) -> &TimeOrderedData<RobotRecord> {
        &self.state_history
    }

    /// Get the name of the robot.
    pub fn name(&self) -> String {
        self.name.clone()
    }

    /// Get a Arc clone of network module.
    pub fn network(&self) -> Arc<RwLock<Network>> {
        Arc::clone(&self.network)
    }

    /// Get a Arc clone of physics module.
    pub fn physics(&self) -> Arc<RwLock<Box<dyn Physic>>> {
        Arc::clone(&self.physic)
    }

    /// Get a Arc clone of sensor manager.
    pub fn sensor_manager(&self) -> Arc<RwLock<SensorManager>> {
        Arc::clone(&self.sensor_manager)
    }

    /// Get a Arc clone of state estimator module.
    pub fn state_estimator(&self) -> Arc<RwLock<Box<dyn StateEstimator>>> {
        Arc::clone(&self.state_estimator)
    }

    /// Get a Arc clone of state estimator module.
    pub fn state_estimator_bench(&self) -> Arc<RwLock<Vec<BenchStateEstimator>>> {
        Arc::clone(&self.state_estimator_bench)
    }

    /// Get a Arc clone of navigator module.
    pub fn navigator(&self) -> Arc<RwLock<Box<dyn Navigator>>> {
        Arc::clone(&self.navigator)
    }

    /// Get a Arc clone of controller module.
    pub fn controller(&self) -> Arc<RwLock<Box<dyn Controller>>> {
        Arc::clone(&self.controller)
    }

    /// Get a Arc clone of sensor manager.
    pub fn service_manager(&self) -> &ServiceManager {
        self.service_manager.as_ref().unwrap()
    }

    /// Test function to receive a string message
    pub fn message_callback_str(&mut self, message: &Value) -> Result<(), ()> {
        if let Value::String(str_msg) = message {
            info!("I accept to receive your message: {}", str_msg);
            return Ok(());
        } else {
            info!("Use next handler please");
            return Err(());
        }
    }

    /// Test function to receive a number message
    pub fn message_callback_number(&mut self, message: &Value) -> Result<(), ()> {
        if let Value::Number(nbr) = message {
            info!("I accept to receive your message: {}", nbr);
            return Ok(());
        } else {
            info!("Use next handler please");
            return Err(());
        }
    }
}

impl Stateful<RobotRecord> for Robot {
    /// Generate the current state record.
    fn record(&self) -> RobotRecord {
        let mut record = RobotRecord {
            name: self.name.clone(),
            navigator: self.navigator.read().unwrap().record(),
            controller: self.controller.read().unwrap().record(),
            physic: self.physic.read().unwrap().record(),
            state_estimator: self.state_estimator.read().unwrap().record(),
            state_estimator_bench: Vec::new(),
            sensors: self.sensor_manager.read().unwrap().record(),
        };
        let other_state_estimators = self.state_estimator_bench.clone();
        for additional_state_estimator in other_state_estimators.read().unwrap().iter() {
            record
                .state_estimator_bench
                .push(BenchStateEstimatorRecord {
                    name: additional_state_estimator.name.clone(),
                    record: additional_state_estimator
                        .state_estimator
                        .read()
                        .unwrap()
                        .record(),
                });
        }
        record
    }

    /// Change the robot to be in the state of the given `record`.
    fn from_record(&mut self, record: RobotRecord) {
        self.navigator
            .write()
            .unwrap()
            .from_record(record.navigator.clone());
        self.controller()
            .write()
            .unwrap()
            .from_record(record.controller.clone());
        self.physics()
            .write()
            .unwrap()
            .from_record(record.physic.clone());
        self.state_estimator()
            .write()
            .unwrap()
            .from_record(record.state_estimator.clone());
        let other_state_estimators = self.state_estimator_bench.clone();
        for (i, additional_state_estimator) in other_state_estimators
            .write()
            .unwrap()
            .iter_mut()
            .enumerate()
        {
            additional_state_estimator
                .state_estimator
                .write()
                .unwrap()
                .from_record(record.state_estimator_bench[i].record.clone());
        }
    }
}
