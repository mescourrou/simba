/*!
Module providing the main robot manager, [`Turtlebot`], along with the configuration
[`TurtlebotConfig`] and the record [`TurtlebotRecord`] structures.
*/

use std::sync::{Arc, Condvar, Mutex, RwLock};

use super::navigators::navigator::{Navigator, NavigatorConfig, NavigatorRecord};
use super::navigators::trajectory_follower;

use crate::controllers::controller::{self, Controller, ControllerConfig, ControllerRecord};
use crate::controllers::pid;

use crate::navigators::navigator;
use crate::networking::message_handler::MessageHandler;
use crate::networking::network::{Network, NetworkConfig};
use crate::physics::physic::{Physic, PhysicConfig, PhysicRecord};
use crate::physics::{perfect_physic, physic};

use crate::sensors::sensor::Sensor;
use crate::state_estimators::state_estimator::{
    StateEstimator, StateEstimatorConfig, StateEstimatorRecord,
};
use crate::state_estimators::{perfect_estimator, state_estimator};

use crate::sensors::sensor_manager::{SensorManager, SensorManagerConfig, SensorManagerRecord};

use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorMetaConfig;
use crate::stateful::Stateful;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::time_ordered_data::TimeOrderedData;

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct BenchStateEstimatorConfig {
    name: String,
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
#[pyclass(get_all)]
pub struct BenchStateEstimatorRecord {
    name: String,
    record: StateEstimatorRecord,
}

#[derive(Debug)]
pub struct BenchStateEstimator {
    name: String,
    state_estimator: Arc<RwLock<Box<dyn StateEstimator>>>,
}

// Configuration for Turtlebot
extern crate confy;
use log::{debug, info, warn};
use pyo3::prelude::*;
use serde_derive::{Deserialize, Serialize};
use serde_json::Value;

/// Configuration of the [`Turtlebot`].
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct TurtlebotConfig {
    /// Name of the robot.
    pub name: String,
    /// [`Navigator`] to use, and its configuration.
    pub navigator: NavigatorConfig,
    /// [`Controller`] to use, and its configuration.
    pub controller: ControllerConfig,
    /// [`Physic`] to use, and its configuration.
    pub physic: PhysicConfig,
    /// [`StateEstimator`] to use, and its configuration.
    pub state_estimator: StateEstimatorConfig,
    /// [`SensorManager`] configuration, which defines the [`Sensor`]s used.
    pub sensor_manager: SensorManagerConfig,
    /// [`Network`] configuration.
    pub network: NetworkConfig,

    /// Additional [`StateEstimator`] to be evaluated but without a feedback
    /// loop with the [`Navigator`]
    pub state_estimator_bench: Vec<BenchStateEstimatorConfig>,
}

impl Default for TurtlebotConfig {
    /// Default configuration, using:
    /// * Default [`TrajectoryFollower`](trajectory_follower::TrajectoryFollower) navigator.
    /// * Default [`PID`](pid::PID) controller.
    /// * Default [`PerfectPhysic`](perfect_physic::PerfectPhysic) physics.
    /// * Default [`PerfectEstimator`](perfect_estimator::PerfectEstimator) state estimator.
    /// * Default [`SensorManager`] config (no sensors).
    /// * Default [`Network`] config.
    fn default() -> Self {
        TurtlebotConfig {
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

/// State record of [`Turtlebot`].
///
/// It contains the dynamic elements and the elements we want to save.
#[derive(Debug, Serialize, Deserialize, Clone)]
#[pyclass(get_all)]
pub struct TurtlebotRecord {
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
pub struct TurtlebotGenericMessageHandler {}

impl TurtlebotGenericMessageHandler {
    /// Create a new instance of [`TurtlebotGenericMessageHandler`].
    pub fn new() -> Self {
        TurtlebotGenericMessageHandler {}
    }
}

impl MessageHandler for TurtlebotGenericMessageHandler {
    /// Handle the message or raise an error (to send the message to the next handler).
    ///
    /// See [`MessageHandler::handle_message`] for trait specification.
    fn handle_message(
        &mut self,
        turtle: &mut Turtlebot,
        _from: &String,
        message: &Value,
        _time: f32,
    ) -> Result<(), ()> {
        if turtle.message_callback_str(&message).is_ok() {
            return Ok(());
        } else if turtle.message_callback_number(&message).is_ok() {
            return Ok(());
        }
        Err(())
    }
}

// Turtlebot itself

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
/// The [`Turtlebot`] internally manages a history of its states, using [`TimeOrderedData`].
/// In this way, it can get back to a past state, in order to treat a message sent
/// from the past. [`Turtlebot::run_next_time_step`] does the necessary
/// so that the required time is reached taking into account all past messages.
#[derive(Debug, Clone)]
#[pyclass]
pub struct Turtlebot {
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
    message_handler: Arc<RwLock<TurtlebotGenericMessageHandler>>,
    /// History of the states ([`TurtlebotRecord`]) of the robot, to set the [`Turtlebot`]
    /// in a past state.
    state_history: TimeOrderedData<TurtlebotRecord>,
    /// Additional [`StateEstimator`] to be evaluated.
    state_estimator_bench: Arc<RwLock<Vec<BenchStateEstimator>>>,
}

impl Turtlebot {
    /// Creates a new [`Turtlebot`] with the given name.
    pub fn new(name: String, time_cv: Arc<(Mutex<usize>, Condvar)>) -> Arc<RwLock<Self>> {
        let turtle = Arc::new(RwLock::new(Self {
            name: name.clone(),
            navigator: Arc::new(RwLock::new(Box::new(
                trajectory_follower::TrajectoryFollower::new(),
            ))),
            controller: Arc::new(RwLock::new(Box::new(pid::PID::new()))),
            physic: Arc::new(RwLock::new(Box::new(perfect_physic::PerfectPhysic::new(
                time_cv.clone(),
            )))),
            state_estimator: Arc::new(RwLock::new(Box::new(
                perfect_estimator::PerfectEstimator::new(),
            ))),
            sensor_manager: Arc::new(RwLock::new(SensorManager::new())),
            network: Arc::new(RwLock::new(Network::new(name.clone(), time_cv.clone()))),
            message_handler: Arc::new(RwLock::new(TurtlebotGenericMessageHandler::new())),
            state_history: TimeOrderedData::new(),
            state_estimator_bench: Arc::new(RwLock::new(Vec::new())),
        }));
        turtle.write().unwrap().save_state(0.);
        turtle
    }

    /// Creates a new [`Turtlebot`] with the given configuration.
    ///
    /// During the creation of the new robot, each module is initialized with its own config.
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the robot, including all modules. See [`TurtlebotConfig`]
    /// * `plugin_api` -- Optional [`PluginAPI`] implementation required to use external modules.
    /// * `meta_config` -- Simulator config.
    ///
    /// ## Return
    /// The new robot is return as a reference counter to be shared among threads.
    pub fn from_config(
        config: &TurtlebotConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        meta_config: SimulatorMetaConfig,
        va_factory: &DeterministRandomVariableFactory,
        time_cv: Arc<(Mutex<usize>, Condvar)>,
    ) -> Arc<RwLock<Self>> {
        let turtle = Arc::new(RwLock::new(Self {
            name: config.name.clone(),
            navigator: navigator::make_navigator_from_config(
                &config.navigator,
                plugin_api,
                meta_config.clone(),
                va_factory,
            ),
            controller: controller::make_controller_from_config(
                &config.controller,
                plugin_api,
                meta_config.clone(),
                va_factory,
            ),
            physic: physic::make_physic_from_config(
                &config.physic,
                plugin_api,
                meta_config.clone(),
                va_factory,
                time_cv.clone(),
            ),
            state_estimator: state_estimator::make_state_estimator_from_config(
                &config.state_estimator,
                plugin_api,
                meta_config.clone(),
                va_factory,
            ),
            sensor_manager: Arc::new(RwLock::new(SensorManager::from_config(
                &config.sensor_manager,
                plugin_api,
                meta_config.clone(),
                va_factory,
            ))),
            network: Arc::new(RwLock::new(Network::from_config(
                config.name.clone(),
                &config.network,
                meta_config.clone(),
                va_factory,
                time_cv.clone(),
            ))),
            message_handler: Arc::new(RwLock::new(TurtlebotGenericMessageHandler::new())),
            state_history: TimeOrderedData::new(),
            state_estimator_bench: Arc::new(RwLock::new(Vec::with_capacity(
                config.state_estimator_bench.len(),
            ))),
            // services_handles: Vec::new(),
        }));

        for state_estimator_config in &config.state_estimator_bench {
            turtle
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
                        meta_config.clone(),
                        va_factory,
                    ),
                })
        }

        {
            let mut writable_turtle = turtle.write().unwrap();
            writable_turtle.network.write().unwrap().subscribe(Arc::<
                RwLock<TurtlebotGenericMessageHandler>,
            >::clone(
                &writable_turtle.message_handler
            ));
            writable_turtle.save_state(0.);
        }
        debug!("[{}] Setup services", turtle.read().unwrap().name());
        // Services
        let physics = turtle.write().unwrap().physics();
        physics.write().unwrap().make_service(turtle.clone());

        turtle
    }

    /// Initialize the robot after its creation.
    /// 
    /// It is used to initialize the sensor manager, which need to know the list of all robots.
    pub fn post_creation_init(
        &mut self,
        turtle_list: &Arc<RwLock<Vec<Arc<RwLock<Turtlebot>>>>>,
        turtle_idx: usize,
    ) {
        let sensor_manager = self.sensor_manager();
        sensor_manager
            .write()
            .unwrap()
            .init(self, turtle_list, turtle_idx);
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
    pub fn run_next_time_step(&mut self, time: f32) -> f32 {
        let mut next_time_step = self.next_time_step();
        while next_time_step <= time {
            self.process_messages();
            self.run_time_step(next_time_step);
            next_time_step = self.next_time_step();
        }

        next_time_step
    }

    /// Process all the messages: one-way (network) and two-way (services).
    pub fn process_messages(&self) -> usize {
        self.network().write().unwrap().process_messages()
            + self.physics().write().unwrap().process_service_requests()
    }

    /// Run only one time step.
    ///
    /// To run the given `time` step, the robot sets itself in the state of this moment
    /// using [`Turtlebot::set_in_state`].
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
    pub fn run_time_step(&mut self, time: f32) {
        self.set_in_state(time);
        info!("[{}] Run time {}", self.name(), time);
        // Update the true state
        self.physic.write().unwrap().update_state(time);

        // Make observations (if it is the right time)
        let observations = self
            .sensor_manager()
            .write()
            .unwrap()
            .get_observations(self, time);

        debug!("[{}] Got {} observations", self.name, observations.len());
        if observations.len() > 0 {
            // Treat the observations
            self.state_estimator()
                .write()
                .unwrap()
                .correction_step(self, &observations, time);

            for state_estimator in self.state_estimator_bench().read().unwrap().iter() {
                state_estimator
                    .state_estimator
                    .write()
                    .unwrap()
                    .correction_step(self, &observations, time);
            }
        }

        // If it is time for the state estimator to do the prediction
        if time >= self.state_estimator.read().unwrap().next_time_step() {
            // Prediction step
            self.state_estimator()
                .write()
                .unwrap()
                .prediction_step(self, time);
            let state = self.state_estimator.read().unwrap().state();

            // Compute the error to the planned path
            let error = self.navigator().write().unwrap().compute_error(self, state);

            // Compute the command from the error
            let command = self
                .controller()
                .write()
                .unwrap()
                .make_command(self, &error, time);

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
                state_estimator
                    .state_estimator
                    .write()
                    .unwrap()
                    .prediction_step(self, time);
            }
        }

        // Treat messages synchronously
        self.network()
            .write()
            .unwrap()
            .handle_message_at_time(self, time);

        self.physics()
            .write()
            .unwrap()
            .handle_service_requests(time);

        // Save state (replace if needed)
        self.save_state(time);
    }

    /// Computes the next time step, using state estimator, sensors and received messages.
    pub fn next_time_step(&self) -> f32 {
        let mut next_time_step = self
            .state_estimator
            .read()
            .unwrap()
            .next_time_step()
            .min(self.sensor_manager.read().unwrap().next_time_step().unwrap_or(f32::INFINITY));

        let message_next_time = self.network().read().unwrap().next_message_time();
        debug!(
            "[{}] In turtlebot: message_next_time: {}",
            self.name(),
            message_next_time.unwrap_or(-1.)
        );
        if let Some(msg_next_time) = message_next_time {
            next_time_step = next_time_step.min(msg_next_time);
            debug!(
                "[{}] Time step changed with message: {}",
                self.name(),
                next_time_step
            );
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

        next_time_step = next_time_step.min(self.physic.read().unwrap().service_next_time());

        debug!("[{}] next_time_step: {}", self.name(), next_time_step);
        next_time_step
    }

    /// Save the current state to the given `time`.
    fn save_state(&mut self, time: f32) {
        self.state_history.insert(time, self.record(), true);
    }

    /// Set the robot in the state just before `time` (but different).
    ///
    /// It should be called for the minimal time before using [`Turtlebot::save_state`].
    fn set_in_state(&mut self, time: f32) {
        let state_at_time = self.state_history.get_data_before_time(time);
        if state_at_time.is_none() {
            warn!("[{}] No state to be set in at time {time}", self.name());
            return;
        }
        let state_at_time = state_at_time.unwrap().1;
        self.from_record(state_at_time.clone());
    }

    /// Returs the current state history.
    pub fn record_history(&self) -> &TimeOrderedData<TurtlebotRecord> {
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

impl Stateful<TurtlebotRecord> for Turtlebot {
    /// Generate the current state record.
    fn record(&self) -> TurtlebotRecord {
        let mut record = TurtlebotRecord {
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
    fn from_record(&mut self, record: TurtlebotRecord) {
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
