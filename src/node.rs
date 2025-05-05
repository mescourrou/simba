/*!
Module providing the main node manager, [`Node`], along with the configuration
[`NodeConfig`] and the record [`NodeRecord`] structures.
*/

use core::f32;
use std::collections::HashMap;
use std::sync::{Arc, Condvar, Mutex, RwLock};

use config_checker::macros::Check;
use log::{debug, info, warn};
use serde::{Deserialize, Serialize};

use super::navigators::navigator::{Navigator, NavigatorConfig, NavigatorRecord};
use super::navigators::trajectory_follower;

use crate::api::internal_api::{self, NodeClient, NodeServer};
use crate::constants::TIME_ROUND;
use crate::controllers::controller::{self, Controller, ControllerConfig, ControllerRecord};

use crate::networking::network::{Network, NetworkConfig};
use crate::networking::service_manager::ServiceManager;
use crate::node_factory::{ComputationUnitRecord, NodeRecord, NodeType, RobotRecord};
use crate::physics::physic::{Physic, PhysicConfig, PhysicRecord};
use crate::physics::{perfect_physic, physic};

use crate::simulator::{SimulatorConfig, TimeCvData};
use crate::state_estimators::state_estimator::{
    BenchStateEstimator, BenchStateEstimatorRecord, StateEstimator, StateEstimatorConfig,
    StateEstimatorRecord,
};
use crate::state_estimators::{perfect_estimator, state_estimator};

use crate::sensors::sensor_manager::{SensorManager, SensorManagerConfig, SensorManagerRecord};

use crate::plugin_api::PluginAPI;
use crate::stateful::Stateful;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::maths::round_precision;
use crate::utils::time_ordered_data::TimeOrderedData;
use crate::{node, time_analysis};

// Node itself

/// Structure managing one node.
///
/// It is composed of modules to manage different aspects:
/// * `navigator` is of [`Navigator`] trait, and defines the error to be sent
/// to the [`Controller`] to follow the required trajectory.
/// * `controller` is of [`Controller`] trait, it defines the command to be sent
/// to the [`Physic`] module.
/// * `physic` is of [`Physic`] trait. It simulates the node behaviour, its real
/// state. It contains a ground truth to evaluate the [`StateEstimator`].
/// * `state_estimator` is of [`StateEstimator`] trait. It estimates the node
/// state, and send it to the [`Navigator`].
///
/// * `sensor_manager`, of type [`SensorManager`], manages the [`Sensor`]s. The
/// observations of the sensors are sent to the [`StateEstimator`].
/// * `network` is the node [`Network`] interface. It manages the reception and
/// the send of messages to other nodes.
///
/// The [`Node`] internally manages a history of its states, using [`TimeOrderedData`].
/// In this way, it can get back to a past state, in order to treat a message sent
/// from the past. [`Node::run_next_time_step`] does the necessary
/// so that the required time is reached taking into account all past messages.
#[derive(Debug)]
pub struct Node {
    pub(crate) node_type: NodeType,
    /// Name of the node. Should be unique among all [`Simulator`](crate::simulator::Simulator)
    /// nodes.
    pub(crate) name: String,
    /// [`Navigator`] module, implementing the navigation strategy.
    pub(crate) navigator: Option<Arc<RwLock<Box<dyn Navigator>>>>,
    /// [`Controller`] module, implementing the control strategy.
    pub(crate) controller: Option<Arc<RwLock<Box<dyn Controller>>>>,
    /// [`Physic`] module, implementing the physics strategy.
    pub(crate) physic: Option<Arc<RwLock<Box<dyn Physic>>>>,
    /// [`StateEstimator`] module, implementing the state estimation strategy.
    pub(crate) state_estimator: Option<Arc<RwLock<Box<dyn StateEstimator>>>>,
    /// Manages all the [`Sensor`]s and send the observations to `state_estimator`.
    pub(crate) sensor_manager: Option<Arc<RwLock<SensorManager>>>,
    /// [`Network`] interface to receive and send messages with other nodes.
    pub(crate) network: Option<Arc<RwLock<Network>>>,
    /// History of the states ([`NodeRecord`]) of the node, to set the [`Node`]
    /// in a past state.
    pub(crate) state_history: TimeOrderedData<NodeRecord>,
    /// Additional [`StateEstimator`] to be evaluated.
    pub(crate) state_estimator_bench: Option<Arc<RwLock<Vec<BenchStateEstimator>>>>,

    // services: Vec<Arc<RwLock<Box<dyn ServiceInterface>>>>,
    /// Not really an option, but for delayed initialization
    pub(crate) service_manager: Option<Arc<RwLock<ServiceManager>>>,

    pub(crate) node_server: Option<NodeServer>,

    pub other_node_names: Vec<String>,
}

impl Node {
    // /// Creates a new [`Node`] with the given configuration.
    // ///
    // /// During the creation of the new node, each module is initialized with its own config.
    // ///
    // ///  ## Arguments
    // /// * `config` -- Scenario config of the node, including all modules. See [`NodeConfig`]
    // /// * `plugin_api` -- Optional [`PluginAPI`] implementation required to use external modules.
    // /// * `meta_config` -- Simulator config.
    // ///
    // /// ## Return
    // /// The new node is return as a reference counter to be shared among threads.
    // pub fn from_config(
    //     config: &NodeConfig,
    //     plugin_api: &Option<Box<&dyn PluginAPI>>,
    //     global_config: &SimulatorConfig,
    //     va_factory: &DeterministRandomVariableFactory,
    //     time_cv: Arc<(Mutex<TimeCvData>, Condvar)>,
    // ) -> Self {
    //     let mut node = Self {
    //         name: config.name.clone(),
    //         navigator: navigator::make_navigator_from_config(
    //             &config.navigator,
    //             plugin_api,
    //             global_config,
    //             va_factory,
    //         ),
    //         controller: controller::make_controller_from_config(
    //             &config.controller,
    //             plugin_api,
    //             global_config,
    //             va_factory,
    //         ),
    //         physic: physic::make_physic_from_config(
    //             &config.physic,
    //             plugin_api,
    //             global_config,
    //             va_factory,
    //         ),
    //         state_estimator: Arc::new(RwLock::new(
    //             state_estimator::make_state_estimator_from_config(
    //                 &config.state_estimator,
    //                 plugin_api,
    //                 global_config,
    //                 va_factory,
    //             ),
    //         )),
    //         sensor_manager: Arc::new(RwLock::new(SensorManager::from_config(
    //             &config.sensor_manager,
    //             plugin_api,
    //             global_config,
    //             &config.name,
    //             va_factory,
    //         ))),
    //         network: Arc::new(RwLock::new(Network::from_config(
    //             config.name.clone(),
    //             &config.network,
    //             global_config,
    //             va_factory,
    //             time_cv.clone(),
    //         ))),
    //         state_history: TimeOrderedData::new(),
    //         state_estimator_bench: Arc::new(RwLock::new(Vec::with_capacity(
    //             config.state_estimator_bench.len(),
    //         ))),
    //         // services: Vec::new(),
    //         service_manager: None,
    //         node_server: None,
    //         other_node_names: Vec::new(),
    //     };

    //     for state_estimator_config in &config.state_estimator_bench {
    //         node
    //             .state_estimator_bench
    //             .write()
    //             .unwrap()
    //             .push(BenchStateEstimator {
    //                 name: state_estimator_config.name.clone(),
    //                 state_estimator: Arc::new(RwLock::new(
    //                     state_estimator::make_state_estimator_from_config(
    //                         &state_estimator_config.config,
    //                         plugin_api,
    //                         global_config,
    //                         va_factory,
    //                     ),
    //                 )),
    //             })
    //     }

    //     let service_manager = Some(Arc::new(RwLock::new(ServiceManager::initialize(
    //         &node, time_cv,
    //     ))));
    //     if plugin_api.is_some() {
    //         if let Some(message_handlers) =
    //             plugin_api.as_ref().unwrap().get_message_handlers(&node)
    //         {
    //             let mut network = node.network.write().unwrap();
    //             for message_handler in message_handlers {
    //                 network.subscribe(message_handler.clone());
    //             }
    //         }
    //     }
    //     // Services
    //     debug!("Setup services");
    //     node.service_manager = service_manager;
    //     node.save_state(0.);

    //     node
    // }

    /// Initialize the node after its creation.
    ///
    /// It is used to initialize the sensor manager, which need to know the list of all nodes.
    pub fn post_creation_init(
        &mut self,
        service_manager_list: &HashMap<String, Arc<RwLock<ServiceManager>>>,
    ) -> NodeClient {
        let service_manager = self.service_manager();
        service_manager
            .write()
            .unwrap()
            .make_links(service_manager_list, self);
        if let Some(sensor_manager) = self.sensor_manager() {
            sensor_manager.write().unwrap().init(self);
        }

        self.other_node_names = service_manager_list
            .iter()
            .filter_map(|n| {
                if n.0 != &self.name {
                    Some(n.0.clone())
                } else {
                    None
                }
            })
            .collect();

        if let Some(network) = &self.network {
            if let Some(sensor_manager) = &self.sensor_manager {
                network.write().unwrap().subscribe(sensor_manager.clone());
            }
        }
        let (node_server, node_client) = internal_api::make_node_api(&self.node_type);
        self.node_server = Some(node_server);
        debug!("Save initial state");
        self.save_state(0.);
        node_client
    }

    /// Run the node to reach the given time.
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
        let mut nb_msg = 0;
        if let Some(network) = self.network() {
            nb_msg += network.write().unwrap().process_messages().unwrap();
        }
        nb_msg += self
            .service_manager
            .as_ref()
            .unwrap()
            .read()
            .unwrap()
            .process_requests();
        nb_msg
    }

    /// Run only one time step.
    ///
    /// To run the given `time` step, the node sets itself in the state of this moment
    /// using [`Node::set_in_state`].
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
    /// Then, the node state is saved.
    pub fn run_time_step(&mut self, time: f32, read_only: bool) {
        info!("Run time {}", time);
        self.set_in_state(time);
        // Update the true state
        if let Some(physics) = &self.physic {
            physics.write().unwrap().update_state(time);
            self.node_server
                .as_ref()
                .unwrap()
                .state_update
                .as_ref()
                .unwrap()
                .send((time, physics.read().unwrap().state(time).clone()))
                .unwrap();
        }

        if let Some(sensor_manager) = &self.sensor_manager() {
            // Make observations (if it is the right time)
            let observations = sensor_manager.write().unwrap().get_observations(self, time);

            debug!("Got {} observations", observations.len());
            if observations.len() > 0 {
                // Treat the observations
                if let Some(state_estimator) = &self.state_estimator() {
                    let ta = time_analysis::time_analysis(
                        time,
                        "control_loop_state_estimator_correction_step".to_string(),
                    );
                    state_estimator
                        .write()
                        .unwrap()
                        .correction_step(self, &observations, time);
                    time_analysis::finished_time_analysis(ta);
                }

                if let Some(state_estimator_bench) = &self.state_estimator_bench() {
                    for state_estimator in state_estimator_bench.read().unwrap().iter() {
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
            }
        }

        // If it is time for the state estimator to do the prediction
        if let Some(state_estimator) = &self.state_estimator() {
            if time >= state_estimator.read().unwrap().next_time_step() {
                // Prediction step
                let ta = time_analysis::time_analysis(
                    time,
                    "control_loop_state_estimator_prediction_step".to_string(),
                );
                state_estimator.write().unwrap().prediction_step(self, time);
                time_analysis::finished_time_analysis(ta);
                let state = state_estimator.read().unwrap().state();

                // Compute the error to the planned path
                let ta = time_analysis::time_analysis(
                    time,
                    "control_loop_navigator_compute_error".to_string(),
                );
                let error = self
                    .navigator()
                    .as_ref()
                    .unwrap()
                    .write()
                    .unwrap()
                    .compute_error(self, state);
                time_analysis::finished_time_analysis(ta);

                // Compute the command from the error
                let ta = time_analysis::time_analysis(
                    time,
                    "control_loop_controller_make_command".to_string(),
                );
                let command = self
                    .controller()
                    .as_ref()
                    .unwrap()
                    .write()
                    .unwrap()
                    .make_command(self, &error, time);
                time_analysis::finished_time_analysis(ta);

                // Apply the command to the physics
                self.physic
                    .as_ref()
                    .unwrap()
                    .write()
                    .unwrap()
                    .apply_command(&command, time);
            }
        }

        if let Some(state_estimator_bench) = &self.state_estimator_bench() {
            for state_estimator in state_estimator_bench.read().unwrap().iter() {
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
        }

        self.handle_messages(time);

        if !read_only {
            // Save state (replace if needed)
            self.save_state(time);
        }
    }

    pub fn handle_messages(&mut self, time: f32) {
        // Treat messages synchronously
        if let Some(network) = self.network() {
            network.write().unwrap().handle_message_at_time(self, time);
        }
        self.service_manager
            .as_ref()
            .unwrap()
            .write()
            .unwrap()
            .handle_requests(time);
    }

    /// Computes the next time step, using state estimator, sensors and received messages.
    pub fn next_time_step(&self) -> (f32, bool) {
        let mut next_time_step = f32::INFINITY;
        if let Some(state_estimator) = &self.state_estimator {
            next_time_step = state_estimator
                .read()
                .unwrap()
                .next_time_step()
                .min(next_time_step);
            debug!("Next time after state estimator: {next_time_step}");
        }

        if let Some(sensor_manager) = &self.sensor_manager {
            next_time_step = sensor_manager
                .read()
                .unwrap()
                .next_time_step()
                .unwrap_or(f32::INFINITY)
                .min(next_time_step);
            debug!("Next time after sensor manager: {next_time_step}");
        }
        let mut read_only = false;

        if let Some(network) = &self.network {
            let message_next_time = network.read().unwrap().next_message_time();
            debug!(
                "In node: message_next_time: {}",
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
            debug!("Next time after network: {next_time_step}");
        }
        if let Some(state_estimator_bench) = &self.state_estimator_bench {
            for state_estimator in state_estimator_bench.read().unwrap().iter() {
                next_time_step = next_time_step.min(
                    state_estimator
                        .state_estimator
                        .read()
                        .unwrap()
                        .next_time_step(),
                );
            }
            debug!("Next time after state estimator bench: {next_time_step}");
        }

        let tpl = self
            .service_manager
            .as_ref()
            .unwrap()
            .read()
            .unwrap()
            .next_time();
        if next_time_step > tpl.0 {
            read_only = tpl.1;
            next_time_step = tpl.0;
        }
        debug!("Next time after service manager: {next_time_step}");
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

    /// Set the node in the state just before `time` (but different).
    ///
    /// It should be called for the minimal time before using [`Node::save_state`].
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
    pub fn record_history(&self) -> &TimeOrderedData<NodeRecord> {
        &self.state_history
    }

    /// Get the name of the node.
    pub fn name(&self) -> String {
        self.name.clone()
    }

    /// Get a Arc clone of network module.
    pub fn network(&self) -> Option<Arc<RwLock<Network>>> {
        match &self.network {
            Some(n) => Some(Arc::clone(n)),
            None => None,
        }
    }

    /// Get a Arc clone of physics module.
    pub fn physics(&self) -> Option<Arc<RwLock<Box<dyn Physic>>>> {
        match &self.physic {
            Some(p) => Some(Arc::clone(p)),
            None => None,
        }
    }

    /// Get a Arc clone of sensor manager.
    pub fn sensor_manager(&self) -> Option<Arc<RwLock<SensorManager>>> {
        match &self.sensor_manager {
            Some(sm) => Some(Arc::clone(sm)),
            None => None,
        }
    }

    /// Get a Arc clone of state estimator module.
    pub fn state_estimator(&self) -> Option<Arc<RwLock<Box<dyn StateEstimator>>>> {
        match &self.state_estimator {
            Some(se) => Some(Arc::clone(se)),
            None => None,
        }
    }

    /// Get a Arc clone of state estimator module.
    pub fn state_estimator_bench(&self) -> Option<Arc<RwLock<Vec<BenchStateEstimator>>>> {
        match &self.state_estimator_bench {
            Some(se) => Some(Arc::clone(se)),
            None => None,
        }
    }

    /// Get a Arc clone of navigator module.
    pub fn navigator(&self) -> Option<Arc<RwLock<Box<dyn Navigator>>>> {
        match &self.navigator {
            Some(n) => Some(Arc::clone(n)),
            None => None,
        }
    }

    /// Get a Arc clone of controller module.
    pub fn controller(&self) -> Option<Arc<RwLock<Box<dyn Controller>>>> {
        match &self.controller {
            Some(c) => Some(Arc::clone(c)),
            None => None,
        }
    }

    /// Get a Arc clone of Service Manager.
    pub fn service_manager(&self) -> Arc<RwLock<ServiceManager>> {
        self.service_manager.as_ref().unwrap().clone()
    }

    fn robot_record(&self) -> RobotRecord {
        let mut record = RobotRecord {
            name: self.name.clone(),
            navigator: self.navigator.as_ref().unwrap().read().unwrap().record(),
            controller: self.controller.as_ref().unwrap().read().unwrap().record(),
            physic: self.physic.as_ref().unwrap().read().unwrap().record(),
            state_estimator: self
                .state_estimator
                .as_ref()
                .unwrap()
                .read()
                .unwrap()
                .record(),
            state_estimator_bench: Vec::new(),
            sensors: self
                .sensor_manager
                .as_ref()
                .unwrap()
                .read()
                .unwrap()
                .record(),
        };
        let other_state_estimators = self.state_estimator_bench.clone();
        for additional_state_estimator in other_state_estimators
            .as_ref()
            .unwrap()
            .read()
            .unwrap()
            .iter()
        {
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

    fn computation_unit_record(&self) -> ComputationUnitRecord {
        let mut record = ComputationUnitRecord {
            name: self.name.clone(),
            state_estimators: Vec::new(),
            sensor_manager: self.sensor_manager().unwrap().read().unwrap().record(),
        };
        let other_state_estimators = self.state_estimator_bench.clone();
        for additional_state_estimator in other_state_estimators
            .as_ref()
            .unwrap()
            .read()
            .unwrap()
            .iter()
        {
            record.state_estimators.push(BenchStateEstimatorRecord {
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
}

impl Stateful<NodeRecord> for Node {
    /// Generate the current state record.
    fn record(&self) -> NodeRecord {
        match &self.node_type {
            NodeType::Robot => NodeRecord::Robot(self.robot_record()),
            NodeType::ComputationUnit => {
                NodeRecord::ComputationUnit(self.computation_unit_record())
            }
            _ => unimplemented!(),
        }
    }

    /// Change the node to be in the state of the given `record`.
    fn from_record(&mut self, record: NodeRecord) {
        let node_type = record.as_node_type();
        assert!(node_type == self.node_type);

        if node_type.has_navigator() {
            self.navigator()
                .unwrap()
                .write()
                .unwrap()
                .from_record(record.navigator().unwrap().clone());
        }
        if node_type.has_controller() {
            self.controller()
                .unwrap()
                .write()
                .unwrap()
                .from_record(record.controller().unwrap().clone());
        }
        if node_type.has_physics() {
            self.physics()
                .unwrap()
                .write()
                .unwrap()
                .from_record(record.physics().unwrap().clone());
        }
        if node_type.has_state_estimator() {
            self.state_estimator()
                .unwrap()
                .write()
                .unwrap()
                .from_record(record.state_estimator().unwrap().clone());
        }
        if node_type.has_state_estimator_bench() {
            let other_state_estimators = self.state_estimator_bench.clone();
            let state_estimator_bench_record = record.state_estimator_bench().unwrap();
            for (i, additional_state_estimator) in other_state_estimators
                .unwrap()
                .write()
                .unwrap()
                .iter_mut()
                .enumerate()
            {
                additional_state_estimator
                    .state_estimator
                    .write()
                    .unwrap()
                    .from_record(state_estimator_bench_record[i].record.clone());
            }
        }
    }
}
