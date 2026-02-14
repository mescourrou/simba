/*!
Module providing the main node manager, [`Node`]. The building of the Nodes is done by [`NodeFactory`](crate::node_factory::NodeFactory).
*/

pub mod node_factory;

use node_factory::{ComputationUnitRecord, NodeRecord, NodeType, RobotRecord};
use serde::{Deserialize, Serialize};
use simba_com::pub_sub::{MultiClientTrait, PathKey};
use simba_macros::EnumToString;

use core::f32;
use std::collections::BTreeMap;
use std::str::FromStr;
use std::sync::Arc;

use log::{debug, info};

use crate::errors::{SimbaError, SimbaErrorTypes};
use crate::networking;
use crate::networking::network::MessageFlag;
use crate::simulator::SimbaBrokerMultiClient;
use crate::state_estimators::State;
use crate::time_analysis::TimeAnalysisNode;
use crate::utils::read_only_lock::RoLock;
use crate::utils::{SharedMutex, SharedRoLock, SharedRwLock};
use crate::{
    api::internal_api::{self, NodeClient, NodeServer},
    constants::TIME_ROUND,
    controllers::Controller,
    errors::SimbaResult,
    logger::is_enabled,
    navigators::Navigator,
    networking::network::Network,
    networking::service_manager::ServiceManager,
    physics::Physics,
    recordable::Recordable,
    sensors::sensor_manager::SensorManager,
    simulator::TimeCv,
    state_estimators::{BenchStateEstimator, BenchStateEstimatorRecord, StateEstimator},
    utils::maths::round_precision,
};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, EnumToString)]
pub enum NodeState {
    Created,
    Running,
    Zombie,
    Terminated,
}

#[derive(Debug, Clone)]
pub struct NodeMetaData {
    pub name: String,
    pub node_type: NodeType,
    pub model_name: String,
    pub labels: Vec<String>,
    pub state: NodeState,
    pub position: Option<[f32; 2]>,
}

// Node itself

/// Structure managing one node.
///
/// It is composed of modules to manage different aspects:
/// * `navigator` is of [`Navigator`] trait, and defines the error to be sent
///   to the [`Controller`] to follow the required trajectory.
/// * `controller` is of [`Controller`] trait, it defines the command to be sent
///   to the [`Physics`] module.
/// * `physics` is of [`Physics`] trait. It simulates the node behaviour, its real
///   state. It contains a ground truth to evaluate the [`StateEstimator`].
/// * `state_estimator` is of [`StateEstimator`] trait. It estimates the node
///   state, and send it to the [`Navigator`].
///
/// * `sensor_manager`, of type [`SensorManager`], manages the [`Sensor`](crate::sensors::sensor::Sensor)s. The
///   observations of the sensors are sent to the [`StateEstimator`].
/// * `network` is the node [`Network`] interface. It manages the reception and
///   the send of messages to other nodes.
///
/// The [`Node`] internally manages a history of its states, using [`TimeOrderedData`].
/// In this way, it can get back to a past state, in order to treat a message sent
/// from the past. [`Node::run_next_time_step`] does the necessary
/// so that the required time is reached taking into account all past messages.
#[derive(Debug)]
pub struct Node {
    /// [`Navigator`] module, implementing the navigation strategy.
    pub(self) navigator: Option<SharedRwLock<Box<dyn Navigator>>>,
    /// [`Controller`] module, implementing the control strategy.
    pub(self) controller: Option<SharedRwLock<Box<dyn Controller>>>,
    /// [`Physics`] module, implementing the physics strategy.
    pub(self) physics: Option<SharedRwLock<Box<dyn Physics>>>,
    /// [`StateEstimator`] module, implementing the state estimation strategy.
    pub(self) state_estimator: Option<SharedRwLock<Box<dyn StateEstimator>>>,
    /// Manages all the [`Sensor`](crate::sensors::sensor::Sensor)s and send the observations to `state_estimator`.
    pub(self) sensor_manager: Option<SharedRwLock<SensorManager>>,
    /// [`Network`] interface to receive and send messages with other nodes.
    pub(self) network: Option<SharedRwLock<Network>>,
    /// Additional [`StateEstimator`] to be evaluated.
    pub(self) state_estimator_bench: Option<SharedRwLock<Vec<BenchStateEstimator>>>,

    // services: Vec<SharedRwLock<Box<dyn ServiceInterface>>>>,
    /// Not really an option, but for delayed initialization
    pub(self) service_manager: Option<SharedRwLock<ServiceManager>>,

    pub(self) node_server: Option<NodeServer>,

    pub(self) other_node_names: Vec<String>,
    pub(self) time_analysis: Option<SharedMutex<TimeAnalysisNode>>,
    pub(self) send_records: bool,

    pub(self) node_meta_data: SharedRwLock<NodeMetaData>,
    pub(self) meta_data_list: Option<SharedRoLock<BTreeMap<String, SharedRoLock<NodeMetaData>>>>,
    pub(self) node_message_client: SimbaBrokerMultiClient,
}

impl Node {
    /// Initialize the node after its creation.
    ///
    /// It is used to initialize the sensor manager, which need to know the list of all nodes.
    pub fn post_creation_init(
        &mut self,
        service_manager_list: &BTreeMap<String, SharedRwLock<ServiceManager>>,
        meta_data_list: SharedRoLock<BTreeMap<String, SharedRoLock<NodeMetaData>>>,
        initial_time: f32,
    ) -> NodeClient {
        if is_enabled(crate::logger::InternalLog::SetupSteps) {
            debug!("Node post-creation initialization")
        }
        let service_manager = self.service_manager();
        service_manager
            .write()
            .unwrap()
            .make_links(service_manager_list, self);
        if let Some(sensor_manager) = self.sensor_manager() {
            sensor_manager.write().unwrap().init(self, initial_time);
        }

        self.other_node_names = service_manager_list
            .iter()
            .filter_map(|n| {
                if n.0 != &self.node_meta_data.read().unwrap().name {
                    Some(n.0.clone())
                } else {
                    None
                }
            })
            .collect();

        // if let Some(network) = &self.network {
        //     if let Some(sensor_manager) = &self.sensor_manager {
        //         network
        //             .write()
        //             .unwrap()
        //             .subscribe(sensor_manager.read().unwrap().get_letter_box());
        //     }
        //     if let Some(state_estimator) = &self.state_estimator {
        //         network
        //             .write()
        //             .unwrap()
        //             .subscribe(state_estimator.read().unwrap().get_letter_box());
        //     }
        //     if let Some(state_estimator_bench) = &self.state_estimator_bench {
        //         for state_estimator in state_estimator_bench.read().unwrap().iter() {
        //             network.write().unwrap().subscribe(
        //                 state_estimator
        //                     .state_estimator
        //                     .read()
        //                     .unwrap()
        //                     .get_letter_box(),
        //             );
        //         }
        //     }
        //     if let Some(navigator) = &self.navigator {
        //         network
        //             .write()
        //             .unwrap()
        //             .subscribe(navigator.read().unwrap().get_letter_box());
        //     }
        //     if let Some(controller) = &self.controller {
        //         network
        //             .write()
        //             .unwrap()
        //             .subscribe(controller.read().unwrap().get_letter_box());
        //     }
        // }
        let (node_server, node_client) =
            internal_api::make_node_api(&self.node_meta_data.read().unwrap().node_type);
        self.node_server = Some(node_server);
        {
            let meta_data = &mut self.node_meta_data.write().unwrap();
            let name = meta_data.name.clone();
            let model_name = meta_data.model_name.clone();
            meta_data.labels.push(name);
            meta_data.labels.push(model_name);
        }
        self.meta_data_list = Some(meta_data_list);
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
    pub fn run_next_time_step(&mut self, time: f32, time_cv: &TimeCv) -> SimbaResult<()> {
        self.process_messages();
        self.run_time_step(time, time_cv)
    }

    /// Process all the messages: one-way (network) and two-way (services).
    pub fn process_messages(&self) -> usize {
        let mut nb_msg = 0;
        nb_msg += self
            .service_manager
            .as_ref()
            .unwrap()
            .read()
            .unwrap()
            .process_requests();
        nb_msg += self.node_message_client.next_message_time().is_some() as usize;
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
    pub fn run_time_step(&mut self, time: f32, time_cv: &TimeCv) -> SimbaResult<()> {
        if self.node_meta_data.read().unwrap().state != NodeState::Running {
            return Err(SimbaError::new(
                SimbaErrorTypes::ImplementationError,
                "Only a Running node should be run!".to_string(),
            ));
        }
        info!("Run time {}", time);

        // Update the true state
        if let Some(physics) = &self.physics {
            physics.write().unwrap().update_state(time);
            let pose = physics.read().unwrap().state(time).pose;
            self.node_meta_data.write().unwrap().position = Some([pose[0], pose[1]]);
        }

        self.sync_with_others(time_cv, time);

        // Pre loop calls to manage messages
        if let Some(state_estimator) = self.state_estimator() {
            state_estimator.write().unwrap().pre_loop_hook(self, time);
        }
        if let Some(state_estimator_bench) = self.state_estimator_bench.clone() {
            for state_estimator in state_estimator_bench.read().unwrap().iter() {
                state_estimator
                    .state_estimator
                    .write()
                    .unwrap()
                    .pre_loop_hook(self, time);
            }
        }
        if let Some(controller) = self.controller() {
            controller.write().unwrap().pre_loop_hook(self, time);
        }
        if let Some(navigator) = self.navigator() {
            navigator.write().unwrap().pre_loop_hook(self, time);
        }

        if let Some(sensor_manager) = &self.sensor_manager() {
            sensor_manager.write().unwrap().handle_messages(time);
        }
        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            debug!("Pre prediction step wait");
        }
        self.sync_with_others(time_cv, time);

        let mut do_control_loop = false;

        // If it is time for the state estimator to do the prediction
        if let Some(state_estimator) = &self.state_estimator()
            && time >= state_estimator.read().unwrap().next_time_step()
        {
            // Prediction step
            let ta = self.time_analysis.as_ref().map(|time_analysis| {
                time_analysis.lock().unwrap().time_analysis(
                    time,
                    "control_loop_state_estimator_prediction_step".to_string(),
                )
            });
            state_estimator.write().unwrap().prediction_step(self, time);
            if let Some(time_analysis) = &self.time_analysis {
                time_analysis
                    .lock()
                    .unwrap()
                    .finished_time_analysis(ta.unwrap());
            }
            do_control_loop = true;
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
                    let ta = self.time_analysis.as_ref().map(|time_analysis| {
                        time_analysis
                            .lock()
                            .unwrap()
                            .time_analysis(time, state_estimator.name.clone() + "_prediction_step")
                    });
                    state_estimator
                        .state_estimator
                        .write()
                        .unwrap()
                        .prediction_step(self, time);
                    if let Some(time_analysis) = &self.time_analysis {
                        time_analysis
                            .lock()
                            .unwrap()
                            .finished_time_analysis(ta.unwrap());
                    }
                }
            }
        }
        if let Some(sensor_manager) = &self.sensor_manager() {
            sensor_manager.write().unwrap().handle_messages(time);
        }
        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            debug!("Post prediction step wait");
        }
        self.sync_with_others(time_cv, time);

        if let Some(sensor_manager) = &self.sensor_manager() {
            sensor_manager.write().unwrap().handle_messages(time);
            sensor_manager
                .write()
                .unwrap()
                .make_observations(self, time);
        }

        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            debug!("Post observation wait");
        }
        self.sync_with_others(time_cv, time);

        if let Some(sensor_manager) = &self.sensor_manager() {
            sensor_manager.write().unwrap().handle_messages(time);
            // Make observations (if it is the right time)
            let observations = sensor_manager.write().unwrap().get_observations();
            if is_enabled(crate::logger::InternalLog::SensorManager) {
                debug!("Got {} observations", observations.len());
            }
            if !observations.is_empty() {
                // Treat the observations
                if let Some(state_estimator) = &self.state_estimator() {
                    let ta = self.time_analysis.as_ref().map(|time_analysis| {
                        time_analysis.lock().unwrap().time_analysis(
                            time,
                            "control_loop_state_estimator_correction_step".to_string(),
                        )
                    });
                    state_estimator
                        .write()
                        .unwrap()
                        .correction_step(self, &observations, time);
                    if let Some(time_analysis) = &self.time_analysis {
                        time_analysis
                            .lock()
                            .unwrap()
                            .finished_time_analysis(ta.unwrap());
                    }
                }

                if let Some(state_estimator_bench) = &self.state_estimator_bench() {
                    for state_estimator in state_estimator_bench.read().unwrap().iter() {
                        let ta = self.time_analysis.as_ref().map(|time_analysis| {
                            time_analysis.lock().unwrap().time_analysis(
                                time,
                                state_estimator.name.clone() + "_correction_step",
                            )
                        });
                        state_estimator
                            .state_estimator
                            .write()
                            .unwrap()
                            .correction_step(self, &observations, time);
                        if let Some(time_analysis) = &self.time_analysis {
                            time_analysis
                                .lock()
                                .unwrap()
                                .finished_time_analysis(ta.unwrap());
                        }
                    }
                }
            }
        }

        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            debug!("Post correction step wait");
        }
        self.sync_with_others(time_cv, time);

        if do_control_loop
            || (self.navigator().is_some()
                && time
                    >= self
                        .navigator()
                        .as_ref()
                        .unwrap()
                        .read()
                        .unwrap()
                        .next_time_step()
                        .unwrap_or(f32::INFINITY))
            || (self.controller().is_some()
                && time
                    >= self
                        .controller()
                        .as_ref()
                        .unwrap()
                        .read()
                        .unwrap()
                        .next_time_step()
                        .unwrap_or(f32::INFINITY))
        {
            let state_estimator = &self.state_estimator().unwrap();
            let world_state = state_estimator.read().unwrap().world_state();

            // Compute the error to the planned path
            let ta = self.time_analysis.as_ref().map(|time_analysis| {
                time_analysis
                    .lock()
                    .unwrap()
                    .time_analysis(time, "control_loop_navigator_compute_error".to_string())
            });
            let error = self
                .navigator()
                .as_ref()
                .unwrap()
                .write()
                .unwrap()
                .compute_error(self, world_state);
            if let Some(time_analysis) = &self.time_analysis {
                time_analysis
                    .lock()
                    .unwrap()
                    .finished_time_analysis(ta.unwrap());
            }

            // Compute the command from the error
            let ta = self.time_analysis.as_ref().map(|time_analysis| {
                time_analysis
                    .lock()
                    .unwrap()
                    .time_analysis(time, "control_loop_controller_make_command".to_string())
            });
            let command = self
                .controller()
                .as_ref()
                .unwrap()
                .write()
                .unwrap()
                .make_command(self, &error, time);
            if let Some(time_analysis) = &self.time_analysis {
                time_analysis
                    .lock()
                    .unwrap()
                    .finished_time_analysis(ta.unwrap());
            }

            // Apply the command to the physics
            self.physics
                .as_ref()
                .unwrap()
                .write()
                .unwrap()
                .apply_command(&command, time);
        }

        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            debug!("Pre-save wait");
        }
        self.sync_with_others(time_cv, time);

        Ok(())
    }

    pub fn sync_with_others(&mut self, time_cv: &TimeCv, time: f32) {
        let mut lk = time_cv.waiting.lock().unwrap();
        let waiting_parity = *time_cv.intermediate_parity.lock().unwrap();
        *lk += 1;
        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            debug!("Increase intermediate waiting nodes: {}", *lk);
        }
        // let circulating_messages = time_cv.circulating_messages.lock().unwrap();
        // if *lk == 0 && *circulating_messages == 0 {
        //     let mut waiting_parity = time_cv.intermediate_parity.lock().unwrap();
        //     *waiting_parity = 1 - *waiting_parity;
        //     time_cv.condvar.notify_all();
        //     if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
        //         debug!("[intermediate wait] I am the last, end wait");
        //     }
        //     return;
        // }
        // std::mem::drop(circulating_messages);
        loop {
            while self.process_messages() > 0 {
                *lk -= 1;
                if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                    debug!("[intermediate wait] Messages to process: handle messages");
                }
                self.handle_messages(time);
                *lk += 1;
            }
            // let circulating_messages = time_cv.circulating_messages.lock().unwrap();
            // // if *lk == nb_nodes && *circulating_messages == 0 {
            // //     // *lk = 0;
            // //     let mut waiting_parity = time_cv.intermediate_parity.lock().unwrap();
            // //     *waiting_parity = 1 - *waiting_parity;
            // //     time_cv.condvar.notify_all();
            // //     if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            // //         debug!("[intermediate wait] I am the last, end wait");
            // //     }
            // //     return;
            // // }
            // if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            //     debug!(
            //         "[intermediate wait] Wait for others (waiting = {}/{nb_nodes}, circulating messages = {})",
            //         *lk, *circulating_messages
            //     );
            // }
            // std::mem::drop(circulating_messages);
            time_cv.condvar.notify_all();
            if self.process_messages() == 0 {
                lk = time_cv.condvar.wait(lk).unwrap();
            }
            if waiting_parity != *time_cv.intermediate_parity.lock().unwrap() {
                if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                    debug!("[intermediate wait] End wait");
                }
                return;
            }
            if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                debug!("[intermediate wait] New loop: waiting = {}", *lk);
            }
        }
    }

    pub fn handle_messages(&mut self, time: f32) {
        self.service_manager
            .as_ref()
            .unwrap()
            .write()
            .unwrap()
            .handle_requests(time);
        while let Some((path, message)) = self.node_message_client.try_receive(time) {
            if path
                == PathKey::from_str(networking::channels::internal::COMMAND)
                    .unwrap()
                    .join_str(self.name().as_str())
            {
                for flag in message.message_flags {
                    if flag == MessageFlag::Kill {
                        self.pre_kill();
                    }
                }
            }
        }
    }

    /// Computes the next time step, using state estimator, sensors and received messages.
    pub fn next_time_step(&self, min_time_excluded: f32) -> SimbaResult<f32> {
        let mut next_time_step = f32::INFINITY;
        if let Some(state_estimator) = &self.state_estimator {
            let next_time = state_estimator.read().unwrap().next_time_step();
            if next_time > min_time_excluded {
                next_time_step = next_time_step.min(next_time);
            }
            if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
                debug!("Next time after state estimator: {next_time_step}");
            }
        }
        if let Some(navigator) = &self.navigator
            && let Some(next_time) = navigator.read().unwrap().next_time_step()
        {
            if next_time > min_time_excluded {
                next_time_step = next_time_step.min(next_time);
            }
            if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
                debug!("Next time after navigator: {next_time_step}");
            }
        }
        if let Some(controller) = &self.controller
            && let Some(next_time) = controller.read().unwrap().next_time_step()
        {
            if next_time > min_time_excluded {
                next_time_step = next_time_step.min(next_time);
            }
            if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
                debug!("Next time after controller: {next_time_step}");
            }
        }
        if let Some(physics) = &self.physics
            && let Some(next_time) = physics.read().unwrap().next_time_step()
        {
            if next_time > min_time_excluded {
                next_time_step = next_time_step.min(next_time);
            }
            if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
                debug!("Next time after physics: {next_time_step}");
            }
        }

        if let Some(sensor_manager) = &self.sensor_manager {
            let next_time = sensor_manager
                .read()
                .unwrap()
                .next_time_step()
                .unwrap_or(f32::INFINITY);
            if next_time > min_time_excluded {
                next_time_step = next_time_step.min(next_time);
            }
            if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
                debug!("Next time after sensor manager: {next_time_step}");
            }
        }
        // if let Some(network) = &self.network {
        //     let message_next_time = network.read().unwrap().next_message_time();
        //     if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
        //         debug!(
        //             "In node: message_next_time: {}",
        //             message_next_time.unwrap_or(-1.)
        //         );
        //     }
        //     if let Some(msg_next_time) = message_next_time
        //         && next_time_step > msg_next_time
        //         && msg_next_time > min_time_excluded
        //     {
        //         next_time_step = msg_next_time;
        //         if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
        //             debug!("Time step changed with message: {}", next_time_step);
        //         }
        //     }
        //     if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
        //         debug!("Next time after network: {next_time_step}");
        //     }
        // }
        if let Some(state_estimator_bench) = &self.state_estimator_bench {
            for state_estimator in state_estimator_bench.read().unwrap().iter() {
                let next_time = state_estimator
                    .state_estimator
                    .read()
                    .unwrap()
                    .next_time_step();
                if next_time > min_time_excluded {
                    next_time_step = next_time_step.min(next_time);
                }
            }
            if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
                debug!("Next time after state estimator bench: {next_time_step}");
            }
        }
        let next_time = self
            .service_manager
            .as_ref()
            .unwrap()
            .read()
            .unwrap()
            .next_time();
        if next_time > min_time_excluded {
            next_time_step = next_time_step.min(next_time);
        }
        if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
            debug!("Next time after service manager: {next_time_step}");
        }
        next_time_step = round_precision(next_time_step, TIME_ROUND).unwrap();
        if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
            debug!("next_time_step: {}", next_time_step);
        }
        Ok(next_time_step)
    }
}

// Getters
impl Node {
    /// Get the name of the node.
    pub fn name(&self) -> String {
        self.node_meta_data.read().unwrap().name.clone()
    }

    pub fn state(&self) -> NodeState {
        self.node_meta_data.read().unwrap().state.clone()
    }

    pub(crate) fn set_state(&mut self, state: NodeState) {
        self.node_meta_data.write().unwrap().state = state;
    }

    pub fn send_records(&self) -> bool {
        self.send_records
    }

    pub fn other_node_names(&self) -> &[String] {
        &self.other_node_names
    }

    pub fn node_type(&self) -> NodeType {
        self.node_meta_data.read().unwrap().node_type.clone()
    }

    /// Get a Arc clone of network module.
    pub fn network(&self) -> Option<SharedRwLock<Network>> {
        match &self.network {
            Some(n) => Some(Arc::clone(n)),
            None => None,
        }
    }

    /// Get a Arc clone of physics module.
    pub fn physics(&self) -> Option<SharedRwLock<Box<dyn Physics>>> {
        match &self.physics {
            Some(p) => Some(Arc::clone(p)),
            None => None,
        }
    }

    /// Get a Arc clone of sensor manager.
    pub fn sensor_manager(&self) -> Option<SharedRwLock<SensorManager>> {
        match &self.sensor_manager {
            Some(sm) => Some(Arc::clone(sm)),
            None => None,
        }
    }

    /// Get a Arc clone of state estimator module.
    pub fn state_estimator(&self) -> Option<SharedRwLock<Box<dyn StateEstimator>>> {
        match &self.state_estimator {
            Some(se) => Some(Arc::clone(se)),
            None => None,
        }
    }

    /// Get a Arc clone of state estimator module.
    pub fn state_estimator_bench(&self) -> Option<SharedRwLock<Vec<BenchStateEstimator>>> {
        match &self.state_estimator_bench {
            Some(se) => Some(Arc::clone(se)),
            None => None,
        }
    }

    /// Get a Arc clone of navigator module.
    pub fn navigator(&self) -> Option<SharedRwLock<Box<dyn Navigator>>> {
        match &self.navigator {
            Some(n) => Some(Arc::clone(n)),
            None => None,
        }
    }

    /// Get a Arc clone of controller module.
    pub fn controller(&self) -> Option<SharedRwLock<Box<dyn Controller>>> {
        match &self.controller {
            Some(c) => Some(Arc::clone(c)),
            None => None,
        }
    }

    /// Get a Arc clone of Service Manager.
    pub fn service_manager(&self) -> SharedRwLock<ServiceManager> {
        self.service_manager.as_ref().unwrap().clone()
    }

    pub fn meta_data(&self) -> SharedRoLock<NodeMetaData> {
        self.node_meta_data.clone() as Arc<dyn RoLock<NodeMetaData>>
    }

    pub fn meta_data_list(
        &self,
    ) -> Option<SharedRoLock<BTreeMap<String, SharedRoLock<NodeMetaData>>>> {
        self.meta_data_list.as_ref().cloned()
    }

    pub fn pre_kill(&mut self) {
        self.node_meta_data.write().unwrap().state = NodeState::Zombie;
    }

    pub fn kill(&mut self, time: f32) {
        self.node_meta_data.write().unwrap().state = NodeState::Zombie;
        if let Some(service_manager) = &self.service_manager {
            service_manager.write().unwrap().unsubscribe_node();
        }
        self.node_server
            .as_ref()
            .unwrap()
            .state_update
            .as_ref()
            .unwrap()
            .send((
                time,
                (
                    State::new(),
                    self.node_meta_data.read().unwrap().state.clone(),
                ),
            ))
            .unwrap();
    }
}

// Record part
impl Node {
    fn robot_record(&self) -> RobotRecord {
        let meta_data = self.node_meta_data.read().unwrap();
        let mut record = RobotRecord {
            name: meta_data.name.clone(),
            model_name: meta_data.model_name.clone(),
            labels: meta_data.labels.clone(),
            navigator: self.navigator.as_ref().unwrap().read().unwrap().record(),
            controller: self.controller.as_ref().unwrap().read().unwrap().record(),
            physics: self.physics.as_ref().unwrap().read().unwrap().record(),
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
            state: meta_data.state.clone(),
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
        let meta_data = self.node_meta_data.read().unwrap();
        let mut record = ComputationUnitRecord {
            name: meta_data.name.clone(),
            state_estimators: Vec::new(),
            sensor_manager: self.sensor_manager().unwrap().read().unwrap().record(),
            labels: meta_data.labels.clone(),
            model_name: meta_data.model_name.clone(),
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

impl Recordable<NodeRecord> for Node {
    /// Generate the current state record.
    fn record(&self) -> NodeRecord {
        match &self.node_meta_data.read().unwrap().node_type {
            NodeType::Robot => NodeRecord::Robot(Box::new(self.robot_record())),
            NodeType::ComputationUnit => {
                NodeRecord::ComputationUnit(self.computation_unit_record())
            }
            _ => unimplemented!(),
        }
    }
}
