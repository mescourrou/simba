/*!
Module providing the main node manager, [`Node`]. The building of the Nodes is done by [`NodeFactory`](crate::node_factory::NodeFactory).
*/

use core::f32;
use std::collections::BTreeMap;
use std::sync::{Arc, RwLock};

use log::{debug, info};

use crate::networking::message_handler::MessageHandler;
use crate::{
    api::internal_api::{self, NodeClient, NodeServer},
    constants::TIME_ROUND,
    controllers::controller::Controller,
    errors::SimbaResult,
    logger::is_enabled,
    navigators::navigator::Navigator,
    networking::network::Network,
    networking::service_manager::ServiceManager,
    node_factory::{ComputationUnitRecord, NodeRecord, NodeType, RobotRecord},
    physics::physics::Physics,
    recordable::Recordable,
    sensors::sensor_manager::SensorManager,
    simulator::TimeCv,
    state_estimators::state_estimator::{
        BenchStateEstimator, BenchStateEstimatorRecord, StateEstimator,
    },
    time_analysis,
    utils::maths::round_precision,
    utils::time_ordered_data::TimeOrderedData,
};

// Node itself

/// Structure managing one node.
///
/// It is composed of modules to manage different aspects:
/// * `navigator` is of [`Navigator`] trait, and defines the error to be sent
/// to the [`Controller`] to follow the required trajectory.
/// * `controller` is of [`Controller`] trait, it defines the command to be sent
/// to the [`Physics`] module.
/// * `physics` is of [`Physics`] trait. It simulates the node behaviour, its real
/// state. It contains a ground truth to evaluate the [`StateEstimator`].
/// * `state_estimator` is of [`StateEstimator`] trait. It estimates the node
/// state, and send it to the [`Navigator`].
///
/// * `sensor_manager`, of type [`SensorManager`], manages the [`Sensor`](crate::sensors::sensor::Sensor)s. The
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
    /// [`Physics`] module, implementing the physics strategy.
    pub(crate) physics: Option<Arc<RwLock<Box<dyn Physics>>>>,
    /// [`StateEstimator`] module, implementing the state estimation strategy.
    pub(crate) state_estimator: Option<Arc<RwLock<Box<dyn StateEstimator>>>>,
    /// Manages all the [`Sensor`](crate::sensors::sensor::Sensor)s and send the observations to `state_estimator`.
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
    pub zombie: bool,
}

impl Node {
    /// Initialize the node after its creation.
    ///
    /// It is used to initialize the sensor manager, which need to know the list of all nodes.
    pub fn post_creation_init(
        &mut self,
        service_manager_list: &BTreeMap<String, Arc<RwLock<ServiceManager>>>,
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
                network
                    .write()
                    .unwrap()
                    .subscribe(sensor_manager.read().unwrap().get_letter_box());
            }
            if let Some(state_estimator) = &self.state_estimator {
                network
                    .write()
                    .unwrap()
                    .subscribe(state_estimator.read().unwrap().get_letter_box());
            }
            if let Some(state_estimator_bench) = &self.state_estimator_bench {
                for state_estimator in state_estimator_bench.read().unwrap().iter() {
                    network.write().unwrap().subscribe(
                        state_estimator
                            .state_estimator
                            .read()
                            .unwrap()
                            .get_letter_box(),
                    );
                }
            }
        }
        let (node_server, node_client) = internal_api::make_node_api(&self.node_type);
        self.node_server = Some(node_server);
        if is_enabled(crate::logger::InternalLog::SetupStepsDetailed) {
            debug!("Save initial state");
        }
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
    pub fn run_next_time_step(
        &mut self,
        time: f32,
        time_cv: &TimeCv,
        nb_nodes: usize,
    ) -> SimbaResult<()> {
        self.process_messages();
        self.run_time_step(time, time_cv, nb_nodes)
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
    pub fn run_time_step(
        &mut self,
        time: f32,
        time_cv: &TimeCv,
        nb_nodes: usize,
    ) -> SimbaResult<()> {
        info!("Run time {}", time);

        // Update the true state
        if let Some(physics) = &self.physics {
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

        self.handle_messages(time);
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
        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            debug!("Pre prediction step wait");
        }
        self.sync_with_others(time_cv, nb_nodes, time);

        let mut do_control_loop = false;

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
                do_control_loop = true;
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

        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            debug!("Post prediction step wait");
        }
        self.sync_with_others(time_cv, nb_nodes, time);

        if let Some(sensor_manager) = &self.sensor_manager() {
            sensor_manager
                .write()
                .unwrap()
                .make_observations(self, time);
        }

        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            debug!("Post observation wait");
        }
        self.sync_with_others(time_cv, nb_nodes, time);

        if let Some(sensor_manager) = &self.sensor_manager() {
            // Make observations (if it is the right time)
            let observations = sensor_manager.write().unwrap().get_observations();
            if is_enabled(crate::logger::InternalLog::SensorManager) {
                debug!("Got {} observations", observations.len());
            }
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

        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            debug!("Post correction step wait");
        }
        self.sync_with_others(time_cv, nb_nodes, time);

        if do_control_loop {
            let state_estimator = &self.state_estimator().unwrap();
            let world_state = state_estimator.read().unwrap().world_state();

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
                .compute_error(self, world_state);
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
        self.sync_with_others(time_cv, nb_nodes, time);

        // Save state (replace if needed)
        self.save_state(time);

        Ok(())
    }

    pub fn sync_with_others(&mut self, time_cv: &TimeCv, nb_nodes: usize, time: f32) {
        let mut lk = time_cv.waiting.lock().unwrap();
        let waiting_parity = *time_cv.intermediate_parity.lock().unwrap();
        *lk += 1;
        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
            debug!("Increase intermediate waiting nodes nodes: {}", *lk);
        }
        let circulating_messages = time_cv.circulating_messages.lock().unwrap();
        if *lk == nb_nodes && *circulating_messages == 0 {
            *lk = 0;
            let mut waiting_parity = time_cv.intermediate_parity.lock().unwrap();
            *waiting_parity = 1 - *waiting_parity;
            time_cv.condvar.notify_all();
            if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                debug!("[intermediate wait] I am the last, end wait");
            }
            return;
        }
        std::mem::drop(circulating_messages);
        loop {
            while self.process_messages() > 0 {
                *lk -= 1;
                if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                    debug!("[intermediate wait] Messages to process: handle messages");
                }
                self.handle_messages(time);
                *lk += 1;
            }
            let circulating_messages = time_cv.circulating_messages.lock().unwrap();
            if *lk == nb_nodes && *circulating_messages == 0 {
                *lk = 0;
                let mut waiting_parity = time_cv.intermediate_parity.lock().unwrap();
                *waiting_parity = 1 - *waiting_parity;
                time_cv.condvar.notify_all();
                if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                    debug!("[intermediate wait] I am the last, end wait");
                }
                return;
            }
            if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                debug!("[intermediate wait] Wait for others (waiting = {}/{nb_nodes}, circulating messages = {})", *lk, *circulating_messages);
            }
            std::mem::drop(circulating_messages);
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
        if let Some(network) = &self.network {
            let message_next_time = network.read().unwrap().next_message_time();
            if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
                debug!(
                    "In node: message_next_time: {}",
                    match message_next_time {
                        Some(time) => time,
                        None => -1.,
                    }
                );
            }
            if let Some(msg_next_time) = message_next_time {
                if next_time_step > msg_next_time && msg_next_time > min_time_excluded {
                    next_time_step = msg_next_time;
                    if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
                        debug!("Time step changed with message: {}", next_time_step);
                    }
                }
            }
            if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
                debug!("Next time after network: {next_time_step}");
            }
        }
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

    /// Save the current state to the given `time`.
    fn save_state(&mut self, time: f32) {
        self.state_history.insert(time, self.record(), true);
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
    pub fn physics(&self) -> Option<Arc<RwLock<Box<dyn Physics>>>> {
        match &self.physics {
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

    pub fn pre_kill(&mut self) {
        self.zombie = true;
    }

    pub fn kill(&mut self) {
        self.zombie = true;
        if let Some(network) = &self.network {
            network.write().unwrap().unsubscribe_node().unwrap();
        }
        if let Some(service_manager) = &self.service_manager {
            service_manager.write().unwrap().unsubscribe_node();
        }
    }
}

// Record part
impl Node {
    fn robot_record(&self) -> RobotRecord {
        let mut record = RobotRecord {
            name: self.name.clone(),
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

impl Recordable<NodeRecord> for Node {
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
}
