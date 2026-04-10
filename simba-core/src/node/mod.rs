//! Node runtime orchestration and lifecycle management.
//!
//! This module defines [`Node`], the central runtime unit that wires together navigation,
//! control, physics simulation, sensing, networking, services, and recording.
//! [`Node`] executes the simulation loop, processes asynchronous messages, and coordinates
//! time synchronization across nodes.
//!
//! Node construction is delegated to [`NodeFactory`](crate::node::node_factory::NodeFactory),
//! which assembles concrete implementations from configuration.

pub mod node_factory;

use node_factory::{ComputationUnitRecord, NodeRecord, NodeType, RobotRecord};
use serde::{Deserialize, Serialize};
use simba_com::pub_sub::{MultiClientTrait, PathKey};
use simba_macros::EnumToString;

use core::f32;
use std::collections::{BTreeMap, HashMap};
use std::str::FromStr;
use std::sync::Arc;

use crate::context::Context;
use crate::environment::Environment;
use crate::errors::{SimbaError, SimbaErrorTypes};
use crate::networking::network::MessageFlag;
use crate::physics::robot_models::Command;
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
    navigators::Navigator,
    networking::network::Network,
    physics::Physics,
    recordable::Recordable,
    sensors::sensor_manager::SensorManager,
    simulator::TimeCv,
    state_estimators::{BenchStateEstimator, BenchStateEstimatorRecord, StateEstimator},
    utils::maths::round_precision,
};
use crate::{internal, networking};

/// Mode State machine.
///
/// TODO: Use Rust type system to enforce correct mode usage instead of runtime checks.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, EnumToString)]
pub enum NodeState {
    /// Node has been created but is not running yet.
    Created,
    /// Node is active and can execute simulation steps.
    Running,
    /// Node is marked for termination and will stop at the next safe point.
    Zombie,
    /// Node has fully terminated and no longer participates in simulation.
    Terminated,
}

/// Metadata describing a node, used for introspection, logging, and inter-node coordination.
#[derive(Debug, Clone)]
pub struct NodeMetaData {
    /// Unique runtime name of the node.
    pub name: String,
    /// Type of node, such as robot or computation unit.
    pub node_type: NodeType,
    /// Name of the model used to instantiate this node.
    pub model_name: String,
    /// User-defined labels used for grouping and filtering.
    pub labels: Vec<String>,
    /// Current lifecycle state of the node.
    pub state: NodeState,
    /// Current ground-truth planar position when available.
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
/// * `sensor_manager`, of type [`SensorManager`], manages the [`Sensor`](crate::sensors::Sensor)s. The
///   observations of the sensors are sent to the [`StateEstimator`].
/// * `network` is the node [`Network`] interface. It manages the reception and
///   the send of messages to other nodes.
/// * `environment` is the shared environment of the simulator, used to get the list of all nodes, and
///   the landmark map.
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
    /// Manages all the [`Sensor`](crate::sensors::Sensor)s and send the observations to `state_estimator`.
    pub(self) sensor_manager: Option<SharedRwLock<SensorManager>>,
    /// [`Network`] interface to receive and send messages with other nodes.
    pub(self) network: Option<SharedRwLock<Network>>,
    /// Additional [`StateEstimator`] to be evaluated.
    pub(self) state_estimator_bench: Option<SharedRwLock<Vec<BenchStateEstimator>>>,

    pub(self) node_server: Option<NodeServer>,

    pub(self) other_node_names: Vec<String>,
    pub(self) other_node_physics: SharedRoLock<BTreeMap<String, SharedRoLock<Box<dyn Physics>>>>,
    pub(self) time_analysis: Option<SharedMutex<TimeAnalysisNode>>,
    pub(self) send_records: bool,

    pub(self) node_meta_data: SharedRwLock<NodeMetaData>,
    pub(self) meta_data_list: Option<SharedRoLock<HashMap<String, SharedRoLock<NodeMetaData>>>>,
    pub(self) node_message_client: SimbaBrokerMultiClient,

    pub(self) current_command: Option<Command>,

    pub(self) environment: Arc<Environment>,
}

impl Node {
    /// Initialize the node after its creation.
    ///
    /// It is used to initialize the sensor manager, which need to know the list of all nodes.
    pub fn post_creation_init(
        &mut self,
        physics_list: SharedRoLock<BTreeMap<String, SharedRoLock<Box<dyn Physics>>>>,
        meta_data_list: SharedRoLock<HashMap<String, SharedRoLock<NodeMetaData>>>,
        initial_time: f32,
        context: &Context,
    ) -> NodeClient {
        let context = context.new_callstack_level("post_creation_init");
        internal!(
            context,
            crate::logger::InternalLog::SetupSteps,
            "Node post-creation initialization"
        );

        self.other_node_physics = physics_list;

        // Using meta_data_list to get names as only nodes with physics are in physics_list.
        for name in meta_data_list.read().unwrap().keys() {
            if name != &self.name() {
                self.other_node_names.push(name.clone());
            }
        }

        // Post init of the modules
        if let Some(physics) = self.physics() {
            physics.write().unwrap().post_init(self, &context).unwrap();
        }
        if let Some(state_estimator) = self.state_estimator() {
            state_estimator
                .write()
                .unwrap()
                .post_init(self, &context)
                .unwrap();
        }
        if let Some(state_estimator_bench) = self.state_estimator_bench.clone() {
            for state_estimator in state_estimator_bench.read().unwrap().iter() {
                state_estimator
                    .state_estimator
                    .write()
                    .unwrap()
                    .post_init(self, &context)
                    .unwrap();
            }
        }
        if let Some(sensor_manager) = self.sensor_manager() {
            sensor_manager
                .write()
                .unwrap()
                .post_init(self, initial_time, &context)
                .unwrap();
        }
        if let Some(navigator) = self.navigator() {
            navigator
                .write()
                .unwrap()
                .post_init(self, &context)
                .unwrap();
        }

        if let Some(controller) = self.controller() {
            controller
                .write()
                .unwrap()
                .post_init(self, &context)
                .unwrap();
        }

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
    /// ## Arguments
    /// * `time` -- Time to reach.
    #[cfg(not(feature = "monothreaded"))]
    pub(crate) fn run_next_time_step(
        &mut self,
        time: f32,
        time_cv: &TimeCv,
        context: &Context,
    ) -> SimbaResult<()> {
        self.process_messages();
        self.run_time_step(time, time_cv, context)
    }

    /// Process all the messages: one-way (network) and two-way (services).
    ///
    /// Processing messages mean here to transfer all the pending messages from the network to the corresponding modules (physics, state estimator, navigator, controller, sensor manager).
    pub(crate) fn process_messages(&self) -> bool {
        self.node_message_client.next_message_time().is_some()
    }

    /// Run only one time step.
    ///
    /// The update step is done in this order:
    /// 1. Update the physics
    /// 2. Call to `pre_loop_hook`s.
    /// 3. Prediction step of the state estimators
    /// 4. Generate the observations (and send them)
    /// 5. Correction step of the state estimator
    /// 6. If it is the time for the state estimator to do its prediction step or required by either controller or navigator:
    ///     1. The navigator computes the error from the state estimation
    ///     2. The command is computed by the Controller
    ///     3. The command is applied to the Physics (but Physics state is not updated yet).
    ///
    /// The network messages are handled between each steps.
    ///
    /// Then, the node state is saved.
    #[cfg(not(feature = "monothreaded"))]
    fn run_time_step(&mut self, time: f32, time_cv: &TimeCv, context: &Context) -> SimbaResult<()> {
        use crate::info;

        let context = context.new_callstack_level("run_time_step");
        if self.node_meta_data.read().unwrap().state != NodeState::Running {
            return Err(SimbaError::new(
                SimbaErrorTypes::ImplementationError,
                "Only a Running node should be run!".to_string(),
            ));
        }
        info!(context, "Run time {}", time);

        // Update the true state
        self.physics_update(time, &context);

        self.handle_messages(time, &context);
        self.sync_with_others(time_cv, time, &context);

        // Pre loop calls to manage messages
        self.pre_loop_hooks(time, &context);

        self.handle_messages(time, &context);
        internal!(
            context,
            crate::logger::InternalLog::NodeSyncDetailed,
            "Pre prediction step wait"
        );
        self.sync_with_others(time_cv, time, &context);

        let do_control_loop = self.prediction_step(time, &context);

        self.handle_messages(time, &context);
        internal!(
            context,
            crate::logger::InternalLog::NodeSyncDetailed,
            "Post prediction step wait"
        );
        self.sync_with_others(time_cv, time, &context);

        self.make_observations(time, &context);

        self.handle_messages(time, &context);
        internal!(
            context,
            crate::logger::InternalLog::NodeSyncDetailed,
            "Post observation wait"
        );
        self.sync_with_others(time_cv, time, &context);

        self.correction_step(time, &context);

        self.handle_messages(time, &context);
        internal!(
            context,
            crate::logger::InternalLog::NodeSyncDetailed,
            "Post correction step wait"
        );
        self.sync_with_others(time_cv, time, &context);

        self.nav_and_control_step(time, do_control_loop, &context);

        self.handle_messages(time, &context);
        internal!(
            context,
            crate::logger::InternalLog::NodeSyncDetailed,
            "Pre-save wait"
        );
        self.sync_with_others(time_cv, time, &context);

        Ok(())
    }

    /// Synchronize this node with the other nodes at an intermediate barrier.
    ///
    /// The method repeatedly processes pending messages while waiting for the
    /// synchronization parity to change.
    #[cfg(not(feature = "monothreaded"))]
    pub(crate) fn sync_with_others(&mut self, time_cv: &TimeCv, time: f32, context: &Context) {
        let context = context.new_callstack_level("sync_with_others");
        let mut lk = time_cv.waiting.lock().unwrap();
        let waiting_parity = *time_cv.intermediate_parity.lock().unwrap();
        *lk += 1;
        internal!(
            context,
            crate::logger::InternalLog::NodeSyncDetailed,
            "Increase intermediate waiting nodes: {}",
            *lk
        );

        loop {
            while self.process_messages() {
                *lk -= 1;
                internal!(
                    context,
                    crate::logger::InternalLog::NodeSyncDetailed,
                    "Messages to process: handle messages"
                );
                self.handle_messages(time, &context);
                *lk += 1;
            }
            time_cv.condvar.notify_all();
            if !self.process_messages() {
                lk = time_cv.condvar.wait(lk).unwrap();
            }
            if waiting_parity != *time_cv.intermediate_parity.lock().unwrap() {
                internal!(
                    context,
                    crate::logger::InternalLog::NodeSyncDetailed,
                    "End wait"
                );
                return;
            }
            internal!(
                context,
                crate::logger::InternalLog::NodeSyncDetailed,
                "New loop: waiting = {}",
                *lk
            );
        }
    }

    /// Handle all pending service and network messages up to `time`.
    ///
    /// It means that the actions linked to each services or messages are executed here.
    ///
    /// ## Arguments
    /// * `time` - Current simulation time upper-bound for message processing.
    /// * `context` - Shared simulation context used for logging.
    pub fn handle_messages(&mut self, time: f32, context: &Context) {
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
        if let Some(sensor_manager) = &self.sensor_manager() {
            sensor_manager
                .write()
                .unwrap()
                .handle_messages(time, context);
        }
    }

    /// Computes the next time step, using state estimator, sensors and received messages.
    ///
    /// ## Arguments
    /// * `min_time_excluded` - Lower bound: candidate times must be strictly greater than this value.
    /// * `context` - Shared simulation context used for logging and scoped call tracing.
    pub fn next_time_step(&self, min_time_excluded: f32, context: &Context) -> SimbaResult<f32> {
        let context = context.new_callstack_level("next_time_step");
        let mut next_time_step = f32::INFINITY;
        if let Some(state_estimator) = &self.state_estimator {
            let next_time = state_estimator.read().unwrap().next_time_step(&context);
            if next_time > min_time_excluded {
                next_time_step = next_time_step.min(next_time);
            }
            internal!(
                context,
                crate::logger::InternalLog::NodeRunningDetailed,
                "Next time after state estimator: {next_time_step}"
            );
        }
        if let Some(navigator) = &self.navigator
            && let Some(next_time) = navigator.read().unwrap().next_time_step(&context)
        {
            if next_time > min_time_excluded {
                next_time_step = next_time_step.min(next_time);
            }
            internal!(
                context,
                crate::logger::InternalLog::NodeRunningDetailed,
                "Next time after navigator: {next_time_step}"
            );
        }
        if let Some(controller) = &self.controller
            && let Some(next_time) = controller.read().unwrap().next_time_step(&context)
        {
            if next_time > min_time_excluded {
                next_time_step = next_time_step.min(next_time);
            }
            internal!(
                context,
                crate::logger::InternalLog::NodeRunningDetailed,
                "Next time after controller: {next_time_step}"
            );
        }
        if let Some(physics) = &self.physics
            && let Some(next_time) = physics.read().unwrap().next_time_step(&context)
        {
            if next_time > min_time_excluded {
                next_time_step = next_time_step.min(next_time);
            }
            internal!(
                context,
                crate::logger::InternalLog::NodeRunningDetailed,
                "Next time after physics: {next_time_step}"
            );
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
            internal!(
                context,
                crate::logger::InternalLog::NodeRunningDetailed,
                "Next time after sensor manager: {next_time_step}"
            );
        }
        if let Some(state_estimator_bench) = &self.state_estimator_bench {
            for state_estimator in state_estimator_bench.read().unwrap().iter() {
                let next_time = state_estimator
                    .state_estimator
                    .read()
                    .unwrap()
                    .next_time_step(&context);
                if next_time > min_time_excluded {
                    next_time_step = next_time_step.min(next_time);
                }
            }
            internal!(
                context,
                crate::logger::InternalLog::NodeRunningDetailed,
                "Next time after state estimator bench: {next_time_step}"
            );
        }
        next_time_step = round_precision(next_time_step, TIME_ROUND).unwrap();
        internal!(
            context,
            crate::logger::InternalLog::NodeRunningDetailed,
            "next_time_step: {next_time_step}"
        );
        Ok(next_time_step)
    }
}

/// Running steps
impl Node {
    /// Physics update
    pub(crate) fn physics_update(&mut self, time: f32, context: &Context) {
        let context = context.new_callstack_level("physics_update");
        if let Some(physics) = &self.physics {
            physics.write().unwrap().update_state(time, &context);
            let pose = physics.read().unwrap().state(time, &context).pose;
            self.node_meta_data.write().unwrap().position = Some([pose[0], pose[1]]);
        }
    }

    pub(crate) fn pre_loop_hooks(&mut self, time: f32, context: &Context) {
        let context = context.new_callstack_level("pre_loop_hooks");
        if let Some(state_estimator) = self.state_estimator() {
            state_estimator
                .write()
                .unwrap()
                .pre_loop_hook(self, time, &context);
        }
        if let Some(state_estimator_bench) = self.state_estimator_bench.clone() {
            for state_estimator in state_estimator_bench.read().unwrap().iter() {
                state_estimator
                    .state_estimator
                    .write()
                    .unwrap()
                    .pre_loop_hook(self, time, &context);
            }
        }
        if let Some(controller) = self.controller() {
            controller
                .write()
                .unwrap()
                .pre_loop_hook(self, time, &context);
        }
        if let Some(navigator) = self.navigator() {
            navigator
                .write()
                .unwrap()
                .pre_loop_hook(self, time, &context);
        }
    }

    pub(crate) fn prediction_step(&mut self, time: f32, context: &Context) -> bool {
        let context = context.new_callstack_level("prediction_step");
        if let Some(state_estimator_bench) = &self.state_estimator_bench() {
            for state_estimator in state_estimator_bench.read().unwrap().iter() {
                if time
                    >= state_estimator
                        .state_estimator
                        .read()
                        .unwrap()
                        .next_time_step(&context)
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
                        .prediction_step(self, self.current_command.clone(), time, &context);
                    if let Some(time_analysis) = &self.time_analysis {
                        time_analysis
                            .lock()
                            .unwrap()
                            .finished_time_analysis(ta.unwrap());
                    }
                }
            }
        }

        if let Some(state_estimator) = &self.state_estimator()
            && time >= state_estimator.read().unwrap().next_time_step(&context)
        {
            // Prediction step
            let ta = self.time_analysis.as_ref().map(|time_analysis| {
                time_analysis.lock().unwrap().time_analysis(
                    time,
                    "control_loop_state_estimator_prediction_step".to_string(),
                )
            });
            state_estimator.write().unwrap().prediction_step(
                self,
                self.current_command.clone(),
                time,
                &context,
            );
            if let Some(time_analysis) = &self.time_analysis {
                time_analysis
                    .lock()
                    .unwrap()
                    .finished_time_analysis(ta.unwrap());
            }
            true
        } else {
            false
        }
    }

    pub(crate) fn make_observations(&mut self, time: f32, context: &Context) {
        let context = context.new_callstack_level("make_observations");
        if let Some(sensor_manager) = &self.sensor_manager() {
            sensor_manager
                .write()
                .unwrap()
                .handle_messages(time, &context);
            sensor_manager
                .write()
                .unwrap()
                .make_observations(self, time, &context);
        }
    }

    pub(crate) fn correction_step(&mut self, time: f32, context: &Context) {
        let context = context.new_callstack_level("correction_step");
        if let Some(sensor_manager) = &self.sensor_manager() {
            sensor_manager
                .write()
                .unwrap()
                .handle_messages(time, &context);
            // Make observations (if it is the right time)
            let observations = sensor_manager.write().unwrap().get_observations();
            internal!(
                context,
                crate::logger::InternalLog::SensorManager,
                "Got {} observations",
                observations.len()
            );
            if !observations.is_empty() {
                // Treat the observations
                if let Some(state_estimator) = &self.state_estimator() {
                    let ta = self.time_analysis.as_ref().map(|time_analysis| {
                        time_analysis.lock().unwrap().time_analysis(
                            time,
                            "control_loop_state_estimator_correction_step".to_string(),
                        )
                    });
                    state_estimator.write().unwrap().correction_step(
                        self,
                        &observations,
                        time,
                        &context,
                    );
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
                            .correction_step(self, &observations, time, &context);
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
    }

    pub(crate) fn nav_and_control_step(
        &mut self,
        time: f32,
        do_control_loop: bool,
        context: &Context,
    ) {
        let context = context.new_callstack_level("nav_and_control_step");
        if do_control_loop
            || (self.navigator().is_some()
                && time
                    >= self
                        .navigator()
                        .as_ref()
                        .unwrap()
                        .read()
                        .unwrap()
                        .next_time_step(&context)
                        .unwrap_or(f32::INFINITY))
            || (self.controller().is_some()
                && time
                    >= self
                        .controller()
                        .as_ref()
                        .unwrap()
                        .read()
                        .unwrap()
                        .next_time_step(&context)
                        .unwrap_or(f32::INFINITY))
        {
            let state_estimator = &self.state_estimator().unwrap();
            let world_state = state_estimator.read().unwrap().world_state(&context);

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
                .compute_error(self, world_state, &context);
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
                .make_command(self, &error, time, &context);
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
                .apply_command(&command, time, &context);
            self.current_command = Some(command);
        }
    }
}

// Getters
impl Node {
    /// Get the name of the node.
    pub fn name(&self) -> String {
        self.node_meta_data.read().unwrap().name.clone()
    }

    /// Get the current lifecycle state.
    pub fn state(&self) -> NodeState {
        self.node_meta_data.read().unwrap().state.clone()
    }

    pub(crate) fn set_state(&mut self, state: NodeState) {
        self.node_meta_data.write().unwrap().state = state;
    }

    /// Return whether this node is configured to emit records.
    pub fn send_records(&self) -> bool {
        self.send_records
    }

    /// Get the names of other known nodes in the simulation.
    pub fn other_node_names(&self) -> &[String] {
        &self.other_node_names
    }

    /// Get this node type.
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

    /// Get the shared simulation [`Environment`].
    pub fn environment(&self) -> &Arc<Environment> {
        &self.environment
    }

    /// Get shared read-only access to the physics module of another node by name.
    /// Returns `None` if the node name is not found or if the physics module is not available.
    pub fn get_other_node_physics(
        &self,
        node_name: &str,
    ) -> Option<SharedRoLock<Box<dyn Physics>>> {
        self.other_node_physics
            .read()
            .unwrap()
            .get(node_name)
            .cloned()
    }

    /// Get shared read-only access to the physics modules of all other nodes.
    /// The returned map is keyed by node name and contains shared read-only locks to the physics modules.
    /// The map is maintained by the simulator and updated as nodes are added or removed.
    /// It allows this node to query the physics state of other nodes for coordination, collision checking, or other purposes.
    pub fn get_all_node_physics(
        &self,
    ) -> &SharedRoLock<BTreeMap<String, SharedRoLock<Box<dyn Physics>>>> {
        &self.other_node_physics
    }

    /// Get shared read-only access to this node metadata.
    pub fn meta_data(&self) -> SharedRoLock<NodeMetaData> {
        self.node_meta_data.clone() as Arc<dyn RoLock<NodeMetaData>>
    }

    /// Get the optional shared metadata map for all nodes.
    pub fn meta_data_list(
        &self,
    ) -> Option<SharedRoLock<HashMap<String, SharedRoLock<NodeMetaData>>>> {
        self.meta_data_list.clone()
            as Option<Arc<dyn RoLock<HashMap<String, SharedRoLock<NodeMetaData>>>>>
    }

    /// Mark this node as [`NodeState::Zombie`]. The kill is done by [`Self::kill`].
    pub fn pre_kill(&mut self) {
        self.node_meta_data.write().unwrap().state = NodeState::Zombie;
    }

    /// Terminate this node and publish its final state update.
    pub fn kill(&mut self, time: f32) {
        self.node_meta_data.write().unwrap().state = NodeState::Terminated;
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
    fn robot_record(&self, context: &Context) -> RobotRecord {
        let context = context.new_callstack_level("robot_record");
        let meta_data = self.node_meta_data.read().unwrap();
        let mut record = RobotRecord {
            name: meta_data.name.clone(),
            model_name: meta_data.model_name.clone(),
            labels: meta_data.labels.clone(),
            navigator: self
                .navigator
                .as_ref()
                .unwrap()
                .read()
                .unwrap()
                .record(&context),
            controller: self
                .controller
                .as_ref()
                .unwrap()
                .read()
                .unwrap()
                .record(&context),
            physics: self
                .physics
                .as_ref()
                .unwrap()
                .read()
                .unwrap()
                .record(&context),
            state_estimator: self
                .state_estimator
                .as_ref()
                .unwrap()
                .read()
                .unwrap()
                .record(&context),
            state_estimator_bench: Vec::new(),
            sensors: self
                .sensor_manager
                .as_ref()
                .unwrap()
                .read()
                .unwrap()
                .record(&context),
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
                        .record(&context),
                });
        }
        record
    }

    fn computation_unit_record(&self, context: &Context) -> ComputationUnitRecord {
        let context = context.new_callstack_level("computation_unit_record");
        let meta_data = self.node_meta_data.read().unwrap();
        let mut record = ComputationUnitRecord {
            name: meta_data.name.clone(),
            state_estimators: Vec::new(),
            sensor_manager: self
                .sensor_manager()
                .unwrap()
                .read()
                .unwrap()
                .record(&context),
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
                    .record(&context),
            });
        }
        record
    }
}

impl Recordable<NodeRecord> for Node {
    /// Generate the current state record.
    fn record(&self, context: &Context) -> NodeRecord {
        match &self.node_meta_data.read().unwrap().node_type {
            NodeType::Robot => NodeRecord::Robot(Box::new(self.robot_record(context))),
            NodeType::ComputationUnit => {
                NodeRecord::ComputationUnit(Box::new(self.computation_unit_record(context)))
            }
            _ => unimplemented!(),
        }
    }
}
