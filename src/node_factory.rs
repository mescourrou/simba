use std::sync::{Arc, Condvar, Mutex, RwLock};

use config_checker::macros::Check;
use log::debug;
use rand::rngs;
use serde::{Deserialize, Serialize};

use crate::{
    controllers::{
        controller::{self, ControllerConfig, ControllerRecord},
        pid,
    },
    gui::{utils::text_singleline_with_apply, UIComponent},
    logger::is_enabled,
    navigators::{
        navigator::{self, NavigatorConfig, NavigatorRecord},
        trajectory_follower,
    },
    networking::{
        network::{Network, NetworkConfig},
        service_manager::ServiceManager,
    },
    node::Node,
    physics::{
        perfect_physic,
        physic::{self, PhysicConfig, PhysicRecord},
    },
    plugin_api::PluginAPI,
    sensors::sensor_manager::{SensorManager, SensorManagerConfig, SensorManagerRecord},
    simulator::{SimulatorConfig, TimeCv},
    state_estimators::{
        perfect_estimator,
        state_estimator::{
            self, BenchStateEstimator, BenchStateEstimatorConfig, BenchStateEstimatorRecord,
            StateEstimatorConfig, StateEstimatorRecord,
        },
    },
    utils::{
        determinist_random_variable::DeterministRandomVariableFactory,
        time_ordered_data::TimeOrderedData,
    },
};

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub enum NodeType {
    Robot,
    Sensor,
    Object,
    ComputationUnit,
}

impl NodeType {
    #[cfg(test)]
    const VALUES: [Self; 4] = [
        Self::Robot,
        Self::Sensor,
        Self::Object,
        Self::ComputationUnit,
    ];

    pub fn has_physics(&self) -> bool {
        match self {
            Self::Robot | Self::Object => true,
            Self::Sensor | Self::ComputationUnit => false,
        }
    }

    pub fn has_controller(&self) -> bool {
        match self {
            Self::Robot => true,
            Self::Object | Self::Sensor | Self::ComputationUnit => false,
        }
    }

    pub fn has_navigator(&self) -> bool {
        match self {
            Self::Robot => true,
            Self::Object | Self::Sensor | Self::ComputationUnit => false,
        }
    }

    pub fn has_state_estimator(&self) -> bool {
        match self {
            Self::Robot => true,
            Self::Object | Self::Sensor | Self::ComputationUnit => false,
        }
    }

    pub fn has_state_estimator_bench(&self) -> bool {
        match self {
            Self::Robot | Self::Sensor | Self::ComputationUnit => true,
            Self::Object => false,
        }
    }

    pub fn has_sensors(&self) -> bool {
        match self {
            Self::Robot | Self::Sensor | Self::ComputationUnit => true,
            Self::Object => false,
        }
    }

    pub fn has_network(&self) -> bool {
        match self {
            Self::Robot | Self::Sensor | Self::ComputationUnit => true,
            Self::Object => false,
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum NodeRecord {
    Robot(RobotRecord),
    ComputationUnit(ComputationUnitRecord),
}

impl NodeRecord {
    pub fn as_node_type(&self) -> NodeType {
        match &self {
            Self::Robot(_) => NodeType::Robot,
            Self::ComputationUnit(_) => NodeType::ComputationUnit,
        }
    }

    pub fn navigator(&self) -> Option<&NavigatorRecord> {
        match &self {
            Self::Robot(robot_record) => Some(&robot_record.navigator),
            Self::ComputationUnit(_) => None,
        }
    }

    pub fn controller(&self) -> Option<&ControllerRecord> {
        match &self {
            Self::Robot(robot_record) => Some(&robot_record.controller),
            Self::ComputationUnit(_) => None,
        }
    }

    pub fn physics(&self) -> Option<&PhysicRecord> {
        match &self {
            Self::Robot(robot_record) => Some(&robot_record.physic),
            Self::ComputationUnit(_) => None,
        }
    }

    pub fn state_estimator(&self) -> Option<&StateEstimatorRecord> {
        match &self {
            Self::Robot(robot_record) => Some(&robot_record.state_estimator),
            Self::ComputationUnit(_) => None,
        }
    }

    pub fn state_estimator_bench(&self) -> Option<&Vec<BenchStateEstimatorRecord>> {
        match &self {
            Self::Robot(robot_record) => Some(&robot_record.state_estimator_bench),
            Self::ComputationUnit(computation_unit_record) => {
                Some(&computation_unit_record.state_estimators)
            }
        }
    }

    pub fn sensor_manager(&self) -> Option<&SensorManagerRecord> {
        match &self {
            Self::Robot(robot_record) => Some(&robot_record.sensors),
            Self::ComputationUnit(r) => Some(&r.sensor_manager),
        }
    }
}

////////////////////////
//// ROBOT
////////////////////////

/// Configuration of the [`NodeType::Robot`].
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
            state_estimator: StateEstimatorConfig::Perfect(
                perfect_estimator::PerfectEstimatorConfig::default(),
            ),
            sensor_manager: SensorManagerConfig::default(),
            network: NetworkConfig::default(),
            state_estimator_bench: Vec::new(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for RobotConfig {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::HashMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        let name_copy = self.name.clone();
        let current_node_name = Some(&name_copy);
        egui::CollapsingHeader::new(&self.name).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Name: ");
                text_singleline_with_apply(
                    ui,
                    format!("robot-name-key-{}", unique_id).as_str(),
                    buffer_stack,
                    &mut self.name,
                );
            });

            self.network.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
            self.navigator.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
            self.physic.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
            self.controller.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
            self.state_estimator.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );

            ui.label("State estimator bench:");
            let mut seb_to_remove = None;
            for (i, seb) in self.state_estimator_bench.iter_mut().enumerate() {
                let seb_unique_id = format!("{}-{}", unique_id, &seb.name);
                ui.horizontal_top(|ui| {
                    seb.show(
                        ui,
                        ctx,
                        buffer_stack,
                        global_config,
                        current_node_name,
                        &seb_unique_id,
                    );
                    if ui.button("X").clicked() {
                        seb_to_remove = Some(i);
                    }
                });
            }
            if let Some(i) = seb_to_remove {
                self.state_estimator_bench.remove(i);
            }
            if ui.button("Add State Estimator benched").clicked() {
                self.state_estimator_bench
                    .push(BenchStateEstimatorConfig::default());
            }
            self.sensor_manager.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
        });
    }
}

/// State record of [`NodeType::Robot`].
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

////////////////////////
//// ComputationUnit
////////////////////////

/// Configuration of the [`NodeType::ComputationUnit`].
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct ComputationUnitConfig {
    /// Name of the unit.
    pub name: String,
    /// [`Network`] configuration.
    #[check]
    pub network: NetworkConfig,

    /// [`StateEstimator`]s
    #[check]
    pub state_estimators: Vec<BenchStateEstimatorConfig>,
}

impl Default for ComputationUnitConfig {
    /// Default configuration, using:
    /// * Default [`Network`] config.
    fn default() -> Self {
        ComputationUnitConfig {
            name: String::from("NoName"),
            network: NetworkConfig::default(),
            state_estimators: Vec::new(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ComputationUnitConfig {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::HashMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        let name_copy = self.name.clone();
        let current_node_name = Some(&name_copy);
        egui::CollapsingHeader::new(&self.name).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Name: ");
                text_singleline_with_apply(
                    ui,
                    format!("cu-name-key-{}", unique_id).as_str(),
                    buffer_stack,
                    &mut self.name,
                );
            });

            self.network.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );

            ui.label("State estimators:");
            let mut se_to_remove = None;
            for (i, seb) in self.state_estimators.iter_mut().enumerate() {
                let seb_unique_id = format!("{}-{}", unique_id, &seb.name);
                ui.horizontal_top(|ui| {
                    seb.show(
                        ui,
                        ctx,
                        buffer_stack,
                        global_config,
                        current_node_name,
                        &seb_unique_id,
                    );
                    if ui.button("X").clicked() {
                        se_to_remove = Some(i);
                    }
                });
            }
            if let Some(i) = se_to_remove {
                self.state_estimators.remove(i);
            }
            if ui.button("Add State Estimator").clicked() {
                self.state_estimators
                    .push(BenchStateEstimatorConfig::default());
            }
        });
    }
}

/// State record of [`NodeType::ComputationUnit`].
///
/// It contains the dynamic elements and the elements we want to save.
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ComputationUnitRecord {
    /// Name of the robot.
    pub name: String,
    pub state_estimators: Vec<BenchStateEstimatorRecord>,
    pub sensor_manager: SensorManagerRecord,
}

////////////////////////
//// Factory
////////////////////////

pub struct NodeFactory {}

impl NodeFactory {
    pub fn make_robot(
        config: &RobotConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &DeterministRandomVariableFactory,
        time_cv: Arc<TimeCv>,
    ) -> Node {
        let node_type = NodeType::Robot;
        let mut node = Node {
            node_type,
            name: config.name.clone(),
            navigator: Some(navigator::make_navigator_from_config(
                &config.navigator,
                plugin_api,
                global_config,
                va_factory,
            )),
            controller: Some(controller::make_controller_from_config(
                &config.controller,
                plugin_api,
                global_config,
                va_factory,
            )),
            physic: Some(physic::make_physic_from_config(
                &config.physic,
                plugin_api,
                global_config,
                va_factory,
            )),
            state_estimator: Some(Arc::new(RwLock::new(
                state_estimator::make_state_estimator_from_config(
                    &config.state_estimator,
                    plugin_api,
                    global_config,
                    va_factory,
                ),
            ))),
            sensor_manager: Some(Arc::new(RwLock::new(SensorManager::from_config(
                &config.sensor_manager,
                plugin_api,
                global_config,
                &config.name,
                va_factory,
            )))),
            network: Some(Arc::new(RwLock::new(Network::from_config(
                config.name.clone(),
                &config.network,
                global_config,
                va_factory,
                time_cv.clone(),
            )))),
            state_history: TimeOrderedData::new(),
            state_estimator_bench: Some(Arc::new(RwLock::new(Vec::with_capacity(
                config.state_estimator_bench.len(),
            )))),
            // services: Vec::new(),
            service_manager: None,
            node_server: None,
            other_node_names: Vec::new(),
            time_cv: time_cv.clone(),
        };

        for state_estimator_config in &config.state_estimator_bench {
            node.state_estimator_bench
                .as_ref()
                .unwrap()
                .write()
                .unwrap()
                .push(BenchStateEstimator {
                    name: state_estimator_config.name.clone(),
                    state_estimator: Arc::new(RwLock::new(
                        state_estimator::make_state_estimator_from_config(
                            &state_estimator_config.config,
                            plugin_api,
                            global_config,
                            va_factory,
                        ),
                    )),
                })
        }

        let service_manager = Some(Arc::new(RwLock::new(ServiceManager::initialize(
            &node, time_cv,
        ))));
        if plugin_api.is_some() {
            if let Some(message_handlers) = plugin_api.as_ref().unwrap().get_message_handlers(&node)
            {
                let mut network = node.network.as_ref().unwrap().write().unwrap();
                for message_handler in message_handlers {
                    network.subscribe(message_handler.clone());
                }
            }
        }
        // Services
        if is_enabled(crate::logger::InternalLog::SetupSteps) {
            debug!("Setup services");
        }
        node.service_manager = service_manager;

        node
    }

    pub fn make_computation_unit(
        config: &ComputationUnitConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &DeterministRandomVariableFactory,
        time_cv: Arc<TimeCv>,
    ) -> Node {
        let node_type = NodeType::ComputationUnit;
        let mut node = Node {
            node_type,
            name: config.name.clone(),
            navigator: None,
            controller: None,
            physic: None,
            state_estimator: None,
            sensor_manager: Some(Arc::new(RwLock::new(SensorManager::from_config(
                &SensorManagerConfig::default(),
                plugin_api,
                global_config,
                &config.name,
                va_factory,
            )))),
            network: Some(Arc::new(RwLock::new(Network::from_config(
                config.name.clone(),
                &config.network,
                global_config,
                va_factory,
                time_cv.clone(),
            )))),
            state_history: TimeOrderedData::new(),
            state_estimator_bench: Some(Arc::new(RwLock::new(Vec::with_capacity(
                config.state_estimators.len(),
            )))),
            service_manager: None,
            node_server: None,
            other_node_names: Vec::new(),
            time_cv: time_cv.clone(),
        };

        for state_estimator_config in &config.state_estimators {
            node.state_estimator_bench
                .as_ref()
                .unwrap()
                .write()
                .unwrap()
                .push(BenchStateEstimator {
                    name: state_estimator_config.name.clone(),
                    state_estimator: Arc::new(RwLock::new(
                        state_estimator::make_state_estimator_from_config(
                            &state_estimator_config.config,
                            plugin_api,
                            global_config,
                            va_factory,
                        ),
                    )),
                })
        }

        let service_manager = Some(Arc::new(RwLock::new(ServiceManager::initialize(
            &node, time_cv,
        ))));
        if plugin_api.is_some() {
            if let Some(message_handlers) = plugin_api.as_ref().unwrap().get_message_handlers(&node)
            {
                let mut network = node.network.as_ref().unwrap().write().unwrap();
                for message_handler in message_handlers {
                    network.subscribe(message_handler.clone());
                }
            }
        }
        // Services
        if is_enabled(crate::logger::InternalLog::SetupSteps) {
            debug!("Setup services");
        }
        node.service_manager = service_manager;

        node
    }
}

#[cfg(test)]
mod tests {

    mod node_type_rules {
        use crate::node_factory::NodeType;

        // The chain state_estimator -> navigator -> controller works together
        // The controller has no purpose if there is nothing to control => needs physics
        #[test]
        fn controller_needs_physics() {
            for node_type in NodeType::VALUES {
                if node_type.has_controller() {
                    assert!(node_type.has_physics());
                }
            }
        }

        #[test]
        fn controller_needs_navigator() {
            for node_type in NodeType::VALUES {
                if node_type.has_controller() {
                    assert!(node_type.has_navigator());
                }
            }
        }

        #[test]
        fn navigator_needs_state_estimator() {
            for node_type in NodeType::VALUES {
                if node_type.has_navigator() {
                    assert!(node_type.has_state_estimator());
                }
            }
        }
    }
}
