use std::sync::{Arc, Condvar, Mutex, RwLock};

use config_checker::macros::Check;
use log::debug;
use serde::{Deserialize, Serialize};

use crate::{
    controllers::{
        controller::{self, ControllerConfig, ControllerRecord},
        pid,
    },
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
    simulator::{SimulatorConfig, TimeCvData},
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
            Self::Robot | Self::Sensor => true,
            Self::Object | Self::ComputationUnit => false,
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
            Self::ComputationUnit(_) => None,
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

/// State record of [`NodeType::ComputationUnit`].
///
/// It contains the dynamic elements and the elements we want to save.
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ComputationUnitRecord {
    /// Name of the robot.
    pub name: String,
    pub state_estimators: Vec<BenchStateEstimatorRecord>,
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
        time_cv: Arc<(Mutex<TimeCvData>, Condvar)>,
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
        debug!("Setup services");
        node.service_manager = service_manager;

        node
    }

    pub fn make_computation_unit(
        config: &ComputationUnitConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &DeterministRandomVariableFactory,
        time_cv: Arc<(Mutex<TimeCvData>, Condvar)>,
    ) -> Node {
        let node_type = NodeType::ComputationUnit;
        let mut node = Node {
            node_type,
            name: config.name.clone(),
            navigator: None,
            controller: None,
            physic: None,
            state_estimator: None,
            sensor_manager: None,
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
        debug!("Setup services");
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
