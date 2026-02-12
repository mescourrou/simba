use std::{
    str::FromStr,
    sync::{Arc, RwLock},
};

use log::debug;
use serde::{Deserialize, Serialize};
use simba_com::pub_sub::{BrokerTrait, PathKey};
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::text_singleline_with_apply};

use crate::{
    controllers::{self, ControllerConfig, ControllerRecord, pid},
    errors::{SimbaError, SimbaErrorTypes, SimbaResult},
    logger::is_enabled,
    navigators::{self, NavigatorConfig, NavigatorRecord, go_to},
    networking::{
        self,
        network::{Network, NetworkConfig},
        service_manager::ServiceManager,
    },
    node::{Node, NodeMetaData, NodeState},
    physics::{self, PhysicsConfig, PhysicsRecord, internal_physics},
    plugin_api::PluginAPI,
    sensors::sensor_manager::{SensorManager, SensorManagerConfig, SensorManagerRecord},
    simulator::{SimbaBroker, SimbaBrokerMultiClient, SimulatorConfig, TimeCv},
    state_estimators::{
        self, BenchStateEstimator, BenchStateEstimatorConfig, BenchStateEstimatorRecord, State,
        StateEstimatorConfig, StateEstimatorRecord, perfect_estimator,
    },
    time_analysis::TimeAnalysisFactory,
    utils::{SharedRwLock, determinist_random_variable::DeterministRandomVariableFactory},
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
    // Box RobotRecord to reduce size of NodeRecord
    Robot(Box<RobotRecord>),
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

    pub fn physics(&self) -> Option<&PhysicsRecord> {
        match &self {
            Self::Robot(robot_record) => Some(&robot_record.physics),
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

    pub fn name(&self) -> &String {
        match &self {
            Self::Robot(robot_record) => &robot_record.name,
            Self::ComputationUnit(r) => &r.name,
        }
    }
}

////////////////////////
/*        ROBOT       */
////////////////////////

/// Configuration of the [`NodeType::Robot`].
#[config_derives]
pub struct RobotConfig {
    /// Name of the robot.
    pub name: String,
    /// [`Navigator`](crate::navigators::navigator::Navigator) to use, and its configuration.
    #[check]
    pub navigator: NavigatorConfig,
    /// [`Controller`](crate::controllers::Controller) to use, and its configuration.
    #[check]
    pub controller: ControllerConfig,
    /// [`Physics`](crate::physics::physics::Physics) to use, and its configuration.
    #[check]
    pub physics: PhysicsConfig,
    /// [`StateEstimator`](crate::state_estimators::state_estimator::StateEstimator) to use, and its configuration.
    #[check]
    pub state_estimator: StateEstimatorConfig,
    /// [`SensorManager`] configuration, which defines the [`Sensor`](crate::sensors::sensor::Sensor)s used.
    #[check]
    pub sensor_manager: SensorManagerConfig,
    /// [`Network`] configuration.
    #[check]
    pub network: NetworkConfig,

    /// Additional [`StateEstimator`](crate::state_estimators::state_estimator::StateEstimator) to be evaluated but without a feedback
    /// loop with the [`Navigator`](crate::navigators::navigator::Navigator)
    #[check]
    pub state_estimator_bench: Vec<BenchStateEstimatorConfig>,
    pub autospawn: bool,
    pub labels: Vec<String>,
}

impl Default for RobotConfig {
    /// Default configuration, using:
    /// * Default [`GoTo`](go_to::GoTo) navigator.
    /// * Default [`PID`](pid::PID) controller.
    /// * Default [`PerfectPhysics`](perfect_physics::PerfectPhysics) physics.
    /// * Default [`PerfectEstimator`](perfect_estimator::PerfectEstimator) state estimator.
    /// * Default [`SensorManager`] config (no sensors).
    /// * Default [`Network`] config.
    fn default() -> Self {
        RobotConfig {
            name: String::from("NoName"),
            navigator: NavigatorConfig::GoTo(go_to::GoToConfig::default()),
            controller: ControllerConfig::PID(pid::PIDConfig::default()),
            physics: PhysicsConfig::Internal(internal_physics::InternalPhysicConfig::default()),
            state_estimator: StateEstimatorConfig::Perfect(
                perfect_estimator::PerfectEstimatorConfig::default(),
            ),
            sensor_manager: SensorManagerConfig::default(),
            network: NetworkConfig::default(),
            state_estimator_bench: Vec::new(),
            autospawn: true,
            labels: Vec::new(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for RobotConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
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

            ui.horizontal(|ui| {
                ui.label("Labels: ");

                let mut to_remove = Vec::new();
                for (i, label) in self.labels.iter_mut().enumerate() {
                    let unique_var_id = format!("robot-labels-key-{}-{}", i, unique_id);
                    ui.horizontal(|ui| {
                        text_singleline_with_apply(ui, &unique_var_id, buffer_stack, label);
                    });
                    if ui.button("-").clicked() {
                        to_remove.push(i);
                    }
                }
                for i in to_remove.iter().rev() {
                    self.labels.remove(*i);
                }
                if ui.button("+").clicked() {
                    self.labels.push(String::new());
                }
            });

            ui.horizontal(|ui| {
                ui.label("Autospawn:");
                ui.checkbox(&mut self.autospawn, "");
            });
            self.network.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
            self.navigator.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
            self.physics.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
            self.controller.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
            self.state_estimator.show_mut(
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
                    seb.show_mut(
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
            self.sensor_manager.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
        });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new(&self.name).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!("Name: {}", self.name));
            });

            ui.horizontal(|ui| {
                ui.label("Labels: ");
                ui.vertical(|ui| {
                    for label in &self.labels {
                        ui.label(format!("- '{}'", label));
                    }
                });
            });

            ui.horizontal(|ui| {
                ui.label(format!("Autospawn: {}", self.autospawn));
            });

            self.network.show(ui, ctx, unique_id);
            self.navigator.show(ui, ctx, unique_id);
            self.physics.show(ui, ctx, unique_id);
            self.controller.show(ui, ctx, unique_id);
            self.state_estimator.show(ui, ctx, unique_id);

            ui.label("State estimator bench:");
            for seb in &self.state_estimator_bench {
                let seb_unique_id = format!("{}-{}", unique_id, &seb.name);
                ui.horizontal_top(|ui| {
                    seb.show(ui, ctx, &seb_unique_id);
                });
            }

            self.sensor_manager.show(ui, ctx, unique_id);
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
    pub model_name: String,
    /// Record of the [`Navigator`](crate::navigators::navigator::Navigator) module.
    pub navigator: NavigatorRecord,
    /// Record of the [`Controller`](crate::controllers::Controller) module.
    pub controller: ControllerRecord,
    /// Record of the [`Physics`](crate::physics::physics::Physics) module.
    pub physics: PhysicsRecord,
    /// Record of the [`StateEstimator`](crate::state_estimators::state_estimator::StateEstimator) module.
    pub state_estimator: StateEstimatorRecord,
    /// Record of the additionnal [`StateEstimator`](crate::state_estimators::state_estimator::StateEstimator)s, only to evaluate them.
    pub state_estimator_bench: Vec<BenchStateEstimatorRecord>,

    pub sensors: SensorManagerRecord,
    pub state: NodeState,
    pub labels: Vec<String>,
}

#[cfg(feature = "gui")]
impl UIComponent for RobotRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!("Name: {}", self.name));

            ui.label(format!("Model Name: {}", self.model_name));

            ui.label("Labels:");
            ui.vertical(|ui| {
                for label in &self.labels {
                    ui.label(format!("- '{}'", label));
                }
            });

            ui.label(format!("State: {}", self.state));

            egui::CollapsingHeader::new("Navigator").show(ui, |ui| {
                self.navigator.show(ui, ctx, unique_id);
            });

            egui::CollapsingHeader::new("Controller").show(ui, |ui| {
                self.controller.show(ui, ctx, unique_id);
            });

            egui::CollapsingHeader::new("Physics").show(ui, |ui| {
                self.physics.show(ui, ctx, unique_id);
            });

            egui::CollapsingHeader::new("State Estimator").show(ui, |ui| {
                self.state_estimator.show(ui, ctx, unique_id);
            });

            ui.label("State Estimator bench:");
            for se in &self.state_estimator_bench {
                egui::CollapsingHeader::new(&se.name).show(ui, |ui| {
                    se.record.show(ui, ctx, unique_id);
                });
            }

            egui::CollapsingHeader::new("Sensors").show(ui, |ui| {
                self.sensors.show(ui, ctx, unique_id);
            });
        });
    }
}

////////////////////////
/* ComputationUnit    */
////////////////////////

/// Configuration of the [`NodeType::ComputationUnit`].
#[config_derives]
pub struct ComputationUnitConfig {
    /// Name of the unit.
    pub name: String,
    /// [`Network`] configuration.
    #[check]
    pub network: NetworkConfig,

    /// [`StateEstimator`](crate::state_estimators::state_estimator::StateEstimator)s
    #[check]
    pub state_estimators: Vec<BenchStateEstimatorConfig>,

    pub labels: Vec<String>,
}

impl Default for ComputationUnitConfig {
    /// Default configuration, using:
    /// * Default [`Network`] config.
    fn default() -> Self {
        ComputationUnitConfig {
            name: String::from("NoName"),
            network: NetworkConfig::default(),
            state_estimators: Vec::new(),
            labels: Vec::new(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ComputationUnitConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
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

            ui.horizontal(|ui| {
                ui.label("Labels: ");

                let mut to_remove = Vec::new();
                for (i, label) in self.labels.iter_mut().enumerate() {
                    let unique_var_id = format!("cu-labels-key-{}-{}", i, unique_id);
                    ui.horizontal(|ui| {
                        text_singleline_with_apply(ui, &unique_var_id, buffer_stack, label);
                    });
                    if ui.button("-").clicked() {
                        to_remove.push(i);
                    }
                }
                for i in to_remove.iter().rev() {
                    self.labels.remove(*i);
                }
                if ui.button("+").clicked() {
                    self.labels.push(String::new());
                }
            });

            self.network.show_mut(
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
                    seb.show_mut(
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

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new(&self.name).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!("Name: {}", self.name));
            });

            ui.horizontal(|ui| {
                ui.label("Labels: ");
                ui.vertical(|ui| {
                    for label in &self.labels {
                        ui.label(format!("- '{}'", label));
                    }
                });
            });

            self.network.show(ui, ctx, unique_id);

            ui.label("State estimators:");
            for seb in &self.state_estimators {
                let seb_unique_id = format!("{}-{}", unique_id, &seb.name);
                ui.horizontal_top(|ui| {
                    seb.show(ui, ctx, &seb_unique_id);
                });
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
    pub model_name: String,
    pub labels: Vec<String>,
}

#[cfg(feature = "gui")]
impl UIComponent for ComputationUnitRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!("Name: {}", self.name));

            ui.label(format!("Model Name: {}", self.model_name));

            ui.label("Labels:");
            ui.vertical(|ui| {
                for label in &self.labels {
                    ui.label(format!("- '{}'", label));
                }
            });

            ui.label("State Estimators:");
            for se in &self.state_estimators {
                egui::CollapsingHeader::new(&se.name).show(ui, |ui| {
                    se.record.show(ui, ctx, unique_id);
                });
            }

            egui::CollapsingHeader::new("Sensor Manager").show(ui, |ui| {
                self.sensor_manager.show(ui, ctx, unique_id);
            });
        });
    }
}

////////////////////////
/*      Factory       */
////////////////////////

pub struct MakeNodeParams<'a> {
    pub plugin_api: &'a Option<Arc<dyn PluginAPI>>,
    pub global_config: &'a SimulatorConfig,
    pub va_factory: &'a Arc<DeterministRandomVariableFactory>,
    pub time_analysis_factory: Option<&'a mut TimeAnalysisFactory>,
    pub time_cv: Arc<TimeCv>,
    pub force_send_results: bool,
    pub new_name: Option<&'a str>,
    pub broker: &'a SharedRwLock<SimbaBroker>,
    pub initial_time: f32,
}

pub struct NodeFactory {}

impl NodeFactory {
    fn make_global_channels(
        node_name: &String,
        broker: &SharedRwLock<SimbaBroker>,
    ) -> SimbaResult<SimbaBrokerMultiClient> {
        if is_enabled(crate::logger::InternalLog::NetworkMessages) {
            debug!("Setup global channels for node '{}'", node_name);
        }
        let mut broker_lock = broker.write().unwrap();
        broker_lock.add_channel(
            PathKey::from_str(networking::channels::internal::COMMAND)
                .unwrap()
                .join_str(node_name.as_str()),
        );
        let log_key = PathKey::from_str(networking::channels::internal::LOG)
            .unwrap()
            .join_str(node_name.as_str());
        broker_lock.add_channel(
            log_key
                .clone()
                .join_str(networking::channels::internal::log::ERROR),
        );
        broker_lock.add_channel(
            log_key
                .clone()
                .join_str(networking::channels::internal::log::WARNING),
        );
        broker_lock.add_channel(
            log_key
                .clone()
                .join_str(networking::channels::internal::log::INFO),
        );
        broker_lock.add_channel(
            log_key
                .clone()
                .join_str(networking::channels::internal::log::DEBUG),
        );
        if is_enabled(crate::logger::InternalLog::NetworkMessages) {
            debug!(
                "Broker channels:\n- {}",
                broker_lock
                    .channel_list()
                    .iter()
                    .map(|c| c.to_string())
                    .collect::<Vec<String>>()
                    .join("\n- ")
            );
        }
        std::mem::drop(broker_lock);
        let mut client = SimbaBrokerMultiClient::new(
            broker.clone(),
            node_name.clone(),
            0.,
            PathKey::from_str("/").unwrap(),
        );
        broker
            .write()
            .unwrap()
            .subscribe_to_list(
                &[PathKey::from_str(networking::channels::internal::COMMAND)
                    .unwrap()
                    .join_str(node_name.as_str())],
                0.,
                &mut client,
            )
            .unwrap();
        Ok(client)
    }

    pub fn make_robot(config: &RobotConfig, params: &mut MakeNodeParams) -> SimbaResult<Node> {
        let node_type = NodeType::Robot;
        // Make global channels
        let client = Self::make_global_channels(&config.name, params.broker)?;
        let network = Arc::new(RwLock::new(Network::from_config(
            config.name.clone(),
            &config.network,
            params.global_config,
            params.va_factory,
            params.broker,
            params.initial_time,
        )));
        let from_config_args = FromConfigArguments {
            global_config: params.global_config,
            initial_time: params.initial_time,
            network: &network,
            node_name: &config.name,
            plugin_api: params.plugin_api,
            va_factory: params.va_factory,
        };
        let physics = physics::make_physics_from_config(&config.physics, &from_config_args)?;
        let initial_state = physics.read().unwrap().state(params.initial_time).clone();
        let mut node = Node {
            node_meta_data: Arc::new(RwLock::new(NodeMetaData {
                name: params.new_name.unwrap_or(&config.name).to_string(),
                node_type,
                model_name: config.name.clone(),
                labels: config.labels.clone(),
                state: if config.autospawn {
                    NodeState::Running
                } else {
                    NodeState::Created
                },
                position: {
                    let pose = physics.read().unwrap().state(params.initial_time).pose;
                    Some([pose.x, pose.y])
                },
            })),
            navigator: Some(navigators::make_navigator_from_config(
                &config.navigator,
                params.plugin_api,
                params.global_config,
                params.va_factory,
                &network,
                params.initial_time,
            )?),
            controller: Some(controllers::make_controller_from_config(
                &config.controller,
                params.plugin_api,
                params.global_config,
                params.va_factory,
                &config.physics,
                &network,
                params.initial_time,
            )?),
            physics: Some(physics),
            state_estimator: Some(Arc::new(RwLock::new(
                state_estimators::make_state_estimator_from_config(
                    &config.state_estimator,
                    params.plugin_api,
                    params.global_config,
                    params.va_factory,
                    &network,
                    params.initial_time,
                )?,
            ))),
            sensor_manager: Some(Arc::new(RwLock::new(SensorManager::from_config(
                &config.sensor_manager,
                &from_config_args,
                &initial_state,
            )?))),
            network: Some(network.clone()),
            state_estimator_bench: Some(Arc::new(RwLock::new(Vec::with_capacity(
                config.state_estimator_bench.len(),
            )))),
            // services: Vec::new(),
            service_manager: None,
            node_server: None,
            other_node_names: Vec::new(),
            time_analysis: params
                .time_analysis_factory
                .as_mut()
                .map(|taf| taf.new_node(config.name.clone())),
            send_records: params.force_send_results || params.global_config.results.is_some(),
            meta_data_list: None,
            node_message_client: client,
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
                        state_estimators::make_state_estimator_from_config(
                            &state_estimator_config.config,
                            params.plugin_api,
                            params.global_config,
                            params.va_factory,
                            &network,
                            params.initial_time,
                        )?,
                    )),
                })
        }

        let service_manager = Some(Arc::new(RwLock::new(ServiceManager::initialize(
            &node,
            params.time_cv.clone(),
        ))));
        // Services
        if is_enabled(crate::logger::InternalLog::SetupSteps) {
            debug!("Setup services");
        }
        node.service_manager = service_manager;

        Ok(node)
    }

    pub fn make_computation_unit(
        config: &ComputationUnitConfig,
        params: &mut MakeNodeParams,
    ) -> SimbaResult<Node> {
        let node_type = NodeType::ComputationUnit;
        let client = Self::make_global_channels(&config.name, params.broker)?;
        let network = Arc::new(RwLock::new(Network::from_config(
            config.name.clone(),
            &config.network,
            params.global_config,
            params.va_factory,
            params.broker,
            params.initial_time,
        )));
        let from_config_args = FromConfigArguments {
            global_config: params.global_config,
            initial_time: params.initial_time,
            network: &network,
            node_name: &config.name,
            plugin_api: params.plugin_api,
            va_factory: params.va_factory,
        };
        let mut node = Node {
            node_meta_data: Arc::new(RwLock::new(NodeMetaData {
                name: params.new_name.unwrap_or(&config.name).to_string(),
                node_type,
                model_name: config.name.clone(),
                labels: config.labels.clone(),
                state: NodeState::Running,
                position: None,
            })),
            navigator: None,
            controller: None,
            physics: None,
            state_estimator: None,
            sensor_manager: Some(Arc::new(RwLock::new(SensorManager::from_config(
                &SensorManagerConfig::default(),
                &from_config_args,
                &State::default(),
            )?))),
            network: Some(network.clone()),
            state_estimator_bench: Some(Arc::new(RwLock::new(Vec::with_capacity(
                config.state_estimators.len(),
            )))),
            service_manager: None,
            node_server: None,
            other_node_names: Vec::new(),
            time_analysis: params
                .time_analysis_factory
                .as_mut()
                .map(|taf| taf.new_node(config.name.clone())),
            send_records: params.force_send_results || params.global_config.results.is_some(),
            meta_data_list: None,
            node_message_client: client,
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
                        state_estimators::make_state_estimator_from_config(
                            &state_estimator_config.config,
                            params.plugin_api,
                            params.global_config,
                            params.va_factory,
                            &network,
                            params.initial_time,
                        )?,
                    )),
                })
        }

        let service_manager = Some(Arc::new(RwLock::new(ServiceManager::initialize(
            &node,
            params.time_cv.clone(),
        ))));
        // Services
        if is_enabled(crate::logger::InternalLog::SetupSteps) {
            debug!("Setup services");
        }
        node.service_manager = service_manager;

        Ok(node)
    }

    pub fn make_node_from_name(name: &str, params: &mut MakeNodeParams) -> SimbaResult<Node> {
        for robot_config in params.global_config.robots.iter() {
            if robot_config.name == name {
                return Self::make_robot(robot_config, params);
            }
        }

        for cu_config in params.global_config.computation_units.iter() {
            if cu_config.name == name {
                return Self::make_computation_unit(cu_config, params);
            }
        }

        Err(SimbaError::new(
            SimbaErrorTypes::ImplementationError,
            format!("Node `{}` unknown in configuration: cannot create", name),
        ))
    }
}

pub struct FromConfigArguments<'a> {
    pub plugin_api: &'a Option<Arc<dyn PluginAPI>>,
    pub global_config: &'a SimulatorConfig,
    pub node_name: &'a String,
    pub va_factory: &'a Arc<DeterministRandomVariableFactory>,
    pub network: &'a SharedRwLock<Network>,
    pub initial_time: f32,
}

#[cfg(test)]
mod tests {

    mod node_type_rules {
        use super::super::NodeType;

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
