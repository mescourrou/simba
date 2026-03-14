//! Robot sensor implementation.
//!
//! This module provides a [`Sensor`] that observes other robots relative to the ego robot frame.
//! It supports configurable filters through
//! [`RobotSensorFilterConfig`] and fault pipelines configured by [`RobotSensorFaultModelConfig`].

use super::fault_models::fault_model::FaultModel;
use super::{Sensor, SensorObservation, SensorRecord};

use crate::constants::TIME_ROUND;

use crate::errors::{SimbaErrorTypes, SimbaResult};
#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::logger::is_enabled;
use crate::networking::service_manager::ServiceError;
use crate::plugin_api::PluginAPI;
use crate::recordable::Recordable;
use crate::sensors::fault_models::additive::{AdditiveFault, AdditiveFaultConfig};
use crate::sensors::fault_models::clutter::{ClutterFault, ClutterFaultConfig};
use crate::sensors::fault_models::external_fault::{ExternalFault, ExternalFaultConfig};
use crate::sensors::fault_models::misassociation::{
    MisassociationFault, MisassociationFaultConfig,
};
use crate::sensors::fault_models::misdetection::{MisdetectionFault, MisdetectionFaultConfig};
use crate::sensors::fault_models::python_fault_model::{PythonFaultModel, PythonFaultModelConfig};
use crate::sensors::sensor_filters::SensorFilter;
use crate::sensors::sensor_filters::external_filter::{ExternalFilter, ExternalFilterConfig};
use crate::sensors::sensor_filters::python_filter::{PythonFilter, PythonFilterConfig};
use crate::sensors::sensor_filters::range_filter::{RangeFilter, RangeFilterConfig};
use crate::sensors::sensor_filters::string_filter::{StringFilter, StringFilterConfig};
use crate::simulator::SimulatorConfig;
use crate::state_estimators::State;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::enum_tools::EnumVariables;
use crate::utils::periodicity::{Periodicity, PeriodicityConfig};
use serde_derive::{Deserialize, Serialize};

use log::debug;
extern crate nalgebra as na;
use na::Vector3;
use simba_macros::{EnumToString, UIComponent, config_derives, enum_variables};

use std::sync::Arc;

enum_variables!(
    "Variables used by robot observations, filters, and fault models."
    RobotSensorVariables;
    "Variables used by sensor filters."
    Filter,
    "Variables used by fault models."
    Faults:
    "Relative X coordinate."
    X, "x";
    Filter, Faults:
    "Relative Y coordinate."
    Y, "y";
    Filter, Faults:
    "Relative orientation (angle) of the observed robot compared to the ego robot."
    Orientation, "orientation", "z";
    Filter, Faults:
    "Relative distance to the observed robot."
    R, "r", "d", "distance";
    Filter, Faults:
    "Relative bearing angle to the observed robot."
    Theta, "theta" ;
    Filter:
    "Velocity of the observator."
    SelfVelocity, "self_velocity";
    Filter:
    "Absolute velocity of the observed robot."
    TargetVelocity, "target_velocity";
);

/// Configuration enum selecting robot sensor fault model strategies.
///
/// Default value: [`RobotSensorFaultModelConfig::AdditiveRobotCentered`] with
/// [`AdditiveFaultConfig::default`].
#[config_derives]
#[derive(UIComponent)]
#[show_all = "Faults"]
pub enum RobotSensorFaultModelConfig {
    /// Additive fault model in robot-centered coordinates.
    #[check]
    AdditiveRobotCentered(AdditiveFaultConfig<RobotSensorVariablesFaults, RobotSensorVariables>),
    /// Additive fault model in observation-centered coordinates.
    AdditiveObservationCentered(
        AdditiveFaultConfig<RobotSensorVariablesFaults, RobotSensorVariables>,
    ),
    /// Clutter fault model.
    #[check]
    Clutter(ClutterFaultConfig<RobotSensorVariablesFaults>),
    /// Misdetection fault model.
    #[check]
    Misdetection(MisdetectionFaultConfig),
    /// Misassociation fault model.
    #[check]
    Misassociation(MisassociationFaultConfig),
    /// Python-implemented fault model.
    #[check]
    Python(PythonFaultModelConfig),
    /// Plugin-provided external fault model.
    #[check]
    External(ExternalFaultConfig),
}

impl Default for RobotSensorFaultModelConfig {
    fn default() -> Self {
        Self::AdditiveRobotCentered(AdditiveFaultConfig::default())
    }
}

/// Runtime enum containing instantiated robot sensor fault models.
#[derive(Debug, EnumToString)]
pub enum RobotSensorFaultModelType {
    /// Instantiated additive robot-centered fault model.
    AdditiveRobotCentered(AdditiveFault<RobotSensorVariablesFaults, RobotSensorVariables>),
    /// Instantiated additive observation-centered fault model.
    AdditiveObservationCentered(AdditiveFault<RobotSensorVariablesFaults, RobotSensorVariables>),
    /// Instantiated clutter fault model.
    Clutter(ClutterFault<RobotSensorVariablesFaults>),
    /// Instantiated misdetection fault model.
    Misdetection(MisdetectionFault),
    /// Instantiated misassociation fault model.
    Misassociation(MisassociationFault),
    /// Instantiated Python fault model.
    Python(PythonFaultModel),
    /// Instantiated external fault model.
    External(ExternalFault),
}

impl RobotSensorFaultModelType {
    /// Initializes fault models that require runtime node context.
    pub fn post_init(
        &mut self,
        node: &mut crate::node::Node,
        initial_time: f32,
    ) -> crate::errors::SimbaResult<()> {
        match self {
            Self::Python(f) => f.post_init(node, initial_time),
            Self::External(f) => f.post_init(node, initial_time),
            Self::AdditiveRobotCentered(_)
            | Self::AdditiveObservationCentered(_)
            | Self::Clutter(_)
            | Self::Misdetection(_)
            | Self::Misassociation(_) => Ok(()),
        }
    }
}

/// Configuration enum selecting among multiple sensor observation filtering strategies for robot sensors.
///
/// This enum provides a unified interface for declaring sensor filters with different decision logic.
/// Each variant wraps the configuration for a specific filter type.
/// When multiple filters are applied to a sensor, all must agree to keep the observation.
///
/// Default value: [`RobotSensorFilterConfig::Range`] with [`RangeFilterConfig::default`].
///
/// # Config example:
/// ```yaml
/// filters:
///   - type: Range
///     variables: [r, theta]
///     max_range: [100, 1.57]
///     min_range: [1., -1.57]
///     inside: true
/// ```
#[config_derives]
#[derive(UIComponent)]
#[show_all = "Filter"]
pub enum RobotSensorFilterConfig {
    /// Range-based filtering on enumerated variables: excludes observations where numeric values fall outside specified bounds.
    #[check]
    Range(RangeFilterConfig<RobotSensorVariablesFilter>),
    /// String pattern filtering on observed object unique id (name for nodes, id for landmarks): excludes observations matching configured regexp patterns.
    #[check]
    Id(StringFilterConfig),
    /// String pattern filtering on observed object labels: excludes observations matching configured regexp patterns.
    #[check]
    Label(StringFilterConfig),
    /// Python-based custom filtering: delegates exclusion logic to user-defined Python methods.
    #[check]
    Python(PythonFilterConfig),
    /// Plugin-based custom filtering: delegates exclusion logic to external compiled or scripted plugins.
    #[check]
    External(ExternalFilterConfig),
}

impl Default for RobotSensorFilterConfig {
    fn default() -> Self {
        Self::Range(RangeFilterConfig::default())
    }
}

/// Runtime enum containing instantiated Robot sensor filters.
#[derive(Debug, EnumToString)]
pub enum RobotSensorFilterType {
    /// Instantiated range filter for Robot sensor variables.
    Range(RangeFilter<RobotSensorVariablesFilter>),
    /// Instantiated string filter for observed object unique id.
    Id(StringFilter),
    /// Instantiated string filter for observed object labels.
    Label(StringFilter),
    /// Instantiated Python filter for Robot sensor observations.
    Python(PythonFilter),
    /// Instantiated external filter for Robot sensor observations.
    External(ExternalFilter),
}

impl RobotSensorFilterType {
    /// Initializes filters that require runtime node context.
    pub fn post_init(
        &mut self,
        node: &mut crate::node::Node,
        initial_time: f32,
    ) -> crate::errors::SimbaResult<()> {
        match self {
            Self::Python(f) => f.post_init(node, initial_time),
            Self::External(f) => f.post_init(node, initial_time),
            Self::Range(_) | Self::Id(_) | Self::Label(_) => Ok(()),
        }
    }
}

/// Configuration of the [`RobotSensor`].
///
/// The robot sensor observes other robots relative to the ego robot frame.
///
/// Occlusions are handled by default, but can be bypassed by setting `xray` to `true`. In this case, the sensor will detect all robots within the `detection_distance` regardless of occlusions.
/// The height of the sensor is considered at 0., so all landmarks can occlude robots.
///
/// Default values:
/// - `detection_distance`: `5.0`
/// - `activation_time`: `Some(PeriodicityConfig { period: 0.1, offset: None, table: None })`
/// - `faults`: empty vector
/// - `filters`: empty vector
/// - `xray`: `false`
#[config_derives]
pub struct RobotSensorConfig {
    /// Max distance of detection.
    pub detection_distance: f32,
    /// Observation period of the sensor.
    #[check]
    pub activation_time: Option<PeriodicityConfig>,
    /// Fault model configurations applied after filtering.
    #[check]
    pub faults: Vec<RobotSensorFaultModelConfig>,
    /// Filter configurations applied before fault injection.
    #[check]
    pub filters: Vec<RobotSensorFilterConfig>,
    /// If `true`, line-of-sight occlusion checks are bypassed.
    pub xray: bool,
}

impl Check for RobotSensorConfig {
    fn do_check(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.detection_distance < 0. {
            errors.push(format!(
                "Detection distance should be positive, got {}",
                self.detection_distance
            ));
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

impl Default for RobotSensorConfig {
    fn default() -> Self {
        Self {
            detection_distance: 5.0,
            activation_time: Some(PeriodicityConfig {
                period: crate::config::NumberConfig::Num(0.1),
                offset: None,
                table: None,
            }),
            faults: Vec::new(),
            filters: Vec::new(),
            xray: false,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for RobotSensorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Robot sensor")
            .id_salt(format!("robot-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Detection distance:");
                    if self.detection_distance < 0. {
                        self.detection_distance = 0.;
                    }
                    ui.add(egui::DragValue::new(&mut self.detection_distance));
                });

                ui.horizontal(|ui| {
                    if let Some(p) = &mut self.activation_time {
                        p.show_mut(
                            ui,
                            ctx,
                            buffer_stack,
                            global_config,
                            current_node_name,
                            unique_id,
                        );
                        if ui.button("Remove activation").clicked() {
                            self.activation_time = None;
                        }
                    } else if ui.button("Add activation").clicked() {
                        self.activation_time = Some(Self::default().activation_time.unwrap());
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("X-Ray mode:");
                    ui.checkbox(&mut self.xray, "");
                });

                RobotSensorFilterConfig::show_all_mut(
                    &mut self.filters,
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );

                RobotSensorFaultModelConfig::show_all_mut(
                    &mut self.faults,
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
        egui::CollapsingHeader::new("Robot sensor")
            .id_salt(format!("robot-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(format!("Detection distance: {}", self.detection_distance));
                });

                ui.horizontal(|ui| {
                    if let Some(p) = &self.activation_time {
                        p.show(ui, ctx, unique_id);
                    } else {
                        ui.label("No activation");
                    }
                });

                ui.horizontal(|ui| {
                    ui.label(format!("X-Ray mode: {}", self.xray));
                });

                RobotSensorFilterConfig::show_all(&self.filters, ui, ctx, unique_id);

                RobotSensorFaultModelConfig::show_all(&self.faults, ui, ctx, unique_id);
            });
    }
}

/// Record of the [`RobotSensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct RobotSensorRecord {
    last_time: Option<f32>,
}

#[cfg(feature = "gui")]
impl UIComponent for RobotSensorRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.label(format!(
            "Last time: {}",
            match self.last_time {
                Some(t) => t.to_string(),
                None => "None".to_string(),
            }
        ));
    }
}

/// Observation of an Oriented Robot.
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct OrientedRobotObservation {
    /// Name of the Robot
    pub name: String,
    /// Labels associated with the observed robot.
    pub labels: Vec<String>,
    /// Pose of the Robot
    pub pose: Vector3<f32>,
    /// Fault models applied to this observation.
    pub applied_faults: Vec<RobotSensorFaultModelConfig>,
}

impl Recordable<OrientedRobotObservationRecord> for OrientedRobotObservation {
    fn record(&self) -> OrientedRobotObservationRecord {
        OrientedRobotObservationRecord {
            name: self.name.clone(),
            labels: self.labels.clone(),
            pose: self.pose.to_owned().into(),
        }
    }
}

/// Serializable record representation of an [`OrientedRobotObservation`].
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct OrientedRobotObservationRecord {
    /// Name of the Robot
    pub name: String,
    /// Labels associated with the observed robot.
    pub labels: Vec<String>,
    /// Pose of the Robot
    pub pose: [f32; 3],
}

#[cfg(feature = "gui")]
impl UIComponent for OrientedRobotObservationRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!("Name: {}", self.name));
            ui.label("Labels:");
            for label in &self.labels {
                ui.label(format!("- {}", label));
            }
            ui.label(format!(
                "Pose: ({}, {}, {})",
                self.pose[0], self.pose[1], self.pose[2]
            ));
        });
    }
}

/// Sensor which observe the other Robots.
#[derive(Debug)]
pub struct RobotSensor {
    /// Detection distance
    detection_distance: f32,
    /// Observation period
    activation_time: Option<Periodicity>,
    /// Last observation time.
    last_time: Option<f32>,
    xray: bool,
    faults: Vec<RobotSensorFaultModelType>,
    filters: Vec<RobotSensorFilterType>,
}

impl RobotSensor {
    /// Makes a new [`RobotSensor`] from the given config.
    ///
    /// The map path is relative to the config path of the simulator.
    pub fn from_config(
        config: &RobotSensorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> SimbaResult<Self> {
        let mut fault_models = Vec::new();
        for fault_config in &config.faults {
            fault_models.push(match &fault_config {
                RobotSensorFaultModelConfig::AdditiveRobotCentered(cfg) => {
                    RobotSensorFaultModelType::AdditiveRobotCentered(AdditiveFault::from_config(
                        cfg,
                        va_factory,
                        initial_time,
                    ))
                }
                RobotSensorFaultModelConfig::AdditiveObservationCentered(cfg) => {
                    RobotSensorFaultModelType::AdditiveObservationCentered(
                        AdditiveFault::from_config(cfg, va_factory, initial_time),
                    )
                }
                RobotSensorFaultModelConfig::Clutter(cfg) => RobotSensorFaultModelType::Clutter(
                    ClutterFault::from_config(cfg, va_factory, initial_time),
                ),
                RobotSensorFaultModelConfig::Misdetection(cfg) => {
                    RobotSensorFaultModelType::Misdetection(MisdetectionFault::from_config(
                        cfg,
                        va_factory,
                        initial_time,
                    ))
                }
                RobotSensorFaultModelConfig::Misassociation(cfg) => {
                    RobotSensorFaultModelType::Misassociation(MisassociationFault::from_config(
                        cfg, va_factory,
                    ))
                }
                RobotSensorFaultModelConfig::Python(cfg) => RobotSensorFaultModelType::Python(
                    PythonFaultModel::from_config(cfg, global_config, initial_time)?,
                ),
                RobotSensorFaultModelConfig::External(cfg) => {
                    RobotSensorFaultModelType::External(ExternalFault::from_config(
                        cfg,
                        plugin_api,
                        global_config,
                        va_factory,
                        initial_time,
                    )?)
                }
            });
        }

        let mut filters = Vec::new();
        for filter_config in &config.filters {
            filters.push(match &filter_config {
                RobotSensorFilterConfig::Range(cfg) => {
                    RobotSensorFilterType::Range(RangeFilter::from_config(cfg, initial_time))
                }
                RobotSensorFilterConfig::Id(cfg) => {
                    RobotSensorFilterType::Id(StringFilter::from_config(cfg, initial_time))
                }
                RobotSensorFilterConfig::Label(cfg) => {
                    RobotSensorFilterType::Label(StringFilter::from_config(cfg, initial_time))
                }
                RobotSensorFilterConfig::Python(cfg) => RobotSensorFilterType::Python(
                    PythonFilter::from_config(cfg, global_config, initial_time)?,
                ),
                RobotSensorFilterConfig::External(cfg) => {
                    RobotSensorFilterType::External(ExternalFilter::from_config(
                        cfg,
                        plugin_api,
                        global_config,
                        va_factory,
                        initial_time,
                    )?)
                }
            });
        }

        let period = config
            .activation_time
            .as_ref()
            .map(|p| Periodicity::from_config(p, va_factory, initial_time));
        Ok(Self {
            detection_distance: config.detection_distance,
            activation_time: period,
            last_time: None,
            faults: fault_models,
            xray: config.xray,
            filters,
        })
    }
}

use crate::node::Node;

impl Sensor for RobotSensor {
    fn post_init(&mut self, node: &mut Node, initial_time: f32) -> crate::errors::SimbaResult<()> {
        for filter in self.filters.iter_mut() {
            filter.post_init(node, initial_time)?;
        }
        for fault_model in self.faults.iter_mut() {
            fault_model.post_init(node, initial_time)?;
        }
        Ok(())
    }

    fn get_observations(&mut self, node: &mut Node, time: f32) -> Vec<SensorObservation> {
        let mut observation_list = Vec::<SensorObservation>::new();
        if let Some(last_time) = self.last_time
            && (time - last_time).abs() < TIME_ROUND
        {
            return observation_list;
        }
        if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
            debug!("Start looking for nodes");
        }
        let state = if let Some(arc_physics) = node.physics() {
            let physics = arc_physics.read().unwrap();
            physics.state(time).clone()
        } else {
            State::new() // 0
        };

        let rotation_matrix =
            nalgebra::geometry::Rotation3::from_euler_angles(0., 0., state.pose.z);
        if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
            debug!("Rotation matrix: {}", rotation_matrix);
        }

        for (i, other_node_name) in node.other_node_names().iter().enumerate() {
            if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
                debug!("Sensing node {}", other_node_name);
            }
            assert!(*other_node_name != node.name());

            let service_manager = node.service_manager();
            match service_manager.read().unwrap().get_real_state(
                &other_node_name.to_string(),
                node,
                time,
            ) {
                Ok(other_state) => {
                    if node.environment().is_target_observable(
                        &other_state.pose.fixed_rows::<2>(0).clone_owned(),
                        Some(0.),
                        &state.pose.fixed_rows::<2>(0).clone_owned(),
                        if self.xray { None } else { Some(0.) },
                        self.detection_distance,
                        Some(node.name().clone()),
                    ) {
                        let robot_seed =
                            (i as f32) / (100. * (time - self.last_time.unwrap_or(-1.)));
                        let pose = rotation_matrix.transpose() * (other_state.pose - state.pose);
                        let labels = node
                            .meta_data_list()
                            .unwrap()
                            .read()
                            .unwrap()
                            .get(other_node_name)
                            .map_or(Vec::new(), |md| md.read().unwrap().labels.clone());
                        let obs = SensorObservation::OrientedRobot(OrientedRobotObservation {
                            name: other_node_name.clone(),
                            labels,
                            pose,
                            applied_faults: Vec::new(),
                        });

                        let mut keep_observation = Some(obs);

                        for filter in self.filters.iter() {
                            if let Some(obs) = keep_observation {
                                keep_observation = match filter {
                                    RobotSensorFilterType::Python(f) => {
                                        f.filter(time, obs, &state, Some(&other_state))
                                    }
                                    RobotSensorFilterType::External(f) => {
                                        f.filter(time, obs, &state, Some(&other_state))
                                    }
                                    RobotSensorFilterType::Id(f) => {
                                        if let SensorObservation::OrientedRobot(obs) = obs {
                                            if f.match_exclusion(std::slice::from_ref(&obs.name)) {
                                                Some(SensorObservation::OrientedRobot(obs))
                                            } else {
                                                None
                                            }
                                        } else {
                                            unreachable!()
                                        }
                                    }
                                    RobotSensorFilterType::Label(f) => {
                                        if let SensorObservation::OrientedRobot(obs) = obs {
                                            if f.match_exclusion(&obs.labels) {
                                                Some(SensorObservation::OrientedRobot(obs))
                                            } else {
                                                None
                                            }
                                        } else {
                                            unreachable!()
                                        }
                                    }
                                    RobotSensorFilterType::Range(f) => {
                                        if let SensorObservation::OrientedRobot(obs) = obs {
                                            if f.match_exclusion(
                                                &RobotSensorVariablesFilter::mapped_values(|variant| {
                                                    match variant {
                                                        RobotSensorVariablesFilter::X => obs.pose.x,
                                                        RobotSensorVariablesFilter::Y => obs.pose.y,
                                                        RobotSensorVariablesFilter::Orientation => {
                                                            obs.pose.z
                                                        }
                                                        RobotSensorVariablesFilter::R => {
                                                            obs.pose.fixed_rows::<2>(0).norm()
                                                        }
                                                        RobotSensorVariablesFilter::Theta => {
                                                            obs.pose.y.atan2(obs.pose.x)
                                                        }
                                                        RobotSensorVariablesFilter::SelfVelocity => {
                                                            state.velocity.fixed_rows::<2>(0).norm()
                                                        }
                                                        RobotSensorVariablesFilter::TargetVelocity => {
                                                            other_state
                                                                .velocity
                                                                .fixed_rows::<2>(0)
                                                                .norm()
                                                        }
                                                    }
                                                }),
                                            ) {
                                                Some(SensorObservation::OrientedRobot(obs))
                                            } else {
                                                None
                                            }
                                        } else {
                                            unreachable!()
                                        }
                                    }
                                };
                            } else {
                                break;
                            }
                        }

                        let mut new_obs = Vec::new();
                        if let Some(observation) = keep_observation {
                            new_obs.push(observation); // Not adding directly to observation_list to apply faults only once
                            for fault_model in self.faults.iter_mut() {
                                match fault_model {
                                    RobotSensorFaultModelType::Python(f) => f.add_faults(
                                        time,
                                        time + robot_seed,
                                        &mut new_obs,
                                        SensorObservation::OrientedRobot(
                                            OrientedRobotObservation::default(),
                                        ),
                                        node.environment(),
                                    ),
                                    RobotSensorFaultModelType::External(f) => f.add_faults(
                                        time,
                                        time + robot_seed,
                                        &mut new_obs,
                                        SensorObservation::OrientedRobot(
                                            OrientedRobotObservation::default(),
                                        ),
                                        node.environment(),
                                    ),
                                    RobotSensorFaultModelType::AdditiveObservationCentered(f) => {
                                        let obs_list_len = new_obs.len();
                                        for (i, obs) in new_obs
                                            .iter_mut()
                                            .map(|o| {
                                                if let SensorObservation::OrientedRobot(
                                                    observation,
                                                ) = o
                                                {
                                                    observation
                                                } else {
                                                    unreachable!()
                                                }
                                            })
                                            .enumerate()
                                        {
                                            let seed =
                                                time + i as f32 / (100. * obs_list_len as f32);
                                            let new_values = f.add_faults(
                                                seed,
                                                RobotSensorVariablesFaults::mapped_values(
                                                    |variant| match variant {
                                                        RobotSensorVariablesFaults::X => obs.pose.x,
                                                        RobotSensorVariablesFaults::Y => obs.pose.y,
                                                        RobotSensorVariablesFaults::Orientation => {
                                                            obs.pose.z
                                                        }
                                                        RobotSensorVariablesFaults::R => 0.,
                                                        RobotSensorVariablesFaults::Theta => {
                                                            obs.pose.z
                                                        }
                                                    },
                                                ),
                                                &RobotSensorVariables::mapped_values(|variant| {
                                                    match variant {
                                                        RobotSensorVariables::Orientation => {
                                                            obs.pose.z
                                                        }
                                                        RobotSensorVariables::R => {
                                                            obs.pose.fixed_rows::<2>(0).norm()
                                                        }
                                                        RobotSensorVariables::Theta => obs.pose.z,
                                                        RobotSensorVariables::X => obs.pose.x,
                                                        RobotSensorVariables::Y => obs.pose.y,
                                                        RobotSensorVariables::SelfVelocity => {
                                                            state.velocity.fixed_rows::<2>(0).norm()
                                                        }
                                                        RobotSensorVariables::TargetVelocity => {
                                                            other_state
                                                                .velocity
                                                                .fixed_rows::<2>(0)
                                                                .norm()
                                                        }
                                                    }
                                                }),
                                            );
                                            if let Some(new_x) =
                                                new_values.get(&RobotSensorVariablesFaults::X)
                                            {
                                                obs.pose.x = *new_x;
                                            }
                                            if let Some(new_y) =
                                                new_values.get(&RobotSensorVariablesFaults::Y)
                                            {
                                                obs.pose.y = *new_y;
                                            }
                                            let new_r = if let Some(new_r) =
                                                new_values.get(&RobotSensorVariablesFaults::R)
                                            {
                                                *new_r
                                            } else {
                                                0.
                                            };
                                            let new_theta = if let Some(new_theta) =
                                                new_values.get(&RobotSensorVariablesFaults::Theta)
                                            {
                                                *new_theta
                                            } else {
                                                obs.pose.z
                                            };
                                            obs.pose.x += new_r * new_theta.cos();
                                            obs.pose.y += new_r * new_theta.sin();
                                            if let Some(new_orientation) = new_values
                                                .get(&RobotSensorVariablesFaults::Orientation)
                                            {
                                                obs.pose.z = *new_orientation;
                                            }
                                        }
                                    }
                                    RobotSensorFaultModelType::AdditiveRobotCentered(f) => {
                                        let obs_list_len = new_obs.len();
                                        for (i, obs) in new_obs
                                            .iter_mut()
                                            .map(|o| {
                                                if let SensorObservation::OrientedRobot(
                                                    observation,
                                                ) = o
                                                {
                                                    observation
                                                } else {
                                                    unreachable!()
                                                }
                                            })
                                            .enumerate()
                                        {
                                            let seed =
                                                time + i as f32 / (100. * obs_list_len as f32);
                                            let new_values = f.add_faults(
                                                seed,
                                                RobotSensorVariablesFaults::mapped_values(
                                                    |variant| match variant {
                                                        RobotSensorVariablesFaults::X => obs.pose.x,
                                                        RobotSensorVariablesFaults::Y => obs.pose.y,
                                                        RobotSensorVariablesFaults::Orientation => {
                                                            obs.pose.z
                                                        }
                                                        RobotSensorVariablesFaults::R => {
                                                            obs.pose.fixed_rows::<2>(0).norm()
                                                        }
                                                        RobotSensorVariablesFaults::Theta => {
                                                            obs.pose.y.atan2(obs.pose.x)
                                                        }
                                                    },
                                                ),
                                                &RobotSensorVariables::mapped_values(|variant| {
                                                    match variant {
                                                        RobotSensorVariables::Orientation => {
                                                            obs.pose.z
                                                        }
                                                        RobotSensorVariables::R => {
                                                            obs.pose.fixed_rows::<2>(0).norm()
                                                        }
                                                        RobotSensorVariables::Theta => {
                                                            obs.pose.y.atan2(obs.pose.x)
                                                        }
                                                        RobotSensorVariables::X => obs.pose.x,
                                                        RobotSensorVariables::Y => obs.pose.y,
                                                        RobotSensorVariables::SelfVelocity => {
                                                            state.velocity.fixed_rows::<2>(0).norm()
                                                        }
                                                        RobotSensorVariables::TargetVelocity => {
                                                            other_state
                                                                .velocity
                                                                .fixed_rows::<2>(0)
                                                                .norm()
                                                        }
                                                    }
                                                }),
                                            );
                                            if let Some(new_x) =
                                                new_values.get(&RobotSensorVariablesFaults::X)
                                            {
                                                obs.pose.x = *new_x;
                                            }
                                            if let Some(new_y) =
                                                new_values.get(&RobotSensorVariablesFaults::Y)
                                            {
                                                obs.pose.y = *new_y;
                                            }
                                            let new_r = if let Some(new_r) =
                                                new_values.get(&RobotSensorVariablesFaults::R)
                                            {
                                                *new_r
                                            } else {
                                                obs.pose.fixed_rows::<2>(0).norm()
                                            };
                                            let new_theta = if let Some(new_theta) =
                                                new_values.get(&RobotSensorVariablesFaults::Theta)
                                            {
                                                *new_theta
                                            } else {
                                                obs.pose.y.atan2(obs.pose.x)
                                            };
                                            obs.pose.x = new_r * new_theta.cos();
                                            obs.pose.y = new_r * new_theta.sin();
                                            if let Some(new_orientation) = new_values
                                                .get(&RobotSensorVariablesFaults::Orientation)
                                            {
                                                obs.pose.z = *new_orientation;
                                            }
                                        }
                                    }
                                    RobotSensorFaultModelType::Clutter(f) => {
                                        let new_obs_from_clutter =
                                            f.add_faults(time + robot_seed, robot_seed / 100.);
                                        for (obs_id, obs_params) in new_obs_from_clutter {
                                            let mut x = obs_params
                                                .get(&RobotSensorVariablesFaults::X)
                                                .cloned()
                                                .unwrap_or(0.);
                                            let mut y = obs_params
                                                .get(&RobotSensorVariablesFaults::Y)
                                                .cloned()
                                                .unwrap_or(0.);
                                            let orientation = obs_params
                                                .get(&RobotSensorVariablesFaults::Orientation)
                                                .cloned()
                                                .unwrap_or(0.);

                                            let r = obs_params
                                                .get(&RobotSensorVariablesFaults::R)
                                                .cloned()
                                                .unwrap_or(0.);
                                            let theta = obs_params
                                                .get(&RobotSensorVariablesFaults::Theta)
                                                .cloned()
                                                .unwrap_or(0.);
                                            x += r * theta.cos();
                                            y += r * theta.sin();

                                            let obs = SensorObservation::OrientedRobot(
                                                OrientedRobotObservation {
                                                    name: obs_id,
                                                    labels: Vec::new(),
                                                    pose: Vector3::new(x, y, orientation),
                                                    applied_faults: vec![
                                                        RobotSensorFaultModelConfig::Clutter(
                                                            f.config().clone(),
                                                        ),
                                                    ],
                                                },
                                            );
                                            new_obs.push(obs);
                                        }
                                    }
                                    RobotSensorFaultModelType::Misassociation(f) => {
                                        for (i, obs) in new_obs.iter_mut().enumerate() {
                                            if let SensorObservation::OrientedRobot(observation) =
                                                obs
                                            {
                                                let new_label = f.new_label(
                                                    time + robot_seed + (i as f32) / 1000.,
                                                    observation.name.clone(),
                                                    observation
                                                        .pose
                                                        .fixed_rows::<2>(0)
                                                        .clone_owned(),
                                                    node.environment(),
                                                );
                                                observation.name = new_label;
                                            } else {
                                                unreachable!()
                                            }
                                        }
                                    }
                                    RobotSensorFaultModelType::Misdetection(f) => {
                                        new_obs = new_obs
                                            .iter()
                                            .enumerate()
                                            .filter_map(|(i, obs)| {
                                                if let SensorObservation::OrientedRobot(
                                                    observation,
                                                ) = obs
                                                {
                                                    if f.detected(
                                                        time + robot_seed + (i as f32) / 1000.,
                                                    ) {
                                                        Some(SensorObservation::OrientedRobot(
                                                            observation.clone(),
                                                        ))
                                                    } else {
                                                        None
                                                    }
                                                } else {
                                                    unreachable!()
                                                }
                                            })
                                            .collect();
                                    }
                                }
                            }
                        } else if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
                            debug!(
                                "Observation of node {} was filtered out",
                                &other_node_name.to_string()
                            );
                        }
                        observation_list.extend(new_obs);
                    };
                }
                Err(e) => {
                    match e.error_type() {
                        SimbaErrorTypes::ServiceError(
                            ServiceError::Unavailable | ServiceError::Closed,
                        ) => (),
                        _ => log::error!(
                            "Error trying to get real state of node {}: {}",
                            &other_node_name.to_string(),
                            e.detailed_error()
                        ),
                    };
                }
            };
        }
        if let Some(p) = self.activation_time.as_mut() {
            p.update(time);
        }
        self.last_time = Some(time);
        observation_list
    }

    /// Get the next observation time.
    fn next_time_step(&self) -> f32 {
        if let Some(activation) = &self.activation_time {
            activation.next_time()
        } else {
            f32::INFINITY
        }
    }
}

impl Recordable<SensorRecord> for RobotSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::RobotSensor(RobotSensorRecord {
            last_time: self.last_time,
        })
    }
}
