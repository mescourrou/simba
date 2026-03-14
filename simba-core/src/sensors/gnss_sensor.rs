//! GNSS-like sensor implementation.
//!
//! This sensor observes the global pose and planar velocity of the node, with optional filtering and fault injection.
//!
//! This module provides a [`Sensor`] that reports global pose and planar
//! velocity, with optional filtering and fault injection.
//! Filtering is configured through [`GNSSSensorFilterConfig`], and fault behavior
//! is configured through [`GNSSSensorFaultModelConfig`].

use std::sync::Arc;

use super::fault_models::fault_model::FaultModel;
use super::{Sensor, SensorObservation, SensorRecord};

use crate::constants::TIME_ROUND;
use crate::errors::SimbaResult;
#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::logger::is_enabled;
use crate::plugin_api::PluginAPI;
use crate::recordable::Recordable;
use crate::sensors::fault_models::additive::{AdditiveFault, AdditiveFaultConfig};
use crate::sensors::fault_models::clutter::{ClutterFault, ClutterFaultConfig};
use crate::sensors::fault_models::external_fault::{ExternalFault, ExternalFaultConfig};
use crate::sensors::fault_models::misdetection::{MisdetectionFault, MisdetectionFaultConfig};
use crate::sensors::fault_models::python_fault_model::{PythonFaultModel, PythonFaultModelConfig};
use crate::sensors::sensor_filters::SensorFilter;
use crate::sensors::sensor_filters::external_filter::{ExternalFilter, ExternalFilterConfig};
use crate::sensors::sensor_filters::python_filter::{PythonFilter, PythonFilterConfig};
use crate::sensors::sensor_filters::range_filter::{RangeFilter, RangeFilterConfig};
use crate::simulator::SimulatorConfig;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::enum_tools::EnumVariables;
use crate::utils::periodicity::{Periodicity, PeriodicityConfig};
use log::debug;
use nalgebra::{Vector2, Vector3};
use serde_derive::{Deserialize, Serialize};
use simba_macros::{EnumToString, UIComponent, config_derives, enum_variables};

extern crate nalgebra as na;

enum_variables!(
    "Variables used by GNSS observations, filters, and fault models."
    GNSSSensorVariables;
    "Variables accepted by GNSS filters."
    Filter,
    "Variables modified by GNSS fault models."
    Faults:
    "Global X position."
    X, "x", "position_x";
    Filter, Faults:
    "Global Y position."
    Y, "y", "position_y";
    Filter, Faults:
    "Global orientation (yaw angle)."
    Orientation, "orientation", "z";
    Filter, Faults:
    "Distance to the origin (sqrt(x^2 + y^2))."
    R, "r", "distance";
    Filter, Faults:
    "Angle to the x-axis (atan2(y, x))."
    Theta, "theta", "angle";
    Filter, Faults:
    "Global velocity in the x direction (in the global frame)."
    VelocityX, "velocity_x", "vx";
    Filter, Faults:
    "Global velocity in the y direction (in the global frame)."
    VelocityY, "velocity_y", "vy";
    Filter:
    "Self-velocity norm."
    SelfVelocity, "self_velocity";
);

/// Configuration enum selecting GNSS fault model strategies.
///
/// Default value: [`GNSSSensorFaultModelConfig::Additive`] with
/// [`AdditiveFaultConfig::default`].
#[config_derives]
#[derive(UIComponent)]
#[show_all = "Faults"]
pub enum GNSSSensorFaultModelConfig {
    /// Additive fault model in robot-centered coordinates.
    Additive(AdditiveFaultConfig<GNSSSensorVariablesFaults, GNSSSensorVariables>),
    /// Clutter fault model.
    Clutter(ClutterFaultConfig<GNSSSensorVariablesFaults>),
    /// Misdetection fault model.
    Misdetection(MisdetectionFaultConfig),
    /// Plugin-provided external fault model.
    External(ExternalFaultConfig),
    /// Python-implemented fault model.
    Python(PythonFaultModelConfig),
}

impl Default for GNSSSensorFaultModelConfig {
    fn default() -> Self {
        Self::Additive(AdditiveFaultConfig::default())
    }
}

/// Runtime enum containing instantiated GNSS fault models.
#[derive(Debug, EnumToString)]
pub enum GNSSSensorFaultModelType {
    /// Instantiated additive robot-centered fault model.
    Additive(AdditiveFault<GNSSSensorVariablesFaults, GNSSSensorVariables>),
    /// Instantiated clutter fault model.
    Clutter(ClutterFault<GNSSSensorVariablesFaults>),
    /// Instantiated misdetection fault model.
    Misdetection(MisdetectionFault),
    /// Instantiated external fault model.
    External(ExternalFault),
    /// Instantiated Python fault model.
    Python(PythonFaultModel),
}

impl GNSSSensorFaultModelType {
    /// Initializes fault models that require runtime node context.
    pub fn post_init(
        &mut self,
        node: &mut crate::node::Node,
        initial_time: f32,
    ) -> crate::errors::SimbaResult<()> {
        match self {
            Self::Python(f) => f.post_init(node, initial_time),
            Self::External(f) => f.post_init(node, initial_time),
            Self::Additive(_) | Self::Clutter(_) | Self::Misdetection(_) => Ok(()),
        }
    }
}

/// Configuration enum selecting GNSS sensor filtering strategies
/// for GNSSSensor observations.
///
/// When multiple filters are applied to a sensor, all must agree to keep the observation.
///
/// Default value: [`GNSSSensorFilterConfig::Range`] with [`RangeFilterConfig::default`].
#[config_derives]
#[derive(UIComponent)]
#[show_all = "Faults"]
pub enum GNSSSensorFilterConfig {
    /// Range-based filtering on enumerated variables: excludes observations where numeric values fall outside specified bounds.
    #[check]
    Range(RangeFilterConfig<GNSSSensorVariablesFilter>),
    /// Python-based custom filtering: delegates exclusion logic to user-defined Python methods.
    #[check]
    Python(PythonFilterConfig),
    /// Plugin-based custom filtering: delegates exclusion logic to external compiled or scripted plugins.
    #[check]
    External(ExternalFilterConfig),
}

impl Default for GNSSSensorFilterConfig {
    fn default() -> Self {
        Self::Range(RangeFilterConfig::default())
    }
}

/// Runtime enum containing instantiated GNSS sensor filters.
#[derive(Debug, EnumToString)]
pub enum GNSSSensorFilterType {
    /// Instantiated range filter for GNSS sensor variables.
    Range(RangeFilter<GNSSSensorVariablesFilter>),
    /// Instantiated Python filter for GNSS sensor observations.
    Python(PythonFilter),
    /// Instantiated external filter for GNSS sensor observations.
    External(ExternalFilter),
}

impl GNSSSensorFilterType {
    /// Initializes filters that require runtime node context.
    pub fn post_init(
        &mut self,
        node: &mut crate::node::Node,
        initial_time: f32,
    ) -> crate::errors::SimbaResult<()> {
        match self {
            Self::Python(f) => f.post_init(node, initial_time),
            Self::External(f) => f.post_init(node, initial_time),
            Self::Range(_) => Ok(()),
        }
    }
}

/// Configuration of the [`GNSSSensor`].
///
/// Default values:
/// - `activation_time`: `Some(PeriodicityConfig { period: 1.0, offset: None, table: None })`
/// - `faults`: empty vector
/// - `filters`: empty vector
#[config_derives]
pub struct GNSSSensorConfig {
    /// Observation period of the sensor.
    #[check]
    pub activation_time: Option<PeriodicityConfig>,
    /// Fault on the x, y positions, and on the x and y velocities
    #[check]
    pub faults: Vec<GNSSSensorFaultModelConfig>,
    /// Filter configurations applied before fault injection.
    #[check]
    pub filters: Vec<GNSSSensorFilterConfig>,
}

impl Default for GNSSSensorConfig {
    fn default() -> Self {
        Self {
            activation_time: Some(PeriodicityConfig {
                period: crate::config::NumberConfig::Num(1.0),
                offset: None,
                table: None,
            }),
            faults: Vec::new(),
            filters: Vec::new(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for GNSSSensorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("GNSS sensor")
            .id_salt(format!("gnss-sensor-{}", unique_id))
            .show(ui, |ui| {
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
                        self.activation_time = Self::default().activation_time;
                    }
                });

                GNSSSensorFilterConfig::show_all_mut(
                    &mut self.filters,
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );

                GNSSSensorFaultModelConfig::show_all_mut(
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
        egui::CollapsingHeader::new("GNSS sensor")
            .id_salt(format!("gnss-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    if let Some(p) = &self.activation_time {
                        p.show(ui, ctx, unique_id);
                    } else {
                        ui.label("No activation");
                    }
                });

                GNSSSensorFilterConfig::show_all(&self.filters, ui, ctx, unique_id);

                GNSSSensorFaultModelConfig::show_all(&self.faults, ui, ctx, unique_id);
            });
    }
}

/// Record of the [`GNSSSensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct GNSSSensorRecord {
    last_time: Option<f32>,
}

#[cfg(feature = "gui")]
impl UIComponent for GNSSSensorRecord {
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

/// Observation of the pose of the node and its speed.
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct GNSSObservation {
    /// Global pose represented as `[x, y, theta]`.
    pub pose: Vector3<f32>,
    /// Global planar velocity represented as `[vx, vy]`.
    pub velocity: Vector2<f32>,
    /// Fault models applied to this observation.
    pub applied_faults: Vec<GNSSSensorFaultModelConfig>,
}

/// Serializable record representation of [`GNSSObservation`].
#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct GNSSObservationRecord {
    /// Recorded global pose represented as `[x, y, theta]`.
    pub pose: [f32; 3],
    /// Recorded global planar velocity represented as `[vx, vy]`.
    pub velocity: [f32; 2],
}

#[cfg(feature = "gui")]
impl UIComponent for GNSSObservationRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!(
                "Position: ({}, {}, {})",
                self.pose[0], self.pose[1], self.pose[2]
            ));
            ui.label(format!(
                "Velocity: ({}, {})",
                self.velocity[0], self.velocity[1]
            ));
        });
    }
}

impl Recordable<GNSSObservationRecord> for GNSSObservation {
    fn record(&self) -> GNSSObservationRecord {
        GNSSObservationRecord {
            pose: [self.pose.x, self.pose.y, self.pose.z],
            velocity: [self.velocity.x, self.velocity.y],
        }
    }
}

/// Sensor which observes the robot's pose in the global frame.
#[derive(Debug)]
pub struct GNSSSensor {
    /// Observation period
    activation_time: Option<Periodicity>,
    /// Last observation time.
    last_time: Option<f32>,
    /// Fault models for x and y positions and on x and y velocities
    faults: Vec<GNSSSensorFaultModelType>,
    filters: Vec<GNSSSensorFilterType>,
}

impl GNSSSensor {
    /// Makes a new [`GNSSSensor`] from the given config.
    pub fn from_config(
        config: &GNSSSensorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> SimbaResult<Self> {
        let mut fault_models = Vec::new();
        for fault_config in &config.faults {
            fault_models.push(match &fault_config {
                GNSSSensorFaultModelConfig::Additive(config) => GNSSSensorFaultModelType::Additive(
                    AdditiveFault::from_config(config, va_factory, initial_time),
                ),
                GNSSSensorFaultModelConfig::Clutter(config) => GNSSSensorFaultModelType::Clutter(
                    ClutterFault::from_config(config, va_factory, initial_time),
                ),
                GNSSSensorFaultModelConfig::Misdetection(config) => {
                    GNSSSensorFaultModelType::Misdetection(MisdetectionFault::from_config(
                        config,
                        va_factory,
                        initial_time,
                    ))
                }
                GNSSSensorFaultModelConfig::External(config) => {
                    GNSSSensorFaultModelType::External(ExternalFault::from_config(
                        config,
                        plugin_api,
                        global_config,
                        va_factory,
                        initial_time,
                    )?)
                }
                GNSSSensorFaultModelConfig::Python(config) => GNSSSensorFaultModelType::Python(
                    PythonFaultModel::from_config(config, global_config, initial_time)?,
                ),
            });
        }

        let mut filters = Vec::new();
        for filter_config in &config.filters {
            filters.push(match &filter_config {
                GNSSSensorFilterConfig::Range(config) => {
                    GNSSSensorFilterType::Range(RangeFilter::from_config(config, initial_time))
                }
                GNSSSensorFilterConfig::Python(config) => GNSSSensorFilterType::Python(
                    PythonFilter::from_config(config, global_config, initial_time)?,
                ),
                GNSSSensorFilterConfig::External(config) => {
                    GNSSSensorFilterType::External(ExternalFilter::from_config(
                        config,
                        plugin_api,
                        global_config,
                        va_factory,
                        initial_time,
                    )?)
                }
            });
        }

        let activation_time = config
            .activation_time
            .as_ref()
            .map(|p| Periodicity::from_config(p, va_factory, initial_time));
        Ok(Self {
            activation_time,
            last_time: None,
            faults: fault_models,
            filters,
        })
    }
}

use crate::node::Node;

impl Sensor for GNSSSensor {
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
        if let Some(last_time) = self.last_time
            && (time - last_time).abs() < TIME_ROUND
        {
            return Vec::new();
        }
        let arc_physic = node
            .physics()
            .expect("Node with GNSS sensor should have Physics");
        let physic = arc_physic.read().unwrap();
        let state = physic.state(time);

        let velocity_norm = state.velocity.fixed_rows::<2>(0).norm();
        let velocity = Vector2::<f32>::from_vec(vec![
            velocity_norm * state.pose.z.cos(),
            velocity_norm * state.pose.z.sin(),
        ]);

        // Apply filters until one rejects the observation
        let obs = SensorObservation::GNSS(GNSSObservation {
            pose: state.pose,
            velocity,
            applied_faults: Vec::new(),
        });

        let mut keep_observation = Some(obs);

        for filter in self.filters.iter() {
            if let Some(obs) = keep_observation {
                keep_observation = match filter {
                    GNSSSensorFilterType::Python(f) => f.filter(time, obs, &state, None),
                    GNSSSensorFilterType::External(f) => f.filter(time, obs, &state, None),
                    GNSSSensorFilterType::Range(f) => {
                        if let SensorObservation::GNSS(obs) = obs {
                            if f.match_exclusion(&GNSSSensorVariablesFilter::mapped_values(
                                |variant| match variant {
                                    GNSSSensorVariablesFilter::X => obs.pose.x,
                                    GNSSSensorVariablesFilter::Y => obs.pose.y,
                                    GNSSSensorVariablesFilter::Orientation => obs.pose.z,
                                    GNSSSensorVariablesFilter::R => {
                                        obs.pose.fixed_rows::<2>(0).norm()
                                    }
                                    GNSSSensorVariablesFilter::Theta => {
                                        obs.pose.y.atan2(obs.pose.x)
                                    }
                                    GNSSSensorVariablesFilter::VelocityX => obs.velocity.x,
                                    GNSSSensorVariablesFilter::VelocityY => obs.velocity.y,
                                    GNSSSensorVariablesFilter::SelfVelocity => {
                                        state.velocity.fixed_rows::<2>(0).norm()
                                    }
                                },
                            )) {
                                None
                            } else {
                                Some(SensorObservation::GNSS(obs))
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

        let mut observation_list = Vec::new();
        if let Some(observation) = keep_observation {
            observation_list.push(observation);
            for fault_model in self.faults.iter_mut() {
                match fault_model {
                    GNSSSensorFaultModelType::Python(f) => f.add_faults(
                        time,
                        time,
                        &mut observation_list,
                        SensorObservation::GNSS(GNSSObservation::default()),
                        node.environment(),
                    ),
                    GNSSSensorFaultModelType::External(f) => f.add_faults(
                        time,
                        time,
                        &mut observation_list,
                        SensorObservation::GNSS(GNSSObservation::default()),
                        node.environment(),
                    ),
                    GNSSSensorFaultModelType::Additive(f) => {
                        let obs_list_len = observation_list.len();
                        for (i, obs) in observation_list
                            .iter_mut()
                            .map(|o| {
                                if let SensorObservation::GNSS(observation) = o {
                                    observation
                                } else {
                                    unreachable!()
                                }
                            })
                            .enumerate()
                        {
                            let seed = time + i as f32 / (100. * obs_list_len as f32);
                            let new_values = f.add_faults(
                                seed,
                                GNSSSensorVariablesFaults::mapped_values(|variant| match variant {
                                    GNSSSensorVariablesFaults::X => obs.pose.x,
                                    GNSSSensorVariablesFaults::Y => obs.pose.y,
                                    GNSSSensorVariablesFaults::Orientation => obs.pose.z,
                                    GNSSSensorVariablesFaults::R => {
                                        obs.pose.fixed_rows::<2>(0).norm()
                                    }
                                    GNSSSensorVariablesFaults::Theta => {
                                        obs.pose.y.atan2(obs.pose.x)
                                    }
                                    GNSSSensorVariablesFaults::VelocityX => obs.velocity.x,
                                    GNSSSensorVariablesFaults::VelocityY => obs.velocity.y,
                                }),
                                &GNSSSensorVariables::mapped_values(|variant| match variant {
                                    GNSSSensorVariables::X => obs.pose.x,
                                    GNSSSensorVariables::Y => obs.pose.y,
                                    GNSSSensorVariables::Orientation => obs.pose.z,
                                    GNSSSensorVariables::R => obs.pose.fixed_rows::<2>(0).norm(),
                                    GNSSSensorVariables::Theta => obs.pose.y.atan2(obs.pose.x),
                                    GNSSSensorVariables::VelocityX => obs.velocity.x,
                                    GNSSSensorVariables::VelocityY => obs.velocity.y,
                                    GNSSSensorVariables::SelfVelocity => {
                                        state.velocity.fixed_rows::<2>(0).norm()
                                    }
                                }),
                            );

                            if let Some(value) = new_values.get(&GNSSSensorVariablesFaults::X) {
                                obs.pose.x = *value;
                            }
                            if let Some(value) = new_values.get(&GNSSSensorVariablesFaults::Y) {
                                obs.pose.y = *value;
                            }
                            if let Some(value) =
                                new_values.get(&GNSSSensorVariablesFaults::Orientation)
                            {
                                obs.pose.z = *value;
                            }
                            let new_r = if let Some(value) =
                                new_values.get(&GNSSSensorVariablesFaults::R)
                            {
                                *value
                            } else {
                                obs.pose.fixed_rows::<2>(0).norm()
                            };
                            let new_theta = if let Some(value) =
                                new_values.get(&GNSSSensorVariablesFaults::Theta)
                            {
                                *value
                            } else {
                                obs.pose.y.atan2(obs.pose.x)
                            };
                            obs.pose.x = new_r * new_theta.cos();
                            obs.pose.y = new_r * new_theta.sin();
                            if let Some(value) =
                                new_values.get(&GNSSSensorVariablesFaults::VelocityX)
                            {
                                obs.velocity.x = *value;
                            }
                            if let Some(value) =
                                new_values.get(&GNSSSensorVariablesFaults::VelocityY)
                            {
                                obs.velocity.y = *value;
                            }
                            obs.applied_faults
                                .push(GNSSSensorFaultModelConfig::Additive(f.config().clone()));
                        }
                    }
                    GNSSSensorFaultModelType::Clutter(f) => {
                        let new_obs_from_clutter = f.add_faults(time, 1. / 100.);
                        for (_, obs_params) in new_obs_from_clutter {
                            let mut x = obs_params
                                .get(&GNSSSensorVariablesFaults::X)
                                .cloned()
                                .unwrap_or(0.);
                            let mut y = obs_params
                                .get(&GNSSSensorVariablesFaults::Y)
                                .cloned()
                                .unwrap_or(0.);
                            let orientation = obs_params
                                .get(&GNSSSensorVariablesFaults::Orientation)
                                .cloned()
                                .unwrap_or(0.);

                            let r = obs_params
                                .get(&GNSSSensorVariablesFaults::R)
                                .cloned()
                                .unwrap_or(0.);
                            let theta = obs_params
                                .get(&GNSSSensorVariablesFaults::Theta)
                                .cloned()
                                .unwrap_or(0.);
                            x += r * theta.cos();
                            y += r * theta.sin();

                            let v_x = obs_params
                                .get(&GNSSSensorVariablesFaults::VelocityX)
                                .cloned()
                                .unwrap_or(0.);
                            let v_y = obs_params
                                .get(&GNSSSensorVariablesFaults::VelocityY)
                                .cloned()
                                .unwrap_or(0.);

                            let obs = SensorObservation::GNSS(GNSSObservation {
                                pose: Vector3::new(x, y, orientation),
                                velocity: Vector2::new(v_x, v_y),
                                applied_faults: vec![GNSSSensorFaultModelConfig::Clutter(
                                    f.config().clone(),
                                )],
                            });
                            observation_list.push(obs);
                        }
                    }
                    GNSSSensorFaultModelType::Misdetection(f) => {
                        observation_list = observation_list
                            .iter()
                            .enumerate()
                            .filter_map(|(i, obs)| {
                                if let SensorObservation::GNSS(observation) = obs {
                                    if f.detected(time + (i as f32) / 1000.) {
                                        Some(SensorObservation::GNSS(observation.clone()))
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
            debug!("GNSS Observation was filtered out");
        }

        if let Some(p) = self.activation_time.as_mut() {
            p.update(time);
        }
        self.last_time = Some(time);
        observation_list
    }

    fn next_time_step(&self) -> f32 {
        if let Some(activation) = &self.activation_time {
            activation.next_time()
        } else {
            f32::INFINITY
        }
    }
}

impl Recordable<SensorRecord> for GNSSSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::GNSSSensor(GNSSSensorRecord {
            last_time: self.last_time,
        })
    }
}
