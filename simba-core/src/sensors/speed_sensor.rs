//! Speed sensor implementation.
//!
//! This module provides a [`Sensor`] that reports linear and angular
//! robot velocities.
//! It supports periodic activation, configurable filter chains through
//! [`SpeedSensorFilterConfig`], and optional fault injection through [`SpeedSensorFaultModelConfig`].

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
use crate::sensors::fault_models::external_fault::{ExternalFault, ExternalFaultConfig};
use crate::sensors::fault_models::python_fault_model::{PythonFaultModel, PythonFaultModelConfig};
use crate::sensors::sensor_filters::SensorFilter;
use crate::sensors::sensor_filters::external_filter::{ExternalFilter, ExternalFilterConfig};
use crate::sensors::sensor_filters::python_filter::{PythonFilter, PythonFilterConfig};
use crate::sensors::sensor_filters::range_filter::{RangeFilter, RangeFilterConfig};
use crate::simulator::SimulatorConfig;
use crate::state_estimators::{State, StateRecord};
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::enum_tools::EnumVariables;
use crate::utils::periodicity::{Periodicity, PeriodicityConfig};
use log::debug;
use serde_derive::{Deserialize, Serialize};
use simba_macros::{EnumToString, UIComponent, config_derives, enum_variables};

extern crate nalgebra as na;

enum_variables!(
    "Variables used by speed sensors and related fault models."
    SpeedSensorVariables;
    "Variables that can be used by filters."
    Filter,
    "Variables that additive speed faults can modify."
    Faults:
    "Linear velocity component."
    V, "v", "linear_velocity", "linear";
    Filter, Faults:
    "Angular velocity component."
    W, "w", "angular_velocity", "angular";
    Filter:
    "Norm of the linear robot velocity."
    SelfVelocity, "self_velocity";
);

/// Configuration enum selecting fault models applied to speed observations.
///
/// Default value: [`SpeedSensorFaultModelConfig::Additive`] with
/// [`AdditiveFaultConfig::default`].
#[config_derives]
#[derive(UIComponent)]
#[show_all = "Faults"]
pub enum SpeedSensorFaultModelConfig {
    /// Additive perturbation fault model.
    Additive(AdditiveFaultConfig<SpeedSensorVariablesFaults, SpeedSensorVariables>),
    /// Python-implemented custom fault model.
    Python(PythonFaultModelConfig),
    /// Plugin-provided external fault model.
    External(ExternalFaultConfig),
}

impl Default for SpeedSensorFaultModelConfig {
    fn default() -> Self {
        Self::Additive(AdditiveFaultConfig::default())
    }
}

/// Enum of instantiated fault models applied to speed observations.
///
/// The variants correspond to the ones of [`SpeedSensorFaultModelConfig`] but contain instantiated fault models instead of their config.
#[derive(Debug, EnumToString)]
pub enum SpeedSensorFaultModelType {
    /// Instantiated additive fault model.
    Additive(AdditiveFault<SpeedSensorVariablesFaults, SpeedSensorVariables>),
    /// Instantiated Python fault model.
    Python(PythonFaultModel),
    /// Instantiated external fault model.
    External(ExternalFault),
}

impl SpeedSensorFaultModelType {
    /// Wraps the post-initialization of fault models that require runtime node context.
    pub fn post_init(&mut self, node: &mut Node, initial_time: f32) -> SimbaResult<()> {
        match self {
            Self::Additive(_) => Ok(()),
            Self::Python(f) => f.post_init(node, initial_time),
            Self::External(f) => f.post_init(node, initial_time),
        }
    }
}

/// Configuration enum selecting among multiple sensor observation filtering strategies for speed sensors.
///
/// This enum provides a unified interface for declaring sensor filters with different decision logic.
/// Each variant wraps the configuration for a specific filter type.
/// When multiple filters are applied to a sensor, all must agree to keep the observation.
///
/// Default value: [`SpeedSensorFilterConfig::Range`] with [`RangeFilterConfig::default`].
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
pub enum SpeedSensorFilterConfig {
    /// Range-based filtering on enumerated variables: excludes observations where numeric values fall outside specified bounds.
    #[check]
    Range(RangeFilterConfig<SpeedSensorVariablesFilter>),
    /// String pattern filtering on observed object unique id (name for nodes, id for landmarks): excludes observations matching configured regexp patterns.
    #[check]
    Python(PythonFilterConfig),
    /// Plugin-based custom filtering: delegates exclusion logic to external compiled or scripted plugins.
    #[check]
    External(ExternalFilterConfig),
}

impl Default for SpeedSensorFilterConfig {
    fn default() -> Self {
        Self::Range(RangeFilterConfig::default())
    }
}

/// Runtime enum containing instantiated Speed sensor filters.
#[derive(Debug, EnumToString)]
pub enum SpeedSensorFilterType {
    /// Instantiated range filter for Speed sensor variables.
    Range(RangeFilter<SpeedSensorVariablesFilter>),
    /// Instantiated Python filter for Speed sensor observations.
    Python(PythonFilter),
    /// Instantiated external filter for Speed sensor observations.
    External(ExternalFilter),
}

impl SpeedSensorFilterType {
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

/// Configuration of the [`SpeedSensor`].
///
/// The [`SpeedSensor`] observes the robot's linear and angular velocity, which can be filtered and perturbed by faults.
///
/// Default values:
/// - `activation_time`: `Some(PeriodicityConfig { period: 0.1, offset: None, table: None })`
/// - `faults`: empty vector
/// - `filters`: empty vector
#[config_derives]
pub struct SpeedSensorConfig {
    /// Periodicity of the sensor.
    #[check]
    pub activation_time: Option<PeriodicityConfig>,
    /// Fault models applied after filtering.
    #[check]
    pub faults: Vec<SpeedSensorFaultModelConfig>,
    /// Filter chain applied before fault injection.
    #[check]
    pub filters: Vec<SpeedSensorFilterConfig>,
}

impl Default for SpeedSensorConfig {
    fn default() -> Self {
        Self {
            activation_time: Some(PeriodicityConfig {
                period: crate::config::NumberConfig::Num(0.1),
                offset: None,
                table: None,
            }),
            faults: Vec::new(),
            filters: Vec::new(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for SpeedSensorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Speed sensor")
            .id_salt(format!("speed-sensor-{}", unique_id))
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

                SpeedSensorFilterConfig::show_all_mut(
                    &mut self.filters,
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );

                SpeedSensorFaultModelConfig::show_all_mut(
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
        egui::CollapsingHeader::new("Speed sensor")
            .id_salt(format!("speed-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    if let Some(p) = &self.activation_time {
                        p.show(ui, ctx, unique_id);
                    } else {
                        ui.label("No activation");
                    }
                });
                SpeedSensorFilterConfig::show_all(&self.filters, ui, ctx, unique_id);

                SpeedSensorFaultModelConfig::show_all(&self.faults, ui, ctx, unique_id);
            });
    }
}

/// Record of the [`SpeedSensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct SpeedSensorRecord {
    last_time: Option<f32>,
    last_state: StateRecord,
}

#[cfg(feature = "gui")]
impl UIComponent for SpeedSensorRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.label(format!(
            "Last time: {}",
            match self.last_time {
                Some(t) => t.to_string(),
                None => "None".to_string(),
            }
        ));
        ui.label("Last state: ");
        self.last_state.show(ui, ctx, unique_id);
    }
}

/// Observation of the speed.
#[derive(Serialize, Deserialize, Debug, Default, Clone)]
pub struct SpeedObservation {
    /// Forward linear velocity component.
    pub linear_velocity: f32,
    /// Lateral linear velocity component.
    pub lateral_velocity: f32,
    /// Angular velocity component.
    pub angular_velocity: f32,
    /// Fault models that were applied to produce this observation.
    pub applied_faults: Vec<SpeedSensorFaultModelConfig>,
}

impl Recordable<SpeedObservationRecord> for SpeedObservation {
    fn record(&self) -> SpeedObservationRecord {
        SpeedObservationRecord {
            linear_velocity: self.linear_velocity,
            lateral_velocity: self.lateral_velocity,
            angular_velocity: self.angular_velocity,
        }
    }
}

/// Serializable record for a [`SpeedObservation`].
#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct SpeedObservationRecord {
    /// Recorded forward linear velocity component.
    pub linear_velocity: f32,
    /// Recorded lateral linear velocity component.
    pub lateral_velocity: f32,
    /// Recorded angular velocity component.
    pub angular_velocity: f32,
}

#[cfg(feature = "gui")]
impl UIComponent for SpeedObservationRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!("Linear velocity: {}", self.linear_velocity));
            ui.label(format!("Lateral velocity: {}", self.lateral_velocity));
            ui.label(format!("Angular velocity: {}", self.angular_velocity));
        });
    }
}

/// Sensor which observes the robot's speed
#[derive(Debug)]
pub struct SpeedSensor {
    /// Last state to compute the velocity.
    last_state: State,
    /// Observation period
    activation_time: Option<Periodicity>,
    /// Last observation time.
    last_time: Option<f32>,
    faults: Vec<SpeedSensorFaultModelType>,
    filters: Vec<SpeedSensorFilterType>,
}

impl SpeedSensor {
    /// Makes a new [`SpeedSensor`] from the given config.
    pub fn from_config(
        config: &SpeedSensorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> SimbaResult<Self> {
        let mut fault_models = Vec::new();
        for fault_config in &config.faults {
            fault_models.push(match &fault_config {
                SpeedSensorFaultModelConfig::Additive(c) => SpeedSensorFaultModelType::Additive(
                    AdditiveFault::from_config(c, va_factory, initial_time),
                ),
                SpeedSensorFaultModelConfig::Python(c) => SpeedSensorFaultModelType::Python(
                    PythonFaultModel::from_config(c, global_config, initial_time)?,
                ),
                SpeedSensorFaultModelConfig::External(c) => {
                    SpeedSensorFaultModelType::External(ExternalFault::from_config(
                        c,
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
                SpeedSensorFilterConfig::Range(c) => {
                    SpeedSensorFilterType::Range(RangeFilter::from_config(c, initial_time))
                }
                SpeedSensorFilterConfig::Python(c) => SpeedSensorFilterType::Python(
                    PythonFilter::from_config(c, global_config, initial_time)?,
                ),
                SpeedSensorFilterConfig::External(c) => {
                    SpeedSensorFilterType::External(ExternalFilter::from_config(
                        c,
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
            last_state: State::new(),
            activation_time: period,
            last_time: None,
            faults: fault_models,
            filters,
        })
    }
}

use crate::node::Node;

impl Sensor for SpeedSensor {
    fn post_init(&mut self, node: &mut Node, initial_time: f32) -> SimbaResult<()> {
        self.last_state = node
            .physics()
            .expect("Node with Speed sensor should have Physics")
            .read()
            .unwrap()
            .state(initial_time)
            .clone();
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
            .expect("Node with Speed sensor should have Physics");
        let physic = arc_physic.read().unwrap();
        let state = physic.state(time);

        let obs = SensorObservation::Speed(SpeedObservation {
            linear_velocity: state.velocity.x,
            lateral_velocity: state.velocity.y,
            angular_velocity: state.velocity.z,
            applied_faults: Vec::new(),
        });

        let mut keep_observation = Some(obs);

        for filter in self.filters.iter() {
            if let Some(obs) = keep_observation {
                keep_observation = match filter {
                    SpeedSensorFilterType::Python(f) => f.filter(time, obs, &state, None),
                    SpeedSensorFilterType::External(f) => f.filter(time, obs, &state, None),
                    SpeedSensorFilterType::Range(f) => {
                        if let SensorObservation::Speed(obs) = obs {
                            if f.match_exclusion(&SpeedSensorVariablesFilter::mapped_values(
                                |variant| match variant {
                                    SpeedSensorVariablesFilter::W => obs.angular_velocity,
                                    SpeedSensorVariablesFilter::V => obs.linear_velocity,
                                    SpeedSensorVariablesFilter::SelfVelocity => {
                                        state.velocity.fixed_rows::<2>(0).norm()
                                    }
                                },
                            )) {
                                None
                            } else {
                                Some(SensorObservation::Speed(obs))
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

        let mut observation_list = Vec::<SensorObservation>::new();
        if let Some(obs) = keep_observation {
            observation_list.push(obs);
            for fault_model in self.faults.iter_mut() {
                match fault_model {
                    SpeedSensorFaultModelType::Python(f) => f.add_faults(
                        time,
                        time,
                        &mut observation_list,
                        SensorObservation::Speed(SpeedObservation::default()),
                        node.environment(),
                    ),
                    SpeedSensorFaultModelType::External(f) => f.add_faults(
                        time,
                        time,
                        &mut observation_list,
                        SensorObservation::Speed(SpeedObservation::default()),
                        node.environment(),
                    ),
                    SpeedSensorFaultModelType::Additive(f) => {
                        let obs_list_len = observation_list.len();
                        for (i, obs) in observation_list
                            .iter_mut()
                            .map(|o| {
                                if let SensorObservation::Speed(observation) = o {
                                    observation
                                } else {
                                    unreachable!()
                                }
                            })
                            .enumerate()
                        {
                            let seed = time + i as f32 / (100. * obs_list_len as f32);
                            f.add_faults(
                                seed,
                                SpeedSensorVariablesFaults::mapped_values(
                                    |variant| match variant {
                                        SpeedSensorVariablesFaults::W => obs.angular_velocity,
                                        SpeedSensorVariablesFaults::V => obs.linear_velocity,
                                    },
                                ),
                                &SpeedSensorVariables::mapped_values(|variant| match variant {
                                    SpeedSensorVariables::W => obs.angular_velocity,
                                    SpeedSensorVariables::V => obs.linear_velocity,
                                    SpeedSensorVariables::SelfVelocity => {
                                        state.velocity.fixed_rows::<2>(0).norm()
                                    }
                                }),
                            )
                            .into_iter()
                            .for_each(|(variant, value)| match variant {
                                SpeedSensorVariablesFaults::W => obs.angular_velocity += value,
                                SpeedSensorVariablesFaults::V => obs.linear_velocity += value,
                            });
                        }
                    }
                }
            }
        } else if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
            debug!("Speed observation was filtered out");
        }

        if let Some(p) = self.activation_time.as_mut() {
            p.update(time);
        }
        self.last_time = Some(time);
        self.last_state = state.clone();
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

impl Recordable<SensorRecord> for SpeedSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::SpeedSensor(SpeedSensorRecord {
            last_time: self.last_time,
            last_state: self.last_state.record(),
        })
    }
}
