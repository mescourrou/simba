/*!
Provides a [`Sensor`] which can provide linear velocity and angular velocity.
*/

use std::sync::{Arc, Mutex};

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
use crate::sensors::sensor_filters::{
    SensorFilter, SensorFilterConfig, SensorFilterType, make_sensor_filter_from_config,
};
use crate::simulator::SimulatorConfig;
use crate::state_estimators::{State, StateRecord};
use crate::utils::SharedMutex;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::enum_tools::EnumVariables;
use crate::utils::periodicity::{Periodicity, PeriodicityConfig};
use log::debug;
use serde_derive::{Deserialize, Serialize};
use simba_macros::{EnumToString, UIComponent, config_derives, enum_variables};

extern crate nalgebra as na;

enum_variables!(
    SpeedSensorVariables;
    Faults: W, "w", "angular_velocity", "angular";
    Faults: V, "v", "linear_velocity", "linear";
    SelfVelocity, "self_velocity";
);

#[config_derives]
#[derive(UIComponent)]
#[show_all = "Faults"]
pub enum SpeedSensorFaultModelConfig {
    Additive(AdditiveFaultConfig<SpeedSensorVariablesFaults, SpeedSensorVariables>),
    Python(PythonFaultModelConfig),
    External(ExternalFaultConfig),
}

impl Default for SpeedSensorFaultModelConfig {
    fn default() -> Self {
        Self::Additive(AdditiveFaultConfig::default())
    }
}

#[derive(Debug, EnumToString)]
pub enum SpeedSensorFaultModelType {
    Additive(AdditiveFault<SpeedSensorVariablesFaults, SpeedSensorVariables>),
    Python(PythonFaultModel),
    External(ExternalFault),
}

/// Configuration of the [`SpeedSensor`].
#[config_derives]
pub struct SpeedSensorConfig {
    /// Observation period of the sensor.
    #[check]
    pub activation_time: Option<PeriodicityConfig>,
    #[check]
    pub faults: Vec<SpeedSensorFaultModelConfig>,
    #[check]
    pub filters: Vec<SensorFilterConfig<SpeedSensorVariables>>,
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

                SensorFilterConfig::show_filters_mut(
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
                SensorFilterConfig::show_filters(&self.filters, ui, ctx, unique_id);

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
    pub linear_velocity: f32,
    pub lateral_velocity: f32,
    pub angular_velocity: f32,
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

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct SpeedObservationRecord {
    pub linear_velocity: f32,
    pub lateral_velocity: f32,
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
    filters: Vec<SensorFilterType<SpeedSensorVariables>>,
}

impl SpeedSensor {
    /// Makes a new [`SpeedSensor`] from the given config.
    pub fn from_config(
        config: &SpeedSensorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        robot_name: &str,
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
            filters.push(make_sensor_filter_from_config(
                filter_config,
                plugin_api,
                global_config,
                va_factory,
                initial_time,
            )?);
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
            match fault_model {
                SpeedSensorFaultModelType::Python(f) => f.post_init(node, initial_time)?,
                SpeedSensorFaultModelType::External(f) => f.post_init(node, initial_time)?,
                SpeedSensorFaultModelType::Additive(_) => (),
            }
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
                    SensorFilterType::PythonFilter(f) => f.filter(time, obs, &state, None),
                    SensorFilterType::External(f) => f.filter(time, obs, &state, None),
                    SensorFilterType::RangeFilter(f) => {
                        if let SensorObservation::Speed(obs) = obs {
                            if f.match_exclusion(&SpeedSensorVariables::mapped_values(|variant| {
                                match variant {
                                    SpeedSensorVariables::W => obs.angular_velocity,
                                    SpeedSensorVariables::V => obs.linear_velocity,
                                    SpeedSensorVariables::SelfVelocity => {
                                        state.velocity.fixed_rows::<2>(0).norm()
                                    }
                                }
                            })) {
                                None
                            } else {
                                Some(SensorObservation::Speed(obs))
                            }
                        } else {
                            unreachable!()
                        }
                    }
                    _ => unimplemented!(
                        "{} filter not implemented for SpeedSensor",
                        filter.to_string()
                    ),
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
