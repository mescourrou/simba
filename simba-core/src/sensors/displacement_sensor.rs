/*!
Provides a [`Sensor`] which can provide the transformation since the last observation.
*/

use std::collections::HashMap;
use std::fmt::Display;
use std::str::FromStr;
use std::sync::{Arc, Mutex};

use super::fault_models::fault_model::{
    FaultModel, FaultModelConfig, make_fault_model_from_config,
};
use super::{Sensor, SensorObservation, SensorRecord};

use crate::config::NumberConfig;
use crate::constants::TIME_ROUND;

use crate::enum_variables;
use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::logger::is_enabled;
use crate::plugin_api::PluginAPI;
use crate::recordable::Recordable;
use crate::sensors::sensor_filters::{
    SensorFilter, SensorFilterConfig, SensorFilterType, make_sensor_filter_from_config,
};
use crate::simulator::SimulatorConfig;
use crate::state_estimators::{State, StateRecord};
use crate::utils::SharedMutex;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::enum_tools::EnumVariables;
use crate::utils::geometry::smallest_theta_diff;
use crate::utils::periodicity::{Periodicity, PeriodicityConfig};
use log::debug;
use nalgebra::{Matrix3, Vector2};
use serde_derive::{Deserialize, Serialize};
use simba_macros::{EnumToString, ToVec, config_derives};

extern crate nalgebra as na;

enum_variables!(
    DisplacementSensorVariables;
    X, "x", "dx";
    Y, "y", "dy";
    Rotation, "rotation", "r";
    Translation, "translation", "t";
    SelfRotation, "self_rotation"
);

/// Configuration of the [`DisplacementSensor`].
#[config_derives]
pub struct DisplacementSensorConfig {
    /// Observation period of the sensor.
    #[check]
    pub activation_time: Option<PeriodicityConfig>,
    #[check]
    pub faults: Vec<FaultModelConfig>,
    #[check]
    pub filters: Vec<SensorFilterConfig<DisplacementSensorVariables>>,
    pub lie_movement: bool,
}

impl Default for DisplacementSensorConfig {
    fn default() -> Self {
        Self {
            activation_time: Some(PeriodicityConfig {
                period: NumberConfig::Num(0.1),
                ..Default::default()
            }),
            faults: Vec::new(),
            filters: Vec::new(),
            lie_movement: false,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for DisplacementSensorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Displacement sensor")
            .id_salt(format!("displacement-sensor-{}", unique_id))
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
                        self.activation_time = Some(PeriodicityConfig::default());
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("Lie movement:");
                    ui.checkbox(&mut self.lie_movement, "");
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

                FaultModelConfig::show_faults_mut(
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
        egui::CollapsingHeader::new("Displacement sensor")
            .id_salt(format!("displacement-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    if let Some(p) = &self.activation_time {
                        p.show(ui, ctx, unique_id);
                    } else {
                        ui.label("No activation");
                    }
                });

                ui.label(format!("Lie movement: {}", self.lie_movement));

                SensorFilterConfig::show_filters(&self.filters, ui, ctx, unique_id);

                FaultModelConfig::show_faults(&self.faults, ui, ctx, unique_id);
            });
    }
}

/// Record of the [`DisplacementSensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct DisplacementSensorRecord {
    last_time: Option<f32>,
    last_state: StateRecord,
    lie_movement: bool, // Default is false
}

#[cfg(feature = "gui")]
impl UIComponent for DisplacementSensorRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.label(format!(
            "Last time: {}",
            match self.last_time {
                Some(t) => t.to_string(),
                None => "None".to_string(),
            }
        ));
        ui.label(format!("Lie movement: {}", self.lie_movement));
        ui.label("Last state: ");
        self.last_state.show(ui, ctx, unique_id);
    }
}

/// Observation of the displacement.
#[derive(Serialize, Deserialize, Debug, Default, Clone)]
pub struct DisplacementObservation {
    pub translation: Vector2<f32>,
    pub rotation: f32,
    pub applied_faults: Vec<FaultModelConfig>,
}

impl Recordable<DisplacementObservationRecord> for DisplacementObservation {
    fn record(&self) -> DisplacementObservationRecord {
        DisplacementObservationRecord {
            translation: self.translation,
            rotation: self.rotation,
            applied_faults: self.applied_faults.clone(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct DisplacementObservationRecord {
    pub translation: Vector2<f32>,
    pub rotation: f32,
    pub applied_faults: Vec<FaultModelConfig>,
}

#[cfg(feature = "gui")]
impl UIComponent for DisplacementObservationRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!("Translation: {:?}", self.translation));
            ui.label(format!("Rotation: {}", self.rotation));
        });
    }
}

/// Sensor which observes the robot displacement since last observation.
#[derive(Debug)]
pub struct DisplacementSensor {
    /// Last state to compute the displacement.
    last_state: State,
    /// Observation period
    activation_time: Option<Periodicity>,
    /// Last observation time.
    last_time: Option<f32>,
    faults: SharedMutex<Vec<Box<dyn FaultModel>>>,
    filters: Vec<SensorFilterType<DisplacementSensorVariables>>,
    lie_movement: bool,
}

impl DisplacementSensor {
    /// Makes a new [`DisplacementSensor`] from the given config.
    pub fn from_config(
        config: &DisplacementSensorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        robot_name: &str,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
        initial_state: &State,
    ) -> SimbaResult<Self> {
        let fault_models = Arc::new(Mutex::new(Vec::new()));
        let mut unlock_fault_model = fault_models.lock().unwrap();
        for fault_config in &config.faults {
            unlock_fault_model.push(make_fault_model_from_config(
                fault_config,
                global_config,
                robot_name,
                va_factory,
                initial_time,
            ));
        }
        drop(unlock_fault_model);

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

        let activation_time = config
            .activation_time
            .as_ref()
            .map(|p| Periodicity::from_config(p, va_factory, initial_time));
        Ok(Self {
            last_state: initial_state.clone(),
            activation_time,
            last_time: None,
            faults: fault_models,
            filters,
            lie_movement: config.lie_movement,
        })
    }
}

use crate::node::Node;

impl Sensor for DisplacementSensor {
    fn post_init(&mut self, node: &mut Node, initial_time: f32) -> SimbaResult<()> {
        self.last_state = node
            .physics()
            .expect("Node with Speed sensor should have Physics")
            .read()
            .unwrap()
            .state(initial_time)
            .clone();
        for filter in self.filters.iter_mut() {
            match filter {
                SensorFilterType::External(f) => f.post_init(node, initial_time)?,
                SensorFilterType::PythonFilter(f) => f.post_init(node, initial_time)?,
                _ => (),
            }
        }
        for fault_model in self.faults.lock().unwrap().iter_mut() {
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
        let arc_physic = node
            .physics()
            .expect("Node with Displacement sensor should have Physics");
        let physic = arc_physic.read().unwrap();
        let state = physic.state(time);

        let (tx, ty, r) = if self.lie_movement {
            todo!("Lie movement not implemented yet for DisplacementSensor");
        } else {
            let dx = state.pose.x - self.last_state.pose.x;
            let dy = state.pose.y - self.last_state.pose.y;
            let dtheta = smallest_theta_diff(state.pose.z, self.last_state.pose.z);

            let rotation_matrix = Matrix3::new(
                state.pose.z.cos(),
                state.pose.z.sin(),
                0.,
                -state.pose.z.sin(),
                state.pose.z.cos(),
                0.,
                0.,
                0.,
                1.,
            );

            let local_displacement = rotation_matrix
                .try_inverse()
                .expect("Rotation matrix should be invertible")
                .transform_vector(&Vector2::new(dx, dy));

            (local_displacement.x, local_displacement.y, dtheta)
        };

        let obs = SensorObservation::Displacement(DisplacementObservation {
            translation: Vector2::new(tx, ty),
            rotation: r,
            applied_faults: Vec::new(),
        });

        let mut keep_observation = Some(obs);

        for filter in self.filters.iter() {
            if let Some(o) = keep_observation {
                keep_observation = match filter {
                    SensorFilterType::External(f) => f.filter(time, o, &state, None),
                    SensorFilterType::PythonFilter(f) => f.filter(time, o, &state, None),
                    SensorFilterType::RangeFilter(f) => {
                        if let SensorObservation::Displacement(obs) = o {
                            if f.match_exclusion(&DisplacementSensorVariables::mapped_values(
                                |variant| match variant {
                                    DisplacementSensorVariables::X => obs.translation.x,
                                    DisplacementSensorVariables::Y => obs.translation.y,
                                    DisplacementSensorVariables::Rotation => obs.rotation,
                                    DisplacementSensorVariables::Translation => {
                                        obs.translation.iter().map(|v| v * v).sum::<f32>().sqrt()
                                    }
                                    DisplacementSensorVariables::SelfRotation => {
                                        state.velocity.fixed_rows::<2>(0).norm()
                                    }
                                },
                            )) {
                                None
                            } else {
                                Some(SensorObservation::Displacement(obs))
                            }
                        } else {
                            unreachable!()
                        }
                    }
                    _ => unimplemented!(
                        "{} filter not implemented for DisplacementSensor",
                        filter.to_string()
                    ),
                };
            } else {
                break;
            }
        }

        if let Some(obs) = keep_observation {
            observation_list.push(obs);
            for fault_model in self.faults.lock().unwrap().iter_mut() {
                fault_model.add_faults(
                    time,
                    time,
                    &mut observation_list,
                    SensorObservation::Displacement(DisplacementObservation::default()),
                    node.environment(),
                );
            }
        } else if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
            debug!("Displacement observation was filtered out");
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

impl Recordable<SensorRecord> for DisplacementSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::DisplacementSensor(DisplacementSensorRecord {
            last_time: self.last_time,
            last_state: self.last_state.record(),
            lie_movement: self.lie_movement,
        })
    }
}
