/*!
Provides a [`Sensor`] which can provide position and velocity in the global frame.
*/

use std::sync::{Arc, Mutex};

use super::fault_models::fault_model::{
    FaultModel, FaultModelConfig, make_fault_model_from_config,
};
use super::{Sensor, SensorObservation, SensorRecord};

use crate::constants::TIME_ROUND;
use crate::enum_variables;
use crate::errors::SimbaResult;
#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::logger::is_enabled;
use crate::plugin_api::PluginAPI;
use crate::recordable::Recordable;
use crate::sensors::sensor_filters::{
    SensorFilter, SensorFilterConfig, SensorFilterType, make_sensor_filter_from_config,
};
use crate::simulator::SimulatorConfig;
use crate::utils::SharedMutex;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::enum_tools::EnumVariables;
use crate::utils::periodicity::{Periodicity, PeriodicityConfig};
use log::debug;
use nalgebra::{Vector2, Vector3};
use serde_derive::{Deserialize, Serialize};
use simba_macros::config_derives;

extern crate nalgebra as na;

enum_variables!(
    GNSSSensorVariables;
    X, "x", "position_x";
    Y, "y", "position_y";
    Orientation, "orientation", "z";
    VelocityX, "velocity_x", "vx";
    VelocityY, "velocity_y", "vy";
    SelfVelocity, "self_velocity";
);

/// Configuration of the [`GNSSSensor`].
#[config_derives]
pub struct GNSSSensorConfig {
    /// Observation period of the sensor.
    #[check]
    pub activation_time: Option<PeriodicityConfig>,
    /// Fault on the x, y positions, and on the x and y velocities
    #[check]
    pub faults: Vec<FaultModelConfig>,
    #[check]
    pub filters: Vec<SensorFilterConfig<GNSSSensorVariables>>,
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

                SensorFilterConfig::show_filters(&self.filters, ui, ctx, unique_id);

                FaultModelConfig::show_faults(&self.faults, ui, ctx, unique_id);
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
    pub pose: Vector3<f32>,
    pub velocity: Vector2<f32>,
    pub applied_faults: Vec<FaultModelConfig>,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct GNSSObservationRecord {
    pub pose: [f32; 3],
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
    faults: SharedMutex<Vec<Box<dyn FaultModel>>>,
    filters: Vec<SensorFilterType<GNSSSensorVariables>>,
}

impl GNSSSensor {
    /// Makes a new [`GNSSSensor`] from the given config.
    pub fn from_config(
        config: &GNSSSensorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        robot_name: &str,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
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
            match filter {
                SensorFilterType::PythonFilter(f) => f.post_init(node, initial_time)?,
                SensorFilterType::External(f) => f.post_init(node, initial_time)?,
                _ => {}
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
                    SensorFilterType::PythonFilter(f) => f.filter(time, obs, &state, None),
                    SensorFilterType::External(f) => f.filter(time, obs, &state, None),
                    SensorFilterType::RangeFilter(f) => {
                        if let SensorObservation::GNSS(obs) = obs {
                            if f.match_exclusion(&GNSSSensorVariables::mapped_values(|variant| {
                                match variant {
                                    GNSSSensorVariables::X => obs.pose.x,
                                    GNSSSensorVariables::Y => obs.pose.y,
                                    GNSSSensorVariables::Orientation => obs.pose.z,
                                    GNSSSensorVariables::VelocityX => obs.velocity.x,
                                    GNSSSensorVariables::VelocityY => obs.velocity.y,
                                    GNSSSensorVariables::SelfVelocity => {
                                        state.velocity.fixed_rows::<2>(0).norm()
                                    }
                                }
                            })) {
                                None
                            } else {
                                Some(SensorObservation::GNSS(obs))
                            }
                        } else {
                            unreachable!()
                        }
                    }
                    _ => unimplemented!(
                        "{} filter not implemented for GNSSSensor",
                        filter.to_string()
                    ),
                };
            } else {
                break;
            }
        }

        if let Some(observation) = keep_observation {
            observation_list.push(observation);
            for fault_model in self.faults.lock().unwrap().iter_mut() {
                fault_model.add_faults(
                    time,
                    time,
                    &mut observation_list,
                    SensorObservation::GNSS(GNSSObservation::default()),
                    node.environment(),
                );
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
