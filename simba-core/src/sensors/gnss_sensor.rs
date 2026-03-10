/*!
Provides a [`Sensor`] which can provide position and velocity in the global frame.
*/

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
use crate::sensors::sensor_filters::{
    SensorFilter, SensorFilterConfig, SensorFilterType, make_sensor_filter_from_config,
};
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
    GNSSSensorVariables;
    Filter, Faults: X, "x", "position_x";
    Filter, Faults: Y, "y", "position_y";
    Filter, Faults: Orientation, "orientation", "z";
    Filter, Faults: R, "r", "distance";
    Filter, Faults: Theta, "theta", "angle";
    Filter, Faults: VelocityX, "velocity_x", "vx";
    Filter, Faults: VelocityY, "velocity_y", "vy";
    Filter: SelfVelocity, "self_velocity";
);

#[config_derives]
#[derive(UIComponent)]
#[show_all = "Faults"]
pub enum GNSSSensorFaultModelConfig {
    AdditiveRobotCentered(AdditiveFaultConfig<GNSSSensorVariablesFaults, GNSSSensorVariables>),
    AdditiveObservationCentered(
        AdditiveFaultConfig<GNSSSensorVariablesFaults, GNSSSensorVariables>,
    ),
    Clutter(ClutterFaultConfig<GNSSSensorVariablesFaults>),
    Misdetection(MisdetectionFaultConfig),
    External(ExternalFaultConfig),
    Python(PythonFaultModelConfig),
}

impl Default for GNSSSensorFaultModelConfig {
    fn default() -> Self {
        Self::AdditiveRobotCentered(AdditiveFaultConfig::default())
    }
}

#[derive(Debug, EnumToString)]
pub enum GNSSSensorFaultModelType {
    AdditiveRobotCentered(AdditiveFault<GNSSSensorVariablesFaults, GNSSSensorVariables>),
    AdditiveObservationCentered(AdditiveFault<GNSSSensorVariablesFaults, GNSSSensorVariables>),
    Clutter(ClutterFault<GNSSSensorVariablesFaults>),
    Misdetection(MisdetectionFault),
    External(ExternalFault),
    Python(PythonFaultModel),
}

impl GNSSSensorFaultModelType {
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
            | Self::Misdetection(_) => Ok(()),
        }
    }
}

/// Configuration of the [`GNSSSensor`].
#[config_derives]
pub struct GNSSSensorConfig {
    /// Observation period of the sensor.
    #[check]
    pub activation_time: Option<PeriodicityConfig>,
    /// Fault on the x, y positions, and on the x and y velocities
    #[check]
    pub faults: Vec<GNSSSensorFaultModelConfig>,
    #[check]
    pub filters: Vec<SensorFilterConfig<GNSSSensorVariablesFilter>>,
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

                SensorFilterConfig::show_filters(&self.filters, ui, ctx, unique_id);

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
    pub pose: Vector3<f32>,
    pub velocity: Vector2<f32>,
    pub applied_faults: Vec<GNSSSensorFaultModelConfig>,
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
    faults: Vec<GNSSSensorFaultModelType>,
    filters: Vec<SensorFilterType<GNSSSensorVariablesFilter>>,
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
                GNSSSensorFaultModelConfig::AdditiveRobotCentered(config) => {
                    GNSSSensorFaultModelType::AdditiveRobotCentered(AdditiveFault::from_config(
                        config,
                        va_factory,
                        initial_time,
                    ))
                }
                GNSSSensorFaultModelConfig::AdditiveObservationCentered(config) => {
                    GNSSSensorFaultModelType::AdditiveObservationCentered(
                        AdditiveFault::from_config(config, va_factory, initial_time),
                    )
                }
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
                    SensorFilterType::PythonFilter(f) => f.filter(time, obs, &state, None),
                    SensorFilterType::External(f) => f.filter(time, obs, &state, None),
                    SensorFilterType::RangeFilter(f) => {
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
                    _ => unimplemented!(
                        "{} filter not implemented for GNSSSensor",
                        filter.to_string()
                    ),
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
                    GNSSSensorFaultModelType::AdditiveRobotCentered(f) => {
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
                            obs.applied_faults.push(
                                GNSSSensorFaultModelConfig::AdditiveRobotCentered(
                                    f.config().clone(),
                                ),
                            );
                        }
                    }
                    _ => todo!(),
                }

                // fault_model.add_faults(
                //     time,
                //     time,
                //     &mut observation_list,
                //     SensorObservation::GNSS(GNSSObservation::default()),
                //     node.environment(),
                // );
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
