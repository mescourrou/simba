/*!
Provides a [`Sensor`] which can provide the transformation since the last observation.
*/

use std::sync::Arc;

use super::fault_models::fault_model::FaultModel;
use super::{Sensor, SensorObservation, SensorRecord};

use crate::config::NumberConfig;
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
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::enum_tools::EnumVariables;
use crate::utils::geometry::smallest_theta_diff;
use crate::utils::periodicity::{Periodicity, PeriodicityConfig};
use log::debug;
use nalgebra::{Matrix3, Vector2};
use serde_derive::{Deserialize, Serialize};
use simba_macros::{EnumToString, UIComponent, config_derives, enum_variables};

extern crate nalgebra as na;

enum_variables!(
    DisplacementSensorVariables;
    Filter, Prop, Faults: X, "x", "dx";
    Filter, Prop, Faults: Y, "y", "dy";
    Filter, Prop, Faults: Rotation, "rotation", "r";
    Filter, Prop, Faults: Translation, "translation", "t";
    Filter, Prop: Distance, "distance", "d";
    Filter, Prop: SelfVelocity, "self_velocity";
);

#[config_derives]
#[derive(UIComponent)]
#[show_all = "Faults"]
pub enum DisplacementSensorFaultModelConfig {
    AdditivePreDisplacement(
        AdditiveFaultConfig<DisplacementSensorVariablesFaults, DisplacementSensorVariablesProp>,
    ),
    AdditivePostDisplacement(
        AdditiveFaultConfig<DisplacementSensorVariablesFaults, DisplacementSensorVariablesProp>,
    ),
    Python(PythonFaultModelConfig),
    External(ExternalFaultConfig),
}

impl Default for DisplacementSensorFaultModelConfig {
    fn default() -> Self {
        Self::AdditivePreDisplacement(AdditiveFaultConfig::default())
    }
}

#[derive(Debug, EnumToString)]
pub enum DisplacementSensorFaultModelType {
    AdditivePreDisplacement(
        AdditiveFault<DisplacementSensorVariablesFaults, DisplacementSensorVariablesProp>,
    ),
    AdditivePostDisplacement(
        AdditiveFault<DisplacementSensorVariablesFaults, DisplacementSensorVariablesProp>,
    ),
    Python(PythonFaultModel),
    External(ExternalFault),
}

impl DisplacementSensorFaultModelType {
    pub fn post_init(
        &mut self,
        node: &mut crate::node::Node,
        initial_time: f32,
    ) -> crate::errors::SimbaResult<()> {
        match self {
            Self::Python(f) => f.post_init(node, initial_time),
            Self::External(f) => f.post_init(node, initial_time),
            Self::AdditivePreDisplacement(_) | Self::AdditivePostDisplacement(_) => Ok(()),
        }
    }
}

/// Configuration of the [`DisplacementSensor`].
#[config_derives]
pub struct DisplacementSensorConfig {
    /// Observation period of the sensor.
    #[check]
    pub activation_time: Option<PeriodicityConfig>,
    #[check]
    pub faults: Vec<DisplacementSensorFaultModelConfig>,
    #[check]
    pub filters: Vec<SensorFilterConfig<DisplacementSensorVariablesFilter>>,
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

                DisplacementSensorFaultModelConfig::show_all_mut(
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

                DisplacementSensorFaultModelConfig::show_all(&self.faults, ui, ctx, unique_id);
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
    pub applied_faults: Vec<DisplacementSensorFaultModelConfig>,
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
    pub applied_faults: Vec<DisplacementSensorFaultModelConfig>,
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
    faults: Vec<DisplacementSensorFaultModelType>,
    filters: Vec<SensorFilterType<DisplacementSensorVariablesFilter>>,
    lie_movement: bool,
}

impl DisplacementSensor {
    /// Makes a new [`DisplacementSensor`] from the given config.
    pub fn from_config(
        config: &DisplacementSensorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
        initial_state: &State,
    ) -> SimbaResult<Self> {
        let mut fault_models = Vec::new();
        for fault_config in &config.faults {
            fault_models.push(match fault_config {
                DisplacementSensorFaultModelConfig::AdditivePreDisplacement(cfg) => {
                    DisplacementSensorFaultModelType::AdditivePreDisplacement(
                        AdditiveFault::from_config(cfg, va_factory, initial_time),
                    )
                }
                DisplacementSensorFaultModelConfig::AdditivePostDisplacement(cfg) => {
                    DisplacementSensorFaultModelType::AdditivePostDisplacement(
                        AdditiveFault::from_config(cfg, va_factory, initial_time),
                    )
                }
                DisplacementSensorFaultModelConfig::Python(cfg) => {
                    DisplacementSensorFaultModelType::Python(PythonFaultModel::from_config(
                        cfg,
                        global_config,
                        initial_time,
                    )?)
                }
                DisplacementSensorFaultModelConfig::External(cfg) => {
                    DisplacementSensorFaultModelType::External(ExternalFault::from_config(
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
                self.last_state.pose.z.cos(),
                self.last_state.pose.z.sin(),
                0.,
                -self.last_state.pose.z.sin(),
                self.last_state.pose.z.cos(),
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

        let lie_distance = self.last_state.velocity.fixed_rows::<2>(0).norm()
            * (time - self.last_time.unwrap_or(time));

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
                            if f.match_exclusion(&DisplacementSensorVariablesFilter::mapped_values(
                                |variant| match variant {
                                    DisplacementSensorVariablesFilter::X => obs.translation.x,
                                    DisplacementSensorVariablesFilter::Y => obs.translation.y,
                                    DisplacementSensorVariablesFilter::Rotation => obs.rotation,
                                    DisplacementSensorVariablesFilter::Translation => {
                                        obs.translation.fixed_rows::<2>(0).norm()
                                    }
                                    DisplacementSensorVariablesFilter::Distance => lie_distance,
                                    DisplacementSensorVariablesFilter::SelfVelocity => {
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

        let mut observation_list = Vec::<SensorObservation>::new();
        if let Some(obs) = keep_observation {
            observation_list.push(obs);
            for fault_model in self.faults.iter_mut() {
                match fault_model {
                    DisplacementSensorFaultModelType::Python(f) => f.add_faults(
                        time,
                        time,
                        &mut observation_list,
                        SensorObservation::Displacement(DisplacementObservation::default()),
                        node.environment(),
                    ),
                    DisplacementSensorFaultModelType::External(f) => f.add_faults(
                        time,
                        time,
                        &mut observation_list,
                        SensorObservation::Displacement(DisplacementObservation::default()),
                        node.environment(),
                    ),
                    DisplacementSensorFaultModelType::AdditivePreDisplacement(f) => {
                        for obs in observation_list.iter_mut() {
                            if let SensorObservation::Displacement(o) = obs {
                                let new_values = f.add_faults(
                                    time,
                                    DisplacementSensorVariablesFaults::mapped_values(|variant| {
                                        match variant {
                                            DisplacementSensorVariablesFaults::X => o.translation.x,
                                            DisplacementSensorVariablesFaults::Y => o.translation.y,
                                            DisplacementSensorVariablesFaults::Rotation => {
                                                o.rotation
                                            }
                                            DisplacementSensorVariablesFaults::Translation => {
                                                o.translation.fixed_rows::<2>(0).norm()
                                            }
                                        }
                                    }),
                                    &DisplacementSensorVariablesProp::mapped_values(|variant| {
                                        match variant {
                                            DisplacementSensorVariablesProp::X => o.translation.x,
                                            DisplacementSensorVariablesProp::Y => o.translation.y,
                                            DisplacementSensorVariablesProp::Rotation => o.rotation,
                                            DisplacementSensorVariablesProp::Translation => {
                                                o.translation.fixed_rows::<2>(0).norm()
                                            }
                                            DisplacementSensorVariablesProp::Distance => {
                                                lie_distance
                                            }
                                            DisplacementSensorVariablesProp::SelfVelocity => {
                                                state.velocity.fixed_rows::<2>(0).norm()
                                            }
                                        }
                                    }),
                                );
                                if let Some(new_x) =
                                    new_values.get(&DisplacementSensorVariablesFaults::X)
                                {
                                    o.translation.x += new_x;
                                }
                                if let Some(new_y) =
                                    new_values.get(&DisplacementSensorVariablesFaults::Y)
                                {
                                    o.translation.y += new_y;
                                }
                                if let Some(new_r) =
                                    new_values.get(&DisplacementSensorVariablesFaults::Rotation)
                                {
                                    o.rotation += new_r;
                                }
                                if let Some(new_t) =
                                    new_values.get(&DisplacementSensorVariablesFaults::Translation)
                                {
                                    let current_t = o.translation.fixed_rows::<2>(0).norm();
                                    let new_t_vec = o.translation * (new_t / current_t);
                                    o.translation += new_t_vec;
                                }
                                o.applied_faults.push(
                                    DisplacementSensorFaultModelConfig::AdditivePreDisplacement(
                                        f.config().clone(),
                                    ),
                                );
                            } else {
                                unreachable!()
                            }
                        }
                    }
                    DisplacementSensorFaultModelType::AdditivePostDisplacement(f) => {
                        let dtheta = smallest_theta_diff(state.pose.z, self.last_state.pose.z);
                        let rotation_matrix = Matrix3::new(
                            dtheta.cos(),
                            dtheta.sin(),
                            0.,
                            -dtheta.sin(),
                            dtheta.cos(),
                            0.,
                            0.,
                            0.,
                            1.,
                        );
                        for obs in observation_list.iter_mut() {
                            if let SensorObservation::Displacement(o) = obs {
                                let mut transformed_translation =
                                    rotation_matrix.transform_vector(&o.translation);
                                let new_values = f.add_faults(
                                    time,
                                    DisplacementSensorVariablesFaults::mapped_values(|variant| {
                                        match variant {
                                            DisplacementSensorVariablesFaults::X => {
                                                transformed_translation.x
                                            }
                                            DisplacementSensorVariablesFaults::Y => {
                                                transformed_translation.y
                                            }
                                            DisplacementSensorVariablesFaults::Rotation => {
                                                o.rotation
                                            }
                                            DisplacementSensorVariablesFaults::Translation => {
                                                o.translation.fixed_rows::<2>(0).norm()
                                            }
                                        }
                                    }),
                                    &DisplacementSensorVariablesProp::mapped_values(|variant| {
                                        match variant {
                                            DisplacementSensorVariablesProp::X => {
                                                transformed_translation.x
                                            }
                                            DisplacementSensorVariablesProp::Y => {
                                                transformed_translation.y
                                            }
                                            DisplacementSensorVariablesProp::Rotation => o.rotation,
                                            DisplacementSensorVariablesProp::Translation => {
                                                o.translation.fixed_rows::<2>(0).norm()
                                            }
                                            DisplacementSensorVariablesProp::Distance => {
                                                lie_distance
                                            }
                                            DisplacementSensorVariablesProp::SelfVelocity => {
                                                state.velocity.fixed_rows::<2>(0).norm()
                                            }
                                        }
                                    }),
                                );
                                if let Some(new_x) =
                                    new_values.get(&DisplacementSensorVariablesFaults::X)
                                {
                                    transformed_translation.x += new_x;
                                }
                                if let Some(new_y) =
                                    new_values.get(&DisplacementSensorVariablesFaults::Y)
                                {
                                    transformed_translation.y += new_y;
                                }
                                o.translation = rotation_matrix
                                    .try_inverse()
                                    .expect("Failed to invert rotation matrix")
                                    .transform_vector(&transformed_translation);
                                if let Some(new_r) =
                                    new_values.get(&DisplacementSensorVariablesFaults::Rotation)
                                {
                                    o.rotation += new_r;
                                }
                                if let Some(new_t) =
                                    new_values.get(&DisplacementSensorVariablesFaults::Translation)
                                {
                                    let current_t = o.translation.fixed_rows::<2>(0).norm();
                                    let new_t_vec = o.translation * (new_t / current_t);
                                    o.translation += new_t_vec;
                                }
                                o.applied_faults.push(
                                    DisplacementSensorFaultModelConfig::AdditivePostDisplacement(
                                        f.config().clone(),
                                    ),
                                );
                            } else {
                                unreachable!()
                            }
                        }
                    }
                }
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
