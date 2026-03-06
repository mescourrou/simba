/*!
Provides a [`Sensor`] which can observe oriented landmarks in the frame of the robot.
*/

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
use crate::state_estimators::State;
use crate::utils::SharedMutex;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::enum_tools::EnumVariables;
use crate::utils::periodicity::{Periodicity, PeriodicityConfig};
use serde_derive::{Deserialize, Serialize};

use log::debug;
extern crate nalgebra as na;
use na::Vector3;
use simba_macros::config_derives;

use std::sync::{Arc, Mutex};
use std::vec;

enum_variables!(
    OrientedLandmarkSensorVariables;
    X, "x";
    Y, "y";
    Orientation, "orientation";
    R, "r";
    Theta, "theta";
    SelfVelocity, "self_velocity";
    Width, "width";
    Height, "height";
);

/// Configuration of the [`OrientedLandmarkSensor`].
#[config_derives]
pub struct OrientedLandmarkSensorConfig {
    /// Max distance of detection.
    pub detection_distance: f32,
    /// Observation period of the sensor.
    #[check]
    pub activation_time: Option<PeriodicityConfig>,
    #[check]
    pub faults: Vec<FaultModelConfig>,
    #[check]
    pub filters: Vec<SensorFilterConfig<OrientedLandmarkSensorVariables>>,
    /// If true, will detect all landmarks, even if they are behind obstacles (no raycasting).
    pub xray: bool,
}

impl Check for OrientedLandmarkSensorConfig {
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

impl Default for OrientedLandmarkSensorConfig {
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
impl UIComponent for OrientedLandmarkSensorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Oriented Landmark sensor")
            .id_salt(format!("oriented-landmark-sensor-{}", unique_id))
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
                        self.activation_time = Self::default().activation_time;
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("X-Ray mode:");
                    ui.checkbox(&mut self.xray, "");
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
        egui::CollapsingHeader::new("Oriented Landmark sensor")
            .id_salt(format!("oriented-landmark-sensor-{}", unique_id))
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

                SensorFilterConfig::show_filters(&self.filters, ui, ctx, unique_id);

                FaultModelConfig::show_faults(&self.faults, ui, ctx, unique_id);
            });
    }
}

/// Record of the [`OrientedLandmarkSensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct OrientedLandmarkSensorRecord {
    last_time: Option<f32>,
}

#[cfg(feature = "gui")]
impl UIComponent for OrientedLandmarkSensorRecord {
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

/// Observation of an [`OrientedLandmark`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OrientedLandmarkObservation {
    /// Id of the landmark
    pub id: i32,
    pub labels: Vec<String>,
    /// Pose of the landmark
    pub pose: Vector3<f32>,
    /// Height of the landmark, used for obstruction checks
    /// Use 0 for fully transparent landmarks
    pub height: f32,
    /// Can be 0 for ponctual landmarks
    pub width: f32,
    pub applied_faults: Vec<FaultModelConfig>,
}

impl Default for OrientedLandmarkObservation {
    fn default() -> Self {
        Self {
            id: 0,
            labels: Vec::new(),
            pose: Vector3::new(0., 0., 0.),
            height: 1.,
            width: 0.,
            applied_faults: Vec::new(),
        }
    }
}

impl Recordable<OrientedLandmarkObservationRecord> for OrientedLandmarkObservation {
    fn record(&self) -> OrientedLandmarkObservationRecord {
        OrientedLandmarkObservationRecord {
            id: self.id,
            labels: self.labels.clone(),
            pose: self.pose.into(),
            height: self.height,
            width: self.width,
            applied_faults: self.applied_faults.clone(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct OrientedLandmarkObservationRecord {
    /// Id of the landmark
    pub id: i32,
    pub labels: Vec<String>,
    /// Pose of the landmark
    pub pose: [f32; 3],
    pub height: f32,
    pub width: f32,
    pub applied_faults: Vec<FaultModelConfig>,
}

#[cfg(feature = "gui")]
impl UIComponent for OrientedLandmarkObservationRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!("Id: {}", self.id));
            ui.label("Labels:");
            for label in &self.labels {
                ui.label(format!("- {}", label));
            }
            ui.label(format!(
                "Pose: ({}, {}, {})",
                self.pose[0], self.pose[1], self.pose[2]
            ));
            if self.height <= 0. {
                ui.label(format!("Height: {}", self.height));
            }
            if self.width <= 0. {
                ui.label(format!("Width: {}", self.width));
            }
            if self.applied_faults.is_empty() {
                ui.label("No applied faults.");
            } else {
                ui.label("Applied faults:");
                for fault in &self.applied_faults {
                    ui.label(format!("{:?}", fault));
                }
            }
        });
    }
}

/// Sensor which observe the map landmarks.
#[derive(Debug)]
pub struct OrientedLandmarkSensor {
    /// Detection distance
    detection_distance: f32,
    /// Observation period
    activation_time: Option<Periodicity>,
    /// Last observation time.
    last_time: Option<f32>,
    faults: SharedMutex<Vec<Box<dyn FaultModel>>>,
    filters: Vec<SensorFilterType<OrientedLandmarkSensorVariables>>,
    /// If true, will detect all landmarks, even if they are behind obstacles (no raycasting).
    xray: bool,
}

impl OrientedLandmarkSensor {
    /// Makes a new [`OrientedLandmarkSensor`] from the given config.
    ///
    /// The map path is relative to the config path of the simulator.
    pub fn from_config(
        config: &OrientedLandmarkSensorConfig,
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
            detection_distance: config.detection_distance,
            activation_time,
            last_time: None,
            faults: fault_models,
            filters,
            xray: config.xray,
        })
    }
}

use crate::node::Node;

impl Sensor for OrientedLandmarkSensor {
    fn post_init(&mut self, node: &mut Node, initial_time: f32) -> crate::errors::SimbaResult<()> {
        for filter in self.filters.iter_mut() {
            match filter {
                SensorFilterType::PythonFilter(f) => f.post_init(node, initial_time)?,
                SensorFilterType::External(f) => f.post_init(node, initial_time)?,
                _ => (), // No post_init needed for other filters for now
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
        let state = if let Some(arc_physic) = node.physics() {
            let physic = arc_physic.read().unwrap();
            physic.state(time).clone()
        } else {
            State::new() // 0
        };

        let rotation_matrix =
            nalgebra::geometry::Rotation3::from_euler_angles(0., 0., state.pose.z);

        let observable_landmarks = node.environment().get_observable_landmarks(
            &state.pose.fixed_rows::<2>(0).clone_owned(),
            if self.xray { None } else { Some(0.) },
            self.detection_distance,
            Some(node.name()),
        );

        for (i, landmark) in observable_landmarks.iter().enumerate() {
            let landmark_seed = (i + 1) as f32 / (100. * (time - self.last_time.unwrap_or(-1.)))
                * ((landmark.id + 1) as f32);
            let pose = rotation_matrix.transpose() * (landmark.pose - state.pose);
            let obs = SensorObservation::OrientedLandmark(OrientedLandmarkObservation {
                id: landmark.id,
                labels: landmark.labels.clone(),
                pose,
                applied_faults: Vec::new(),
                height: landmark.height,
                width: landmark.width,
            });
            let mut new_obs = Vec::new();

            let mut keep_observation = Some(obs);

            for filter in self.filters.iter() {
                if let Some(obs) = keep_observation {
                    keep_observation = match filter {
                        SensorFilterType::External(f) => f.filter(time, obs, &state, None),
                        SensorFilterType::PythonFilter(f) => f.filter(time, obs, &state, None),
                        SensorFilterType::RangeFilter(f) => {
                            if let SensorObservation::OrientedLandmark(obs) = obs {
                                if f.match_exclusion(
                                    &OrientedLandmarkSensorVariables::mapped_values(|variant| {
                                        match variant {
                                            OrientedLandmarkSensorVariables::X => obs.pose.x,
                                            OrientedLandmarkSensorVariables::Y => obs.pose.y,
                                            OrientedLandmarkSensorVariables::Orientation => {
                                                obs.pose.z
                                            }
                                            OrientedLandmarkSensorVariables::R => {
                                                obs.pose.fixed_rows::<2>(0).norm()
                                            }
                                            OrientedLandmarkSensorVariables::Theta => {
                                                obs.pose.y.atan2(obs.pose.x)
                                            }
                                            OrientedLandmarkSensorVariables::SelfVelocity => {
                                                state.velocity.fixed_rows::<2>(0).norm()
                                            }
                                            OrientedLandmarkSensorVariables::Width => obs.width,
                                            OrientedLandmarkSensorVariables::Height => obs.height,
                                        }
                                    }),
                                ) {
                                    None
                                } else {
                                    Some(SensorObservation::OrientedLandmark(obs))
                                }
                            } else {
                                unreachable!()
                            }
                        }
                        SensorFilterType::IdFilter(f) => {
                            if let SensorObservation::OrientedLandmark(obs) = obs {
                                if f.match_exclusion(&[obs.id.to_string()]) {
                                    None
                                } else {
                                    Some(SensorObservation::OrientedLandmark(obs))
                                }
                            } else {
                                unreachable!()
                            }
                        }
                        SensorFilterType::LabelFilter(f) => {
                            if let SensorObservation::OrientedLandmark(obs) = obs {
                                if f.match_exclusion(&obs.labels) {
                                    None
                                } else {
                                    Some(SensorObservation::OrientedLandmark(obs))
                                }
                            } else {
                                unreachable!()
                            }
                        }
                    }
                } else {
                    break;
                }
            }

            if let Some(observation) = keep_observation {
                new_obs.push(observation); // Not adding directly to observation_list to apply faults only once
                for fault_model in self.faults.lock().unwrap().iter_mut() {
                    fault_model.add_faults(
                        time,
                        time + landmark_seed,
                        &mut new_obs,
                        SensorObservation::OrientedLandmark(OrientedLandmarkObservation::default()),
                        node.environment(),
                    );
                }
                observation_list.extend(new_obs);
            } else if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
                debug!(
                    "Observation {i} of landmark {} was filtered out",
                    landmark.id
                );
            }
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

impl Recordable<SensorRecord> for OrientedLandmarkSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::OrientedLandmarkSensor(OrientedLandmarkSensorRecord {
            last_time: self.last_time,
        })
    }
}
