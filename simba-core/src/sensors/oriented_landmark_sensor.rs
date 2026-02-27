/*!
Provides a [`Sensor`] which can observe oriented landmarks in the frame of the robot.
*/

use super::fault_models::fault_model::{
    FaultModel, FaultModelConfig, make_fault_model_from_config,
};
use super::{Sensor, SensorObservation, SensorRecord};

use crate::constants::TIME_ROUND;
use crate::environment::oriented_landmark::OrientedLandmark;
use crate::environment::{Environment, Map};
use crate::logger::{InternalLog, is_enabled};
use crate::plugin_api::PluginAPI;
use crate::recordable::Recordable;
use crate::sensors::sensor_filters::{
    SensorFilter, SensorFilterConfig, make_sensor_filter_from_config,
};
use crate::simulator::SimulatorConfig;
use crate::state_estimators::State;
use crate::utils::SharedMutex;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::geometry::{
    segment_circle_intersection, segment_to_line_intersection, segment_triangle_intersection,
    segments_intersection,
};
use crate::utils::maths::round_precision;
use crate::utils::periodicity::{Periodicity, PeriodicityConfig};
#[cfg(feature = "gui")]
use crate::{
    constants::TIME_ROUND_DECIMALS,
    gui::{UIComponent, utils::path_finder},
};
use nalgebra::Vector2;
use serde_derive::{Deserialize, Serialize};

use log::{debug, error};
extern crate nalgebra as na;
use na::Vector3;
use simba_macros::config_derives;

use std::path::Path;
use std::sync::{Arc, Mutex};
use std::{fmt, vec};

/// Configuration of the [`OrientedLandmarkSensor`].
#[config_derives]
pub struct OrientedLandmarkSensorConfig {
    /// Max distance of detection.
    #[check[ge(0.)]]
    pub detection_distance: f32,
    /// Observation period of the sensor.
    pub activation_time: Option<PeriodicityConfig>,
    #[check]
    pub faults: Vec<FaultModelConfig>,
    #[check]
    pub filters: Vec<SensorFilterConfig>,
    /// If true, will detect all landmarks, even if they are behind obstacles (no raycasting).
    pub xray: bool,
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
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct OrientedLandmarkSensorRecord {
    last_time: Option<f32>  ,
}

impl Default for OrientedLandmarkSensorRecord {
    fn default() -> Self {
        Self { last_time: None }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for OrientedLandmarkSensorRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.label(format!("Last time: {}", match self.last_time {
            Some(t) => t.to_string(),
            None => "None".to_string(),
        }));
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
    filters: SharedMutex<Vec<Box<dyn SensorFilter>>>,
    /// If true, will detect all landmarks, even if they are behind obstacles (no raycasting).
    xray: bool,
}

impl OrientedLandmarkSensor {
    /// Makes a new [`OrientedLandmarkSensor`].
    pub fn new() -> Self {
        OrientedLandmarkSensor::from_config(
            &OrientedLandmarkSensorConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &"NoName".to_string(),
            &DeterministRandomVariableFactory::default(),
            0.0,
        )
    }

    /// Makes a new [`OrientedLandmarkSensor`] from the given config.
    ///
    /// The map path is relative to the config path of the simulator.
    pub fn from_config(
        config: &OrientedLandmarkSensorConfig,
        _plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        robot_name: &String,
        va_factory: &DeterministRandomVariableFactory,
        initial_time: f32,
    ) -> Self {
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

        let filters = Arc::new(Mutex::new(Vec::new()));
        let mut unlock_filters = filters.lock().unwrap();
        for filter_config in &config.filters {
            unlock_filters.push(make_sensor_filter_from_config(
                filter_config,
                global_config,
                initial_time,
            ));
        }
        drop(unlock_filters);

        let activation_time = config
            .activation_time
            .as_ref()
            .map(|p| Periodicity::from_config(p, va_factory, initial_time));

        Self {
            detection_distance: config.detection_distance,
            activation_time,
            last_time: None,
            faults: fault_models,
            filters,
            xray: config.xray,
        }
    }
}

impl Default for OrientedLandmarkSensor {
    fn default() -> Self {
        Self::new()
    }
}

use crate::node::Node;

impl Sensor for OrientedLandmarkSensor {
    fn post_init(&mut self, node: &mut Node, initial_time: f32) -> crate::errors::SimbaResult<()> {
        for filter in self.filters.lock().unwrap().iter_mut() {
            filter.post_init(node, initial_time)?;
        }
        for fault_model in self.faults.lock().unwrap().iter_mut() {
            fault_model.post_init(node, initial_time)?;
        }
        Ok(())
    }

    fn get_observations(&mut self, node: &mut Node, time: f32) -> Vec<SensorObservation> {
        let mut observation_list = Vec::<SensorObservation>::new();
        if let Some(last_time) = self.last_time && (time - last_time).abs() < TIME_ROUND {
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

        // let mut in_range_landmarks = Vec::new();

        // // List all landmarks in range
        // // Intersections concerns only non-ponctual landmarks and contains either the intersection
        // // with the detection circle, or extremitie(s) of the landmark segment if inside the
        // // detection circle
        // for landmark in &self.map.landmarks {
        //     let d = ((landmark.pose.x - state.pose.x).powi(2)
        //         + (landmark.pose.y - state.pose.y).powi(2))
        //     .sqrt();
        //     let mut intersections = None;
        //     let half_width = landmark.width / 2.0;
        //     let dir_vector = Vector2::new(
        //         half_width * (landmark.pose.z + std::f32::consts::FRAC_PI_2).cos(),
        //         half_width * (landmark.pose.z + std::f32::consts::FRAC_PI_2).sin(),
        //     );
        //     let pt1 = landmark.pose.fixed_rows::<2>(0).clone_owned() + dir_vector;
        //     let pt2 = landmark.pose.fixed_rows::<2>(0).clone_owned() - dir_vector;
        //     if d > self.detection_distance {
        //         if landmark.width == 0.0 {
        //             continue;
        //         }

        //         intersections =
        //             segment_circle_intersection(&pt1, &pt2, &state_pos, self.detection_distance);
        //         if intersections.is_none() {
        //             continue;
        //         }
        //     } else if landmark.width > 0.0 {
        //         intersections =
        //             segment_circle_intersection(&pt1, &pt2, &state_pos, self.detection_distance);
        //         if intersections.is_none() {
        //             // Entire segment is inside the detection circle
        //             intersections = Some((pt1, pt2));
        //         }
        //     }

        //     // In range landmark
        //     // Ponctual one has no intersection
        //     // Otherwise, intersection is either with circle or extremities
        //     in_range_landmarks.push((landmark, intersections));
        // }

        // // Create observations
        // for (landmark, intersections) in &in_range_landmarks {
        //     let mut intersections = if let Some((p1, p2)) = intersections {
        //         vec![*p1, *p2]
        //     } else {
        //         vec![landmark.pose.fixed_rows::<2>(0).clone_owned()]
        //     };
        //     if !self.xray {
        //         // Check for obstruction
        //         // TODO: use a more efficient algorithm, and less specific case-oriented
        //         for (possible_obstruction, possible_intersect) in &in_range_landmarks {
        //             if is_enabled(InternalLog::SensorManagerDetailed) {
        //                 debug!(
        //                     "Checking obstruction of landmark {} by landmark {}",
        //                     landmark.id, possible_obstruction.id
        //                 );
        //             }
        //             // Cannot obstruct itself
        //             // If obstruction is lower than landmark, cannot obstruct
        //             // If obstruction has no height, cannot obstruct
        //             if possible_obstruction.id == landmark.id
        //                 || possible_obstruction.height < landmark.height
        //                 || possible_obstruction.height <= 0.0
        //             {
        //                 continue;
        //             }
        //             // Ponctual landmark cannot obstruct
        //             if possible_intersect.is_none() {
        //                 continue;
        //             }
        //             if is_enabled(InternalLog::SensorManagerDetailed) {
        //                 debug!(
        //                     "Possible obstruction intersection points: {:?}",
        //                     possible_intersect
        //                 );
        //             }
        //             let (p1, p2) = possible_intersect.unwrap();
        //             if intersections.len() >= 2 {
        //                 assert!(
        //                     intersections.len() % 2 == 0,
        //                     "Intersections should be pairs of points."
        //                 );
        //                 if is_enabled(InternalLog::SensorManagerDetailed) {
        //                     debug!("Current intersections: {:?}", intersections);
        //                 }
        //                 let mut new_intersections = Vec::new();
        //                 for i in 0..intersections.len() / 2 {
        //                     // Look at a non-ponctual landmark. Check if the landmark is obstructed by segment
        //                     let mut chunk_intersections = intersections[2 * i..2 * i + 2].to_vec();
        //                     if let Some((i1, i2)) = segment_triangle_intersection(
        //                         &p1,
        //                         &p2,
        //                         &state_pos,
        //                         &chunk_intersections[0],
        //                         &chunk_intersections[1],
        //                     ) {
        //                         let projected1 = segment_to_line_intersection(
        //                             &state_pos,
        //                             &i1,
        //                             &chunk_intersections[0],
        //                             &chunk_intersections[1],
        //                         );
        //                         let projected2 = segment_to_line_intersection(
        //                             &state_pos,
        //                             &i2,
        //                             &chunk_intersections[0],
        //                             &chunk_intersections[1],
        //                         );
        //                         let chunk_length = (chunk_intersections[1]
        //                             - chunk_intersections[0])
        //                             .norm_squared();
        //                         if is_enabled(InternalLog::SensorManagerDetailed) {
        //                             debug!(
        //                                 "Projected intersections: {:?}, {:?}",
        //                                 projected1, projected2
        //                             );
        //                             debug!("Triangle intersection: i1: {:?}, i2: {:?}", i1, i2);
        //                         }
        //                         if projected1.is_none() && projected2.is_none() {
        //                             // No intersection, not obstructed, keep points
        //                             if is_enabled(InternalLog::SensorManagerDetailed) {
        //                                 debug!(
        //                                     "No intersection after projection, not obstructed, keeping points."
        //                                 );
        //                             }
        //                             new_intersections.extend_from_slice(&chunk_intersections);
        //                             continue;
        //                         }
        //                         if projected1.is_none()
        //                             && let Some(projected2) = projected2
        //                         {
        //                             let dot2 = (projected2 - chunk_intersections[0])
        //                                 .dot(&(chunk_intersections[1] - chunk_intersections[0]));
        //                             if dot2 - 1e-3 < 0. || dot2 + 1e-3 > chunk_length {
        //                                 // projected2 is out of segment, not obstructed, keep points
        //                                 if is_enabled(InternalLog::SensorManagerDetailed) {
        //                                     debug!(
        //                                         "Projected2 is out of segment, not obstructed, keeping points."
        //                                     );
        //                                 }
        //                                 new_intersections.extend_from_slice(&chunk_intersections);
        //                                 continue;
        //                             }
        //                             // partially obstructed, keep second part
        //                             if is_enabled(InternalLog::SensorManagerDetailed) {
        //                                 debug!("Partially obstructed, keeping second part.");
        //                             }
        //                             new_intersections.push(projected2);
        //                             new_intersections.push(chunk_intersections[1]);
        //                         } else if let Some(projected1) = projected1
        //                             && projected2.is_none()
        //                         {
        //                             let dot1 = (projected1 - chunk_intersections[0])
        //                                 .dot(&(chunk_intersections[1] - chunk_intersections[0]));
        //                             if dot1 - 1e-3 < 0. || dot1 + 1e-3 > chunk_length {
        //                                 // projected1 is out of segment, not obstructed, keep points
        //                                 if is_enabled(InternalLog::SensorManagerDetailed) {
        //                                     debug!(
        //                                         "Projected1 is out of segment, not obstructed, keeping points."
        //                                     );
        //                                 }
        //                                 new_intersections.extend_from_slice(&chunk_intersections);
        //                                 continue;
        //                             }
        //                             // partially obstructed, keep first part
        //                             if is_enabled(InternalLog::SensorManagerDetailed) {
        //                                 debug!("Partially obstructed, keeping first part.");
        //                             }
        //                             new_intersections.push(chunk_intersections[0]);
        //                             new_intersections.push(projected1);
        //                         } else {
        //                             let projected1 = projected1.unwrap();
        //                             let projected2 = projected2.unwrap();
        //                             let dot1 = (projected1 - chunk_intersections[0])
        //                                 .dot(&(chunk_intersections[1] - chunk_intersections[0]));
        //                             let dot2 = (projected2 - chunk_intersections[0])
        //                                 .dot(&(chunk_intersections[1] - chunk_intersections[0]));
        //                             if is_enabled(InternalLog::SensorManagerDetailed) {
        //                                 debug!(
        //                                     "Dot products: dot1: {}, dot2: {}, chunk_length: {}",
        //                                     dot1, dot2, chunk_length
        //                                 );
        //                             }
        //                             let inside1 = dot1 - 1e-3 > 0. && dot1 + 1e-3 < chunk_length;
        //                             let inside2 = dot2 - 1e-3 > 0. && dot2 + 1e-3 < chunk_length;
        //                             if dot1 > dot2 {
        //                                 chunk_intersections.swap(0, 1);
        //                             }

        //                             if !inside1 && !inside2 {
        //                                 if (dot1 < 0. && dot2 < 0.)
        //                                     || (dot1 > chunk_length && dot2 > chunk_length)
        //                                 {
        //                                     // both projected points are out of segment, not obstructed, keep points
        //                                     if is_enabled(InternalLog::SensorManagerDetailed) {
        //                                         debug!(
        //                                             "Both projected points are out of segment, not obstructed, keeping points."
        //                                         );
        //                                     }
        //                                     new_intersections
        //                                         .extend_from_slice(&chunk_intersections);
        //                                     continue;
        //                                 } else {
        //                                     if is_enabled(InternalLog::SensorManagerDetailed) {
        //                                         debug!(
        //                                             "Projected points are on different sides, fully obstructed, removing points."
        //                                         );
        //                                     }
        //                                     continue;
        //                                 }
        //                             }
        //                             if inside1 && !inside2 {
        //                                 // partially obstructed, keep first part
        //                                 if (chunk_intersections[0] - projected1).norm() > 1e-3 {
        //                                     if is_enabled(InternalLog::SensorManagerDetailed) {
        //                                         debug!("Partially obstructed, keeping first part.");
        //                                     }
        //                                     new_intersections.push(chunk_intersections[0]);
        //                                     new_intersections.push(projected1);
        //                                 }
        //                                 continue;
        //                             }
        //                             if !inside1 && inside2 {
        //                                 // partially obstructed, keep second part
        //                                 if (chunk_intersections[1] - projected2).norm() > 1e-3 {
        //                                     if is_enabled(InternalLog::SensorManagerDetailed) {
        //                                         debug!(
        //                                             "Partially obstructed, keeping second part."
        //                                         );
        //                                     }
        //                                     new_intersections.push(projected2);
        //                                     new_intersections.push(chunk_intersections[1]);
        //                                 }
        //                                 continue;
        //                             }
        //                             // both inside, keep external parts
        //                             if is_enabled(InternalLog::SensorManagerDetailed) {
        //                                 debug!("Partially obstructed, keeping external parts.");
        //                             }

        //                             if (chunk_intersections[0] - projected1).norm() > 1e-3 {
        //                                 new_intersections.push(chunk_intersections[0]);
        //                                 new_intersections.push(projected1);
        //                             }
        //                             if (chunk_intersections[1] - projected2).norm() > 1e-3 {
        //                                 new_intersections.push(projected2);
        //                                 new_intersections.push(chunk_intersections[1]);
        //                             }
        //                         }
        //                     } else {
        //                         // else: no intersection, not obstructed, keep points
        //                         new_intersections.extend_from_slice(&chunk_intersections);
        //                     }
        //                 }
        //                 intersections = new_intersections;
        //             } else if intersections.len() == 1 {
        //                 // Try to look at a ponctual landmark, check if segment intersects obstruction
        //                 if segments_intersection(&state_pos, &intersections[0], &p1, &p2).is_some()
        //                 {
        //                     // Obstructed, remove point
        //                     intersections.clear();
        //                     break;
        //                 }
        //             }
        //         }
        //         if is_enabled(InternalLog::SensorManagerDetailed) {
        //             debug!(
        //                 "Intersections after obstruction checks: {:?}",
        //                 intersections
        //             );
        //         }
        //         if intersections.is_empty() {
        //             // Fully obstructed
        //             continue;
        //         }
        //     }

        //     let mut observed_poses = Vec::new();
        //     let mut observed_width = Vec::new();
        //     if intersections.len() == 1 {
        //         observed_poses.push(intersections[0].insert_row(2, landmark.pose.z));
        //         observed_width.push(0.0);
        //     } else {
        //         for i in 0..intersections.len() / 2 {
        //             let chunk_intersections = &intersections[2 * i..2 * i + 2];
        //             let p1 = chunk_intersections[0];
        //             let p2 = chunk_intersections[1];
        //             observed_poses.push(Vector3::new(
        //                 0.5 * (p1.x + p2.x),
        //                 0.5 * (p1.y + p2.y),
        //                 landmark.pose.z,
        //             ));
        //             observed_width.push((p2 - p1).norm());
        //         }
        //     }

        //     for (i, (pose, width)) in observed_poses.iter().zip(observed_width.iter()).enumerate() {
        //         let landmark_seed =
        //             (i + 1) as f32 / (100. * (time - self.last_time)) * ((landmark.id + 1) as f32);
        //         let pose = rotation_matrix.transpose() * (pose - state.pose);
        //         let mut new_obs = Vec::new();
        //         let obs = SensorObservation::OrientedLandmark(OrientedLandmarkObservation {
        //             id: landmark.id,
        //             labels: landmark.labels.clone(),
        //             pose,
        //             applied_faults: Vec::new(),
        //             height: landmark.height,
        //             width: *width,
        //         });
        //         if let Some(observation) = self
        //             .filters
        //             .lock()
        //             .unwrap()
        //             .iter()
        //             .try_fold(obs, |obs, filter| filter.filter(time, obs, &state, None))
        //         {
        //             new_obs.push(observation); // Not adding directly to observation_list to apply faults only once
        //             for fault_model in self.faults.lock().unwrap().iter_mut() {
        //                 fault_model.add_faults(
        //                     time,
        //                     time + landmark_seed,
        //                     &mut new_obs,
        //                     SensorObservation::OrientedLandmark(
        //                         OrientedLandmarkObservation::default(),
        //                     ),
        //                 );
        //             }
        //             observation_list.extend(new_obs);
        //         } else if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
        //             debug!(
        //                 "Observation {i} of landmark {} was filtered out",
        //                 landmark.id
        //             );
        //         }
        //     }
        // }

        let observable_landmarks = node.environment().get_observable_landmarks(
            &state.pose.fixed_rows::<2>(0).clone_owned(),
            if self.xray { None } else { Some(0.) },
            self.detection_distance,
        );
        debug!("Observable landmarks: {:?}", observable_landmarks);
        for (i, landmark) in observable_landmarks.iter().enumerate() {
            let landmark_seed =
                (i + 1) as f32 / (100. * (time - self.last_time.unwrap_or(-1.))) * ((landmark.id + 1) as f32);
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
            if let Some(observation) = self
                .filters
                .lock()
                .unwrap()
                .iter()
                .try_fold(obs, |obs, filter| filter.filter(time, obs, &state, None))
            {
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
        self.activation_time.as_mut().map(|p| p.update(time));
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
