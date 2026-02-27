#[cfg(feature = "gui")]
use std::collections::BTreeMap;
use std::{
    collections::HashMap,
    path::Path,
    sync::{Arc, RwLock},
};

use log::debug;
use nalgebra::{Vector2, Vector3};
use serde::{Deserialize, Serialize};
use simba_macros::config_derives;

use crate::{
    environment::oriented_landmark::OrientedLandmark,
    errors::{SimbaError, SimbaErrorTypes, SimbaResult},
    logger::{InternalLog, is_enabled},
    node::NodeMetaData,
    utils::{
        SharedRoLock, SharedRwLock,
        geometry::{
            segment_circle_intersection, segment_to_line_intersection,
            segment_triangle_intersection, segments_intersection,
        },
    },
};
#[cfg(feature = "gui")]
use crate::{gui::utils::path_finder, simulator::SimulatorConfig};

pub mod oriented_landmark;

#[config_derives]
#[derive(Default)]
pub struct EnvironmentConfig {
    pub map_path: Option<String>,
}

#[cfg(feature = "gui")]
impl crate::gui::UIComponent for EnvironmentConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        _unique_id: &str,
    ) {
        ui.horizontal(|ui| {
            ui.label("Map path: ");
            if let Some(map_path) = &mut self.map_path {
                path_finder(ui, map_path, &global_config.base_path);
                if ui.button("-").clicked() {
                    self.map_path = None;
                }
            } else if ui.button("+").clicked() {
                self.map_path = Some(String::new());
            }
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.horizontal(|ui| {
            ui.label("Map path: ");
            if let Some(map_path) = &self.map_path {
                ui.label(map_path);
            } else {
                ui.label("None");
            }
        });
    }
}

type TwoPoints = (Vector2<f32>, Vector2<f32>);
type CacheValue = (Vector2<f32>, f32, Vec<(OrientedLandmark, Option<TwoPoints>)>);

#[derive(Debug, Clone, Default)]
pub struct Environment {
    map: Map,
    meta_data_list: SharedRwLock<HashMap<String, SharedRoLock<NodeMetaData>>>,
    /// Cache for landmark_in_range, to avoid recomputing it multiple times for the same position and max_distance.
    cache: SharedRwLock<
        HashMap<
            String,
            CacheValue,
        >,
    >,
}

impl Environment {
    pub fn from_config(
        config: &EnvironmentConfig,
        global_config: &SimulatorConfig,
    ) -> SimbaResult<Self> {
        let map = if let Some(map_path) = &config.map_path {
            Map::load_from_path(&global_config.base_path.join(map_path))?
        } else {
            Map::new()
        };
        Ok(Self {
            map,
            meta_data_list: Arc::new(RwLock::new(HashMap::new())),
            cache: Arc::new(RwLock::new(HashMap::new())),
        })
    }

    pub fn map(&self) -> &Map {
        &self.map
    }


    /// Get the list of landmarks that are in range from the given position.
    /// For widthed landmarks, they are returned if they are in the observation circle or intersect it.
    /// The intersection points are also returned, which can be extremities of the landmark of intersection with the observation circle.
    fn landmarks_in_range(
        &self,
        position: &Vector2<f32>,
        max_distance: f32,
        cache_key: Option<String>,
    ) -> Vec<(OrientedLandmark, Option<TwoPoints>)> {
        if let Some(cache_key) = &cache_key
            && let Some((cached_position, cached_distance, cached_landmarks)) =
                self.cache.read().unwrap().get(cache_key)
            && (cached_position - position).norm() < 1e-6
            && (*cached_distance - max_distance).abs() < 1e-6
        {
            if is_enabled(InternalLog::EnvironmentDetailed) {
                debug!("Cache hit for landmarks_in_range with key {}", cache_key);
            }
            return cached_landmarks.clone();
        }

        let mut in_range_landmarks = Vec::new();

        // List all landmarks in range
        // Intersections concerns only non-ponctual landmarks and contains either the intersection
        // with the detection circle, or extremitie(s) of the landmark segment if inside the
        // detection circle
        for landmark in &self.map.landmarks {
            let d = ((landmark.pose.x - position.x).powi(2)
                + (landmark.pose.y - position.y).powi(2))
            .sqrt();
            let mut intersections = None;
            let half_width = landmark.width / 2.0;
            let dir_vector = Vector2::new(
                half_width * (landmark.pose.z + std::f32::consts::FRAC_PI_2).cos(),
                half_width * (landmark.pose.z + std::f32::consts::FRAC_PI_2).sin(),
            );
            let pt1 = landmark.pose.fixed_rows::<2>(0).clone_owned() + dir_vector;
            let pt2 = landmark.pose.fixed_rows::<2>(0).clone_owned() - dir_vector;
            if d > max_distance {
                if landmark.width == 0.0 {
                    continue;
                }

                intersections = segment_circle_intersection(&pt1, &pt2, position, max_distance);
                if intersections.is_none() {
                    continue;
                }
            } else if landmark.width > 0.0 {
                intersections = segment_circle_intersection(&pt1, &pt2, position, max_distance);
                if intersections.is_none() {
                    // Entire segment is inside the detection circle
                    intersections = Some((pt1, pt2));
                }
            }

            // In range landmark
            // Ponctual one has no intersection
            // Otherwise, intersection is either with circle or extremities
            in_range_landmarks.push((landmark.clone(), intersections));
        }

        if let Some(cache_key) = cache_key {
            self.cache.write().unwrap().insert(
                cache_key,
                (
                    position.clone_owned(),
                    max_distance,
                    in_range_landmarks.clone(),
                ),
            );
        }

        in_range_landmarks
    }

    /// Get the list of landmarks that are observable from the given position, considering the detection distance and possible obstructions.
    ///
    /// # Arguments
    /// * `position` - The position of the observer.
    /// * `observer_height` - The height of the observer, used for obstruction checks. If None, no obstruction checks are performed (equivalent to xray mode).
    /// * `max_distance` - The maximum distance at which landmarks can be observed.
    ///
    /// # Returns
    /// A vector of observed landmarks, with their observed pose and width (if partially observed) and in the map frame.
    pub fn get_observable_landmarks(
        &self,
        position: &Vector2<f32>,
        observer_height: Option<f32>,
        max_distance: f32,
        cache_key: Option<String>,
    ) -> Vec<OrientedLandmark> {
        let in_range_landmarks = self.landmarks_in_range(position, max_distance, cache_key);

        let mut observed_landmarks = Vec::new();

        // Create observations
        for (landmark, intersections) in &in_range_landmarks {
            let mut intersections = if let Some((p1, p2)) = intersections {
                vec![p1.clone_owned(), p2.clone_owned()]
            } else {
                vec![landmark.pose.fixed_rows::<2>(0).clone_owned()]
            };
            if let Some(observer_height) = observer_height {
                // Check for obstruction
                // TODO: use a more efficient algorithm, and less specific case-oriented
                for (possible_obstruction, possible_intersect) in &in_range_landmarks {
                    if is_enabled(InternalLog::Environment)
                        || is_enabled(InternalLog::EnvironmentDetailed)
                    {
                        debug!(
                            "Checking obstruction of landmark {} by landmark {}",
                            landmark.id, possible_obstruction.id
                        );
                    }
                    if possible_obstruction.height < observer_height {
                        if is_enabled(InternalLog::EnvironmentDetailed) {
                            debug!(
                                "Possible obstruction {} is lower than observer, cannot obstruct.",
                                possible_obstruction.id
                            );
                        }
                        continue;
                    }

                    // Cannot obstruct itself
                    // If obstruction is lower than landmark, cannot obstruct
                    // If obstruction has no height, cannot obstruct
                    if possible_obstruction.id == landmark.id
                        || possible_obstruction.height < landmark.height
                        || possible_obstruction.height <= 0.0
                    {
                        continue;
                    }
                    // Ponctual landmark cannot obstruct
                    if possible_intersect.is_none() {
                        continue;
                    }
                    if is_enabled(InternalLog::EnvironmentDetailed) {
                        debug!(
                            "Possible obstruction intersection points: {:?}",
                            possible_intersect
                        );
                    }
                    let (p1, p2) = possible_intersect.unwrap();
                    if intersections.len() >= 2 {
                        assert!(
                            intersections.len() % 2 == 0,
                            "Intersections should be pairs of points."
                        );
                        if is_enabled(InternalLog::EnvironmentDetailed) {
                            debug!("Current intersections: {:?}", intersections);
                        }
                        let mut new_intersections = Vec::new();
                        for i in 0..intersections.len() / 2 {
                            // Look at a non-ponctual landmark. Check if the landmark is obstructed by segment
                            let mut chunk_intersections = intersections[2 * i..2 * i + 2].to_vec();
                            if let Some((i1, i2)) = segment_triangle_intersection(
                                &p1,
                                &p2,
                                position,
                                &chunk_intersections[0],
                                &chunk_intersections[1],
                            ) {
                                let projected1 = segment_to_line_intersection(
                                    position,
                                    &i1,
                                    &chunk_intersections[0],
                                    &chunk_intersections[1],
                                );
                                let projected2 = segment_to_line_intersection(
                                    position,
                                    &i2,
                                    &chunk_intersections[0],
                                    &chunk_intersections[1],
                                );
                                let chunk_length = (chunk_intersections[1]
                                    - chunk_intersections[0])
                                    .norm_squared();
                                if is_enabled(InternalLog::EnvironmentDetailed) {
                                    debug!(
                                        "Projected intersections: {:?}, {:?}",
                                        projected1, projected2
                                    );
                                    debug!("Triangle intersection: i1: {:?}, i2: {:?}", i1, i2);
                                }
                                if projected1.is_none() && projected2.is_none() {
                                    // No intersection, not obstructed, keep points
                                    if is_enabled(InternalLog::EnvironmentDetailed) {
                                        debug!(
                                            "No intersection after projection, not obstructed, keeping points."
                                        );
                                    }
                                    new_intersections.extend_from_slice(&chunk_intersections);
                                    continue;
                                }
                                if projected1.is_none()
                                    && let Some(projected2) = projected2
                                {
                                    let dot2 = (projected2 - chunk_intersections[0])
                                        .dot(&(chunk_intersections[1] - chunk_intersections[0]));
                                    if dot2 - 1e-3 < 0. || dot2 + 1e-3 > chunk_length {
                                        // projected2 is out of segment, not obstructed, keep points
                                        if is_enabled(InternalLog::EnvironmentDetailed) {
                                            debug!(
                                                "Projected2 is out of segment, not obstructed, keeping points."
                                            );
                                        }
                                        new_intersections.extend_from_slice(&chunk_intersections);
                                        continue;
                                    }
                                    // partially obstructed, keep second part
                                    if is_enabled(InternalLog::EnvironmentDetailed) {
                                        debug!("Partially obstructed, keeping second part.");
                                    }
                                    new_intersections.push(projected2);
                                    new_intersections.push(chunk_intersections[1]);
                                } else if let Some(projected1) = projected1
                                    && projected2.is_none()
                                {
                                    let dot1 = (projected1 - chunk_intersections[0])
                                        .dot(&(chunk_intersections[1] - chunk_intersections[0]));
                                    if dot1 - 1e-3 < 0. || dot1 + 1e-3 > chunk_length {
                                        // projected1 is out of segment, not obstructed, keep points
                                        if is_enabled(InternalLog::EnvironmentDetailed) {
                                            debug!(
                                                "Projected1 is out of segment, not obstructed, keeping points."
                                            );
                                        }
                                        new_intersections.extend_from_slice(&chunk_intersections);
                                        continue;
                                    }
                                    // partially obstructed, keep first part
                                    if is_enabled(InternalLog::EnvironmentDetailed) {
                                        debug!("Partially obstructed, keeping first part.");
                                    }
                                    new_intersections.push(chunk_intersections[0]);
                                    new_intersections.push(projected1);
                                } else {
                                    let projected1 = projected1.unwrap();
                                    let projected2 = projected2.unwrap();
                                    let dot1 = (projected1 - chunk_intersections[0])
                                        .dot(&(chunk_intersections[1] - chunk_intersections[0]));
                                    let dot2 = (projected2 - chunk_intersections[0])
                                        .dot(&(chunk_intersections[1] - chunk_intersections[0]));
                                    if is_enabled(InternalLog::EnvironmentDetailed) {
                                        debug!(
                                            "Dot products: dot1: {}, dot2: {}, chunk_length: {}",
                                            dot1, dot2, chunk_length
                                        );
                                    }
                                    let inside1 = dot1 - 1e-3 > 0. && dot1 + 1e-3 < chunk_length;
                                    let inside2 = dot2 - 1e-3 > 0. && dot2 + 1e-3 < chunk_length;
                                    if dot1 > dot2 {
                                        chunk_intersections.swap(0, 1);
                                    }

                                    if !inside1 && !inside2 {
                                        if (dot1 < 0. && dot2 < 0.)
                                            || (dot1 > chunk_length && dot2 > chunk_length)
                                        {
                                            // both projected points are out of segment, not obstructed, keep points
                                            if is_enabled(InternalLog::EnvironmentDetailed) {
                                                debug!(
                                                    "Both projected points are out of segment, not obstructed, keeping points."
                                                );
                                            }
                                            new_intersections
                                                .extend_from_slice(&chunk_intersections);
                                            continue;
                                        } else {
                                            if is_enabled(InternalLog::EnvironmentDetailed) {
                                                debug!(
                                                    "Projected points are on different sides, fully obstructed, removing points."
                                                );
                                            }
                                            continue;
                                        }
                                    }
                                    if inside1 && !inside2 {
                                        // partially obstructed, keep first part
                                        if (chunk_intersections[0] - projected1).norm() > 1e-3 {
                                            if is_enabled(InternalLog::EnvironmentDetailed) {
                                                debug!("Partially obstructed, keeping first part.");
                                            }
                                            new_intersections.push(chunk_intersections[0]);
                                            new_intersections.push(projected1);
                                        }
                                        continue;
                                    }
                                    if !inside1 && inside2 {
                                        // partially obstructed, keep second part
                                        if (chunk_intersections[1] - projected2).norm() > 1e-3 {
                                            if is_enabled(InternalLog::EnvironmentDetailed) {
                                                debug!(
                                                    "Partially obstructed, keeping second part."
                                                );
                                            }
                                            new_intersections.push(projected2);
                                            new_intersections.push(chunk_intersections[1]);
                                        }
                                        continue;
                                    }
                                    // both inside, keep external parts
                                    if is_enabled(InternalLog::EnvironmentDetailed) {
                                        debug!("Partially obstructed, keeping external parts.");
                                    }

                                    if (chunk_intersections[0] - projected1).norm() > 1e-3 {
                                        new_intersections.push(chunk_intersections[0]);
                                        new_intersections.push(projected1);
                                    }
                                    if (chunk_intersections[1] - projected2).norm() > 1e-3 {
                                        new_intersections.push(projected2);
                                        new_intersections.push(chunk_intersections[1]);
                                    }
                                }
                            } else {
                                // else: no intersection, not obstructed, keep points
                                new_intersections.extend_from_slice(&chunk_intersections);
                            }
                        }
                        intersections = new_intersections;
                    } else if intersections.len() == 1 && segments_intersection(position, &intersections[0], &p1, &p2).is_some() { 
                        // Try to look at a ponctual landmark, check if segment intersects obstruction
                        // Obstructed, remove point
                        intersections.clear();
                        break;
                    }
                }
                if is_enabled(InternalLog::Environment)
                    || is_enabled(InternalLog::EnvironmentDetailed)
                {
                    debug!(
                        "Intersections after obstruction checks: {:?}",
                        intersections
                    );
                }
                if intersections.is_empty() {
                    // Fully obstructed
                    continue;
                }
            }

            let mut observed_poses = Vec::new();
            let mut observed_width = Vec::new();
            if intersections.len() == 1 {
                observed_poses.push(intersections[0].insert_row(2, landmark.pose.z));
                observed_width.push(0.0);
            } else {
                for i in 0..intersections.len() / 2 {
                    let chunk_intersections = &intersections[2 * i..2 * i + 2];
                    let p1 = chunk_intersections[0];
                    let p2 = chunk_intersections[1];
                    observed_poses.push(Vector3::new(
                        0.5 * (p1.x + p2.x),
                        0.5 * (p1.y + p2.y),
                        landmark.pose.z,
                    ));
                    observed_width.push((p2 - p1).norm());
                }
            }

            for (pose, width) in observed_poses.iter().zip(observed_width.iter()) {
                let obs = OrientedLandmark {
                    id: landmark.id,
                    labels: landmark.labels.clone(),
                    pose: *pose,
                    height: landmark.height,
                    width: *width,
                };
                observed_landmarks.push(obs);
            }
        }
        observed_landmarks
    }

    pub fn is_target_observable(
        &self,
        target_position: &Vector2<f32>,
        target_height: Option<f32>,
        observer_position: &Vector2<f32>,
        observer_height: Option<f32>,
        max_distance: f32,
        cache_key: Option<String>,
    ) -> bool {
        if (target_position - observer_position).norm() > max_distance {
            return false;
        }
        // If target or observer has no height, we consider it observable
        // xray mode for observer
        // and always observable for target if it has no height (e.g. a landmark with height 0)
        if target_height.is_none() || observer_height.is_none() {
            return true;
        }

        let in_range_landmarks =
            self.landmarks_in_range(observer_position, max_distance, cache_key);

        for (possible_obstruction, possible_intersect) in &in_range_landmarks {
            if is_enabled(InternalLog::EnvironmentDetailed) {
                debug!(
                    "Checking obstruction of target by landmark {}",
                    possible_obstruction.id
                );
            }
            if possible_obstruction.height < observer_height.unwrap() {
                if is_enabled(InternalLog::EnvironmentDetailed) {
                    debug!(
                        "Possible obstruction {} is lower than observer, cannot obstruct.",
                        possible_obstruction.id
                    );
                }
                continue;
            }
            if possible_obstruction.height <= 0. {
                if is_enabled(InternalLog::EnvironmentDetailed) {
                    debug!(
                        "Possible obstruction {} has no height, cannot obstruct.",
                        possible_obstruction.id
                    );
                }
                continue;
            }
            // Ponctual landmark cannot obstruct
            if possible_intersect.is_none() {
                continue;
            }
            let (p1, p2) = possible_intersect.unwrap();
            if segments_intersection(observer_position, target_position, &p1, &p2).is_some() {
                if is_enabled(InternalLog::EnvironmentDetailed) {
                    debug!(
                        "Target is obstructed by landmark {}",
                        possible_obstruction.id
                    );
                }
                // Obstructed
                return false;
            }
        }
        true
    }

    pub fn clear_meta_data(&self) {
        self.meta_data_list.write().unwrap().clear();
    }

    pub fn get_meta_data(&self) -> &SharedRwLock<HashMap<String, SharedRoLock<NodeMetaData>>> {
        &self.meta_data_list
    }

    pub fn insert_meta_data(&self, node_name: String, meta_data: SharedRoLock<NodeMetaData>) {
        self.meta_data_list
            .write()
            .unwrap()
            .insert(node_name, meta_data);
    }
}

/// Map, containing multiple [`OrientedLandmark`], used for the map file.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct Map {
    pub landmarks: Vec<OrientedLandmark>,
}

impl Map {
    pub fn new() -> Self {
        Self {
            landmarks: Vec::new(),
        }
    }

    /// Load the map from the given `path`.
    pub fn load_from_path(path: &Path) -> SimbaResult<Map> {
        let map: Map = match confy::load_path(path) {
            Ok(config) => config,
            Err(error) => {
                return Err(SimbaError::new(
                    SimbaErrorTypes::ConfigError,
                    format!(
                        "Error from Confy while loading the map file {} : {}",
                        path.display(),
                        error
                    ),
                ));
            }
        };
        Ok(map)
    }
}
