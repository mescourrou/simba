/*!
Provides a [`Sensor`] which can observe oriented landmarks in the frame of the robot.
*/

use super::fault_models::fault_model::{
    make_fault_model_from_config, FaultModel, FaultModelConfig,
};
use super::sensor::{Sensor, SensorObservation, SensorRecord};

use crate::constants::TIME_ROUND;
#[cfg(feature = "gui")]
use crate::gui::{utils::path_finder, UIComponent};
use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::state_estimator::State;
use crate::recordable::Recordable;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::maths::round_precision;
use config_checker::macros::Check;
use serde_derive::{Deserialize, Serialize};

use log::error;
extern crate nalgebra as na;
use na::Vector3;

use std::fmt;
use std::path::Path;
use std::sync::{Arc, Mutex};

/// Configuration of the [`OrientedLandmarkSensor`].
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct OrientedLandmarkSensorConfig {
    /// Max distance of detection.
    #[check[ge(0.)]]
    pub detection_distance: f32,
    /// Path to the map (with real position of the landmarks), relative to the simulator
    /// config path.
    pub map_path: String,
    /// Observation period of the sensor.
    #[check[ge(0.)]]
    pub period: f32,
    #[check]
    pub faults: Vec<FaultModelConfig>,
}

impl Default for OrientedLandmarkSensorConfig {
    fn default() -> Self {
        Self {
            detection_distance: 5.0,
            map_path: String::from(""),
            period: 0.1,
            faults: Vec::new(),
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
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new("Oriented Landmark sensor")
            .id_source(format!("oriented-landmark-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Detection distance:");
                    if self.detection_distance < 0. {
                        self.detection_distance = 0.;
                    }
                    ui.add(egui::DragValue::new(&mut self.detection_distance));
                });

                ui.horizontal(|ui| {
                    ui.label("Period:");
                    if self.period < TIME_ROUND {
                        self.period = TIME_ROUND;
                    }
                    ui.add(
                        egui::DragValue::new(&mut self.period)
                            .max_decimals((1. / TIME_ROUND) as usize),
                    );
                });

                ui.horizontal(|ui| {
                    ui.label("Map path:");
                    path_finder(ui, &mut self.map_path, &global_config.base_path);
                });

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

    fn show(
        &self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new("Oriented Landmark sensor")
            .id_source(format!("oriented-landmark-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(format!("Detection distance: {}", self.detection_distance));
                });

                ui.horizontal(|ui| {
                    ui.label(format!("Period: {}", self.period));
                });

                ui.horizontal(|ui| {
                    ui.label(format!("Map path: {}", self.map_path));
                });

                FaultModelConfig::show_faults(
                    &self.faults,
                    ui,
                    ctx,
                    unique_id,
                );
            });
    }
}

/// Record of the [`OrientedLandmarkSensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct OrientedLandmarkSensorRecord {
    last_time: f32,
}

impl Default for OrientedLandmarkSensorRecord {
    fn default() -> Self {
        Self { last_time: 0. }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for OrientedLandmarkSensorRecord {
    fn show(
            &self,
            ui: &mut egui::Ui,
            ctx: &egui::Context,
            unique_id: &String,
        ) {
        ui.label(format!("Last time: {}", self.last_time));
    }
}

/// Landmark struct, with an `id` and a `pose`, used to read the map file.
#[derive(Debug, Clone)]
pub struct OrientedLandmark {
    pub id: i32,
    pub pose: Vector3<f32>,
}

use serde::ser::{Serialize, SerializeStruct, Serializer};

impl Serialize for OrientedLandmark {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // 3 is the number of fields in the struct.
        let mut state = serializer.serialize_struct("OrientedLandmark", 4)?;
        state.serialize_field("id", &self.id)?;
        state.serialize_field("x", &self.pose.x)?;
        state.serialize_field("y", &self.pose.y)?;
        state.serialize_field("theta", &self.pose.z)?;
        state.end()
    }
}

use serde::de::{self, Deserialize, Deserializer, MapAccess, SeqAccess, Visitor};

impl<'de> Deserialize<'de> for OrientedLandmark {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        enum Field {
            Id,
            X,
            Y,
            Theta,
            Unknown,
        }

        // This part could also be generated independently by:
        //
        //    #[derive(Deserialize)]
        //    #[serde(field_identifier, rename_all = "lowercase")]
        //    enum Field { Secs, Nanos }
        impl<'de> Deserialize<'de> for Field {
            fn deserialize<D>(deserializer: D) -> Result<Field, D::Error>
            where
                D: Deserializer<'de>,
            {
                struct FieldVisitor;

                impl<'de> Visitor<'de> for FieldVisitor {
                    type Value = Field;

                    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                        formatter.write_str("`id` or `x` or `y` or `theta`")
                    }

                    fn visit_str<E>(self, value: &str) -> Result<Field, E>
                    where
                        E: de::Error,
                    {
                        match value {
                            "id" => Ok(Field::Id),
                            "x" => Ok(Field::X),
                            "y" => Ok(Field::Y),
                            "theta" => Ok(Field::Theta),
                            _ => Ok(Field::Unknown),
                        }
                    }
                }

                deserializer.deserialize_identifier(FieldVisitor)
            }
        }

        struct OrientedLandmarkVisitor;

        impl<'de> Visitor<'de> for OrientedLandmarkVisitor {
            type Value = OrientedLandmark;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct OrientedLandmark")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<OrientedLandmark, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let id: i32 = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                let x: f32 = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(1, &self))?;
                let y: f32 = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(2, &self))?;
                let theta: f32 = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(3, &self))?;
                Ok(OrientedLandmark {
                    id: id,
                    pose: Vector3::from_vec(vec![x, y, theta]),
                })
            }

            fn visit_map<V>(self, mut map: V) -> Result<OrientedLandmark, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut id = None;
                let mut x = None;
                let mut y = None;
                let mut theta = None;

                while let Some(key) = map.next_key()? {
                    match key {
                        Field::Id => {
                            if id.is_some() {
                                return Err(de::Error::duplicate_field("id"));
                            }
                            id = Some(map.next_value()?);
                        }
                        Field::X => {
                            if x.is_some() {
                                return Err(de::Error::duplicate_field("x"));
                            }
                            x = Some(map.next_value()?);
                        }
                        Field::Y => {
                            if y.is_some() {
                                return Err(de::Error::duplicate_field("y"));
                            }
                            y = Some(map.next_value()?);
                        }
                        Field::Theta => {
                            if theta.is_some() {
                                return Err(de::Error::duplicate_field("theta"));
                            }
                            theta = Some(map.next_value()?);
                        }
                        Field::Unknown => {}
                    }
                }
                let id = id.ok_or_else(|| de::Error::missing_field("id"))?;
                let x = x.ok_or_else(|| de::Error::missing_field("x"))?;
                let y = y.ok_or_else(|| de::Error::missing_field("y"))?;
                let theta = theta.ok_or_else(|| de::Error::missing_field("theta"))?;
                Ok(OrientedLandmark {
                    id: id,
                    pose: Vector3::from_vec(vec![x, y, theta]),
                })
            }
        }

        const FIELDS: &'static [&'static str] = &["id", "x", "y", "theta"];
        deserializer.deserialize_struct("OrientedLandmark", FIELDS, OrientedLandmarkVisitor)
    }
}

/// Observation of an [`OrientedLandmark`].
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct OrientedLandmarkObservation {
    /// Id of the landmark
    pub id: i32,
    /// Pose of the landmark
    pub pose: Vector3<f32>,
}

impl Recordable<OrientedLandmarkObservationRecord> for OrientedLandmarkObservation {
    fn record(&self) -> OrientedLandmarkObservationRecord {
        OrientedLandmarkObservationRecord {
            id: self.id,
            pose: self.pose.into(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct OrientedLandmarkObservationRecord {
    /// Id of the landmark
    pub id: i32,
    /// Pose of the landmark
    pub pose: [f32; 3],
}

#[cfg(feature = "gui")]
impl UIComponent for OrientedLandmarkObservationRecord {
    fn show(
            &self,
            ui: &mut egui::Ui,
            ctx: &egui::Context,
            unique_id: &String,
        ) {
        ui.vertical(|ui| {
            ui.label(format!("Id: {}", self.id));
            ui.label(format!("Pose: ({}, {}, {})", self.pose[0], self.pose[1], self.pose[2]));
        });
    }
}

/// Map, containing multiple [`OrientedLandmark`], used for the map file.
#[derive(Serialize, Deserialize, Debug, Default)]
pub struct Map {
    pub landmarks: Vec<OrientedLandmark>,
}

/// Sensor which observe the map landmarks.
#[derive(Debug)]
pub struct OrientedLandmarkSensor {
    /// Detection distance
    detection_distance: f32,
    /// Landmarks list.
    landmarks: Vec<OrientedLandmark>,
    /// Observation period
    period: f32,
    /// Last observation time.
    last_time: f32,
    faults: Arc<Mutex<Vec<Box<dyn FaultModel>>>>,
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
        )
    }

    /// Makes a new [`OrientedLandmarkSensor`] from the given config.
    ///
    /// The map path is relative to the config path of the simulator.
    pub fn from_config(
        config: &OrientedLandmarkSensorConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        robot_name: &String,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        let mut path = Path::new(&config.map_path);
        let fault_models = Arc::new(Mutex::new(Vec::new()));
        let mut unlock_fault_model = fault_models.lock().unwrap();
        for fault_config in &config.faults {
            unlock_fault_model.push(make_fault_model_from_config(
                fault_config,
                global_config,
                robot_name,
                va_factory,
            ));
        }
        drop(unlock_fault_model);
        let mut sensor = Self {
            detection_distance: config.detection_distance,
            landmarks: Vec::new(),
            period: config.period,
            last_time: 0.,
            faults: fault_models,
        };

        if config.map_path == "" {
            return sensor;
        }
        let joined_path = global_config.base_path.join(&config.map_path);
        if path.is_relative() {
            path = joined_path.as_path();
        }
        sensor.landmarks = Self::load_map_from_path(&path);

        sensor
    }

    /// Load the map from the given `path`.
    pub fn load_map_from_path(path: &Path) -> Vec<OrientedLandmark> {
        let map: Map = match confy::load_path(&path) {
            Ok(config) => config,
            Err(error) => {
                error!(
                    "Error from Confy while loading the map file {} : {}",
                    path.display(),
                    error
                );
                return Vec::new();
            }
        };
        map.landmarks
    }
}

use crate::node::Node;

impl Sensor for OrientedLandmarkSensor {
    fn init(&mut self, _robot: &mut Node) {}

    fn get_observations(&mut self, robot: &mut Node, time: f32) -> Vec<SensorObservation> {
        let mut observation_list = Vec::<SensorObservation>::new();
        if (time - self.next_time_step()).abs() >= TIME_ROUND {
            return observation_list;
        }
        let state = if let Some(arc_physic) = robot.physics() {
            let physic = arc_physic.read().unwrap();
            physic.state(time).clone()
        } else {
            State::new() // 0
        };

        let rotation_matrix =
            nalgebra::geometry::Rotation3::from_euler_angles(0., 0., state.pose.z);

        for landmark in &self.landmarks {
            let d = ((landmark.pose.x - state.pose.x).powi(2)
                + (landmark.pose.y - state.pose.y).powi(2))
            .sqrt();
            if d <= self.detection_distance {
                let landmark_seed = 1. / (100. * self.period) * (landmark.id as f32);
                let pose = rotation_matrix.transpose() * (landmark.pose - state.pose);
                observation_list.push(SensorObservation::OrientedLandmark(
                    OrientedLandmarkObservation {
                        id: landmark.id,
                        pose,
                    },
                ));
                for fault_model in self.faults.lock().unwrap().iter() {
                    fault_model.add_faults(
                        time + landmark_seed,
                        self.period,
                        &mut observation_list,
                        SensorObservation::OrientedLandmark(OrientedLandmarkObservation::default()),
                    );
                }
            }
        }
        self.last_time = time;
        observation_list
    }

    /// Get the next observation time.
    fn next_time_step(&self) -> f32 {
        round_precision(self.last_time + self.period, TIME_ROUND).unwrap()
    }

    /// Get the observation period.
    fn period(&self) -> f32 {
        self.period
    }
}

impl Recordable<SensorRecord> for OrientedLandmarkSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::OrientedLandmarkSensor(OrientedLandmarkSensorRecord {
            last_time: self.last_time,
        })
    }
}
