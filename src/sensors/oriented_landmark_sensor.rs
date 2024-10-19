/*!
Provides a [`Sensor`] which can observe oriented landmarks in the frame of the robot.
*/

use super::sensor::{Observation, Sensor, SensorRecord};

use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorMetaConfig;
use crate::stateful::Stateful;
use crate::utils::determinist_random_variable::{
    DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig,
};
use serde_derive::{Deserialize, Serialize};

use log::error;
extern crate nalgebra as na;
use na::Vector3;

use std::fmt;
use std::path::Path;
use std::sync::{Arc, RwLock};

/// Configuration of the [`OrientedLandmarkSensor`].
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct OrientedLandmarkSensorConfig {
    /// Max distance of detection.
    pub detection_distance: f32,
    /// Path to the map (with real position of the landmarks), relative to the simulator
    /// config path.
    pub map_path: String,
    /// Observation period of the sensor.
    pub period: f32,
    pub x_noise: RandomVariableTypeConfig,
    pub y_noise: RandomVariableTypeConfig,
    pub theta_noise: RandomVariableTypeConfig,
}

impl Default for OrientedLandmarkSensorConfig {
    fn default() -> Self {
        Self {
            detection_distance: 5.0,
            map_path: String::from(""),
            period: 0.1,
            x_noise: RandomVariableTypeConfig::None,
            y_noise: RandomVariableTypeConfig::None,
            theta_noise: RandomVariableTypeConfig::None,
        }
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

/// Landmark struct, with an `id` and a `pose`, used to read the map file.
#[derive(Debug)]
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
#[derive(Debug)]
pub struct OrientedLandmarkObservation {
    /// Id of the landmark
    pub id: i32,
    /// Pose of the landmark
    pub pose: Vector3<f32>,
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
    gen_x: Box<dyn DeterministRandomVariable>,
    gen_y: Box<dyn DeterministRandomVariable>,
    gen_theta: Box<dyn DeterministRandomVariable>,
}

impl OrientedLandmarkSensor {
    /// Makes a new [`OrientedLandmarkSensor`].
    pub fn new() -> Self {
        OrientedLandmarkSensor::from_config(
            &OrientedLandmarkSensorConfig::default(),
            &None,
            SimulatorMetaConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Makes a new [`OrientedLandmarkSensor`] from the given config.
    ///
    /// The map path is relative to the config path of the simulator.
    pub fn from_config(
        config: &OrientedLandmarkSensorConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        meta_config: SimulatorMetaConfig,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        let mut path = Path::new(&config.map_path);

        let mut sensor = Self {
            detection_distance: config.detection_distance,
            landmarks: Vec::new(),
            period: config.period,
            last_time: 0.,
            gen_x: va_factory.make_variable(config.x_noise.clone()),
            gen_y: va_factory.make_variable(config.y_noise.clone()),
            gen_theta: va_factory.make_variable(config.theta_noise.clone()),
        };

        if config.map_path == "" {
            return sensor;
        }
        let joined_path = meta_config
            .config_path
            .unwrap()
            .parent()
            .unwrap_or(Path::new("."))
            .join(path);
        if path.is_relative() {
            path = joined_path.as_path();
        }
        sensor.landmarks = Self::load_map_from_path(&path);

        sensor
    }

    /// Load the map from the given `path`.
    fn load_map_from_path(path: &Path) -> Vec<OrientedLandmark> {
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

use crate::turtlebot::Turtlebot;

impl Sensor for OrientedLandmarkSensor {
    fn init(
        &mut self,
        _turtle: &mut Turtlebot,
        _turtle_list: &Arc<RwLock<Vec<Arc<RwLock<Turtlebot>>>>>,
        _turtle_idx: usize,
    ) {
    }

    fn get_observations(&mut self, turtle: &mut Turtlebot, time: f32) -> Vec<Observation> {
        let arc_physic = turtle.physics();
        let physic = arc_physic.read().unwrap();
        let mut observation_list = Vec::<Observation>::new();
        if time < self.next_time_step() {
            return observation_list;
        }
        let state = physic.state(time);

        let rotation_matrix =
            nalgebra::geometry::Rotation3::from_euler_angles(0., 0., state.pose.z);

        for landmark in &self.landmarks {
            let d = ((landmark.pose.x - state.pose.x).powi(2)
                + (landmark.pose.y - state.pose.y).powi(2))
            .sqrt();
            if d <= self.detection_distance {
                let landmark_seed = 1. / (100. * self.period) * (landmark.id as f32);
                let noisy_pose = na::Vector3::<f32>::from_vec(vec![
                    self.gen_x.gen(time + landmark_seed),
                    self.gen_y.gen(time + landmark_seed),
                    self.gen_theta.gen(time + landmark_seed),
                ]);
                observation_list.push(Observation::OrientedLandmark(OrientedLandmarkObservation {
                    id: landmark.id,
                    pose: rotation_matrix * landmark.pose + state.pose + noisy_pose,
                }));
            }
        }
        self.last_time = time;
        observation_list
    }

    /// Get the next observation time.
    fn next_time_step(&self) -> f32 {
        self.last_time + self.period
    }

    /// Get the observation period.
    fn period(&self) -> f32 {
        self.period
    }
}

impl Stateful<SensorRecord> for OrientedLandmarkSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::OrientedLandmarkSensor(OrientedLandmarkSensorRecord {
            last_time: self.last_time,
        })
    }

    fn from_record(&mut self, record: SensorRecord) {
        if let SensorRecord::OrientedLandmarkSensor(oriented_landmark_record) = record {
            self.last_time = oriented_landmark_record.last_time;
        }
    }
}
