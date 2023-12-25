use super::sensor::{Sensor, SensorRecord, GenericObservation};
use crate::physics::physic::Physic;

use serde_derive::{Serialize, Deserialize};

extern crate nalgebra as na;
use na::{Vector3};

use std::path::Path;
use std::fmt;

#[derive(Serialize, Deserialize, Debug)]
#[serde(default)]
pub struct OrientedLandmarkSensorConfig {
    detection_distance: f32,
    map_path: String,
    period: f32
}

impl Default for OrientedLandmarkSensorConfig {
    fn default() -> Self {
        Self {
            detection_distance: 5.0,
            map_path: String::from(""),
            period: 0.1
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct OrientedLandmarkSensorRecord {
    
}

impl Default for OrientedLandmarkSensorRecord {
    fn default() -> Self {
        Self {
        }
    }
}

#[derive(Debug)]
pub struct OrientedLandmark {
    pub id: i32,
    pub pose: Vector3<f32>
}


use serde::ser::{Serialize, Serializer, SerializeStruct};

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

use serde::de::{self, Deserialize, Deserializer, Visitor, SeqAccess, MapAccess};

impl<'de> Deserialize<'de> for OrientedLandmark {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        enum Field { Id, X, Y, Theta, Unknown }

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
                let id: i32 = seq.next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                let x: f32 = seq.next_element()?
                    .ok_or_else(|| de::Error::invalid_length(1, &self))?;
                let y: f32 = seq.next_element()?
                    .ok_or_else(|| de::Error::invalid_length(2, &self))?;
                let theta: f32 = seq.next_element()?
                    .ok_or_else(|| de::Error::invalid_length(3, &self))?;
                Ok(OrientedLandmark {
                    id: id,
                    pose: Vector3::from_vec(vec![x, y, theta])
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
                    pose: Vector3::from_vec(vec![x, y, theta])
                })
            }
        }

        const FIELDS: &'static [&'static str] = &["id", "x", "y", "theta"];
        deserializer.deserialize_struct("OrientedLandmark", FIELDS, OrientedLandmarkVisitor)
    }
}

#[derive(Debug)]
pub struct OrientedLandmarkObservation {
    id: i32,
    pose: Vector3<f32>
}

impl GenericObservation for OrientedLandmarkObservation {}

#[derive(Serialize, Deserialize, Debug, Default)]
pub struct Map {
    pub landmarks: Vec<OrientedLandmark>
}

#[derive(Debug)]
pub struct OrientedLandmarkSensor {
    detection_distance: f32,
    landmarks: Vec<OrientedLandmark>,
    period: f32,
    last_time: f32
}

impl OrientedLandmarkSensor {
    pub fn new() -> Self {
        OrientedLandmarkSensor::from_config(&OrientedLandmarkSensorConfig::default())
    }

    pub fn from_config(config: &OrientedLandmarkSensorConfig) -> Self {
        let mut path = Path::new(&config.map_path);

        let mut sensor = Self {
            detection_distance: config.detection_distance,
            landmarks: Vec::new(),
            period: config.period,
            last_time: -1.
        };

        if config.map_path == "" {
            return sensor;
        }
        let joined_path = Path::new("./configs").join(path);
        if path.is_relative() {
            path = joined_path.as_path();
        }
        sensor.landmarks = Self::load_map_from_path(&path);

        sensor

    }

    fn load_map_from_path(path: &Path) -> Vec<OrientedLandmark> {
        let map: Map =  match confy::load_path(&path) {
            Ok(config) => config,
            Err(error) => {
                println!("Error from Confy while loading the map file {} : {}", path.display(), error);
                return Vec::new();
            }
        };
        map.landmarks
    }
}

impl Sensor for OrientedLandmarkSensor {
    fn get_observations(&mut self, physic: &dyn Physic, time: f32) -> Vec<Box<dyn GenericObservation>> {
        let mut observation_list = Vec::<Box<dyn GenericObservation>>::new();
        if time < self.next_time_step() {
            return observation_list;
        }
        let state  = physic.state(time);

        let rotation_matrix = nalgebra::geometry::Rotation3::from_euler_angles(0., 0., state.pose.z);
        println!("Rotation matrix: {}", rotation_matrix);

        for landmark in &self.landmarks {
            let d = ((landmark.pose.x - state.pose.x).powf(2.) + (landmark.pose.y - state.pose.y).powf(2.)).sqrt();
            if d <= self.detection_distance {
                observation_list.push(Box::new(OrientedLandmarkObservation {
                    id: landmark.id,
                    pose: rotation_matrix * landmark.pose + state.pose
                }));
            }
        }
        self.last_time = time;
        observation_list
    }

    fn record(&self) ->  SensorRecord {
        SensorRecord::OrientedLandmarkSensor(OrientedLandmarkSensorRecord {

        })
    }

    fn next_time_step(&self) -> f32 {
        self.last_time + self.period
    }

    fn period(&self) -> f32 {
        self.period
    }
}