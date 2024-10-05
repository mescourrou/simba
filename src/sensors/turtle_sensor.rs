/*!
Provides a [`Sensor`] which can observe the other robots in the frame of the ego robot.
*/

use super::sensor::{GenericObservation, Sensor, SensorRecord};

use crate::plugin_api::PluginAPI;
use crate::simulator::{Simulator, SimulatorMetaConfig};
use crate::stateful::Stateful;
use crate::utils::determinist_random_variable::{DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig};
use serde_derive::{Deserialize, Serialize};

use log::error;
extern crate nalgebra as na;
use na::Vector3;

use std::fmt;
use std::path::Path;
use std::sync::{Arc, RwLock};

/// Configuration of the [`TurtleSensor`].
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct TurtleSensorConfig {
    /// Max distance of detection.
    pub detection_distance: f32,
    /// Observation period of the sensor.
    pub period: f32,
    pub x_noise: RandomVariableTypeConfig,
    pub y_noise: RandomVariableTypeConfig,
    pub theta_noise: RandomVariableTypeConfig,
}

impl Default for TurtleSensorConfig {
    fn default() -> Self {
        Self {
            detection_distance: 5.0,
            period: 0.1,
            x_noise: RandomVariableTypeConfig::None,
            y_noise: RandomVariableTypeConfig::None,
            theta_noise: RandomVariableTypeConfig::None,
        }
    }
}

/// Record of the [`TurtleSensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TurtleSensorRecord {
    last_time: f32,
}

impl Default for TurtleSensorRecord {
    fn default() -> Self {
        Self { last_time: 0. }
    }
}

/// Landmark struct, with an `id` and a `pose`, used to read the map file.
#[derive(Debug)]
pub struct OrientedTurtle {
    pub name: String,
    pub pose: Vector3<f32>,
}

use serde::ser::{Serialize, SerializeStruct, Serializer};

impl Serialize for OrientedTurtle {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // 3 is the number of fields in the struct.
        let mut state = serializer.serialize_struct("OrientedTurtle", 4)?;
        state.serialize_field("name", self.name.as_str())?;
        state.serialize_field("x", &self.pose.x)?;
        state.serialize_field("y", &self.pose.y)?;
        state.serialize_field("theta", &self.pose.z)?;
        state.end()
    }
}

use serde::de::{self, Deserialize, Deserializer, MapAccess, SeqAccess, Visitor};

impl<'de> Deserialize<'de> for OrientedTurtle {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        enum Field {
            Name,
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
                        formatter.write_str("`name` or `x` or `y` or `theta`")
                    }

                    fn visit_str<E>(self, value: &str) -> Result<Field, E>
                    where
                        E: de::Error,
                    {
                        match value {
                            "name" => Ok(Field::Name),
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

        struct OrientedTurtleVisitor;

        impl<'de> Visitor<'de> for OrientedTurtleVisitor {
            type Value = OrientedTurtle;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct OrientedTurtle")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<OrientedTurtle, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let name: &str = seq
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
                Ok(OrientedTurtle {
                    name: name.to_string(),
                    pose: Vector3::from_vec(vec![x, y, theta]),
                })
            }

            fn visit_map<V>(self, mut map: V) -> Result<OrientedTurtle, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut name = None;
                let mut x = None;
                let mut y = None;
                let mut theta = None;

                while let Some(key) = map.next_key()? {
                    match key {
                        Field::Name => {
                            if name.is_some() {
                                return Err(de::Error::duplicate_field("name"));
                            }
                            name = Some(map.next_value()?);
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
                let name = name.ok_or_else(|| de::Error::missing_field("name"))?;
                let x = x.ok_or_else(|| de::Error::missing_field("x"))?;
                let y = y.ok_or_else(|| de::Error::missing_field("y"))?;
                let theta = theta.ok_or_else(|| de::Error::missing_field("theta"))?;
                Ok(OrientedTurtle {
                    name,
                    pose: Vector3::from_vec(vec![x, y, theta]),
                })
            }
        }

        const FIELDS: &'static [&'static str] = &["name", "x", "y", "theta"];
        deserializer.deserialize_struct("OrientedTurtle", FIELDS, OrientedTurtleVisitor)
    }
}

/// Observation of an [`OrientedTurtle`].
#[derive(Debug)]
pub struct OrientedTurtleObservation {
    /// Name of the turtle
    pub name: String,
    /// Pose of the turtle
    pub pose: Vector3<f32>,
}

impl GenericObservation for OrientedTurtleObservation {}

/// Sensor which observe the other turtles.
#[derive(Debug)]
pub struct TurtleSensor {
    /// Detection distance
    detection_distance: f32,
    /// Observation period
    period: f32,
    /// Last observation time.
    last_time: f32,
    gen_x: Box<dyn DeterministRandomVariable>,
    gen_y: Box<dyn DeterministRandomVariable>,
    gen_theta: Box<dyn DeterministRandomVariable>,
}

impl TurtleSensor {
    /// Makes a new [`TurtleSensor`].
    pub fn new() -> Self {
        TurtleSensor::from_config(
            &TurtleSensorConfig::default(),
            &None,
            SimulatorMetaConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Makes a new [`TurtleSensor`] from the given config.
    ///
    /// The map path is relative to the config path of the simulator.
    pub fn from_config(
        config: &TurtleSensorConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        meta_config: SimulatorMetaConfig,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        Self {
            detection_distance: config.detection_distance,
            period: config.period,
            last_time: 0.,
            gen_x: va_factory.make_variable(config.x_noise.clone()),
            gen_y: va_factory.make_variable(config.y_noise.clone()),
            gen_theta: va_factory.make_variable(config.theta_noise.clone()),
        }
    }

}

use crate::turtlebot::Turtlebot;

impl Sensor for TurtleSensor {
    fn init(&mut self, _turtle: &mut Turtlebot) {}

    fn get_observations(
        &mut self,
        turtle: &mut Turtlebot,
        time: f32,
        turtle_list: &Arc<RwLock<Vec<Arc<RwLock<Turtlebot>>>>>,
    ) -> Vec<Box<dyn GenericObservation>> {
        let arc_physic = turtle.physics();
        let physic = arc_physic.read().unwrap();
        let mut observation_list = Vec::<Box<dyn GenericObservation>>::new();
        if time < self.next_time_step() {
            return observation_list;
        }
        let state = physic.state(time);

        let rotation_matrix =
            nalgebra::geometry::Rotation3::from_euler_angles(0., 0., state.pose.z);

        let turtle_unlock_list = turtle_list.read().unwrap();
        let mut i = 0;
        for other_turtle in turtle_unlock_list.iter() {
            i+= 1;
            let turtle_unlocked = other_turtle.read().unwrap();
            if turtle_unlocked.name() == turtle.name() {
                continue;
            }
            let other_arc_physic = turtle_unlocked.physics();
            let other_physic = other_arc_physic.read().unwrap();
            let other_state = other_physic.state(time);
            let d = ((other_state.pose.x - state.pose.x).powi(2)
                + (other_state.pose.y - state.pose.y).powi(2))
            .sqrt();
            if d <= self.detection_distance {
                let turtle_seed = 1./(100.*self.period)*(i as f32);
                let noisy_pose = na::Vector3::<f32>::from_vec(vec![
                    self.gen_x.gen(time + turtle_seed),
                    self.gen_y.gen(time + turtle_seed),
                    self.gen_theta.gen(time + turtle_seed)
                ]);
                observation_list.push(Box::new(OrientedTurtleObservation {
                    name: turtle_unlocked.name(),
                    pose: rotation_matrix * other_state.pose + state.pose + noisy_pose,
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

impl Stateful<SensorRecord> for TurtleSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::TurtleSensor(TurtleSensorRecord {
            last_time: self.last_time,
        })
    }

    fn from_record(&mut self, record: SensorRecord) {
        if let SensorRecord::TurtleSensor(turtle_sensor_record) = record {
            self.last_time = turtle_sensor_record.last_time;
        }
    }
}
