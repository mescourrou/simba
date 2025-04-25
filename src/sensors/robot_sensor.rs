/*!
Provides a [`Sensor`] which can observe the other nodes in the frame of the ego node.
*/

use super::fault_models::fault_model::{FaultModel, FaultModelConfig};
use super::sensor::{Observation, Sensor, SensorRecord};

use crate::constants::TIME_ROUND;
use crate::networking::network::MessageFlag;
use crate::networking::service::ServiceClient;
use crate::networking::service_manager;
use crate::physics::physic::{GetRealStateReq, GetRealStateResp};
use crate::plugin_api::PluginAPI;
use crate::sensors::fault_models::fault_model::make_fault_model_from_config;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::state_estimator::State;
use crate::stateful::Stateful;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::maths::round_precision;
use config_checker::macros::Check;
use serde_derive::{Deserialize, Serialize};

use log::debug;
extern crate nalgebra as na;
use na::Vector3;

use std::collections::BTreeMap;
use std::fmt;
use std::sync::{Arc, Mutex, RwLock};

/// Configuration of the [`RobotSensor`].
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct RobotSensorConfig {
    /// Max distance of detection.
    #[check[ge(0.)]]
    pub detection_distance: f32,
    /// Observation period of the sensor.
    #[check[ge(0.)]]
    pub period: f32,
    #[check]
    pub faults: Vec<FaultModelConfig>,
}

impl Default for RobotSensorConfig {
    fn default() -> Self {
        Self {
            detection_distance: 5.0,
            period: 0.1,
            faults: Vec::new(),
        }
    }
}

/// Record of the [`RobotSensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RobotSensorRecord {
    last_time: f32,
}

impl Default for RobotSensorRecord {
    fn default() -> Self {
        Self { last_time: 0. }
    }
}

/// Landmark struct, with an `id` and a `pose`, used to read the map file.
#[derive(Debug)]
pub struct OrientedRobot {
    pub name: String,
    pub pose: Vector3<f32>,
}

use serde::ser::{Serialize, SerializeStruct, Serializer};

impl Serialize for OrientedRobot {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // 3 is the number of fields in the struct.
        let mut state = serializer.serialize_struct("OrientedRobot", 4)?;
        state.serialize_field("name", self.name.as_str())?;
        state.serialize_field("x", &self.pose.x)?;
        state.serialize_field("y", &self.pose.y)?;
        state.serialize_field("theta", &self.pose.z)?;
        state.end()
    }
}

use serde::de::{self, Deserialize, Deserializer, MapAccess, SeqAccess, Visitor};

impl<'de> Deserialize<'de> for OrientedRobot {
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

        struct OrientedRobotVisitor;

        impl<'de> Visitor<'de> for OrientedRobotVisitor {
            type Value = OrientedRobot;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct OrientedRobot")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<OrientedRobot, V::Error>
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
                Ok(OrientedRobot {
                    name: name.to_string(),
                    pose: Vector3::from_vec(vec![x, y, theta]),
                })
            }

            fn visit_map<V>(self, mut map: V) -> Result<OrientedRobot, V::Error>
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
                Ok(OrientedRobot {
                    name,
                    pose: Vector3::from_vec(vec![x, y, theta]),
                })
            }
        }

        const FIELDS: &'static [&'static str] = &["name", "x", "y", "theta"];
        deserializer.deserialize_struct("OrientedRobot", FIELDS, OrientedRobotVisitor)
    }
}

/// Observation of an [`OrientedRobot`].
#[derive(Debug, Default, Clone)]
pub struct OrientedRobotObservation {
    /// Name of the Robot
    pub name: String,
    /// Pose of the Robot
    pub pose: Vector3<f32>,
}

impl Stateful<OrientedRobotObservationRecord> for OrientedRobotObservation {
    fn record(&self) -> OrientedRobotObservationRecord {
        OrientedRobotObservationRecord {
            name: self.name.clone(),
            pose: self.pose.to_owned().into(),
        }
    }

    fn from_record(&mut self, record: OrientedRobotObservationRecord) {
        self.name = record.name;
        self.pose = Vector3::from(record.pose);
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct OrientedRobotObservationRecord {
    /// Name of the Robot
    pub name: String,
    /// Pose of the Robot
    pub pose: [f32; 3],
}

/// Sensor which observe the other Robots.
#[derive(Debug)]
pub struct RobotSensor {
    /// Detection distance
    detection_distance: f32,
    /// Observation period
    period: f32,
    /// Last observation time.
    last_time: f32,
    faults: Arc<Mutex<Vec<Box<dyn FaultModel>>>>,
}

impl RobotSensor {
    /// Makes a new [`RobotSensor`].
    pub fn new() -> Self {
        RobotSensor::from_config(
            &RobotSensorConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &"NoName".to_string(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Makes a new [`RobotSensor`] from the given config.
    ///
    /// The map path is relative to the config path of the simulator.
    pub fn from_config(
        config: &RobotSensorConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        node_name: &String,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        assert!(config.period != 0.);
        let fault_models = Arc::new(Mutex::new(Vec::new()));
        let mut unlock_fault_model = fault_models.lock().unwrap();
        for fault_config in &config.faults {
            unlock_fault_model.push(make_fault_model_from_config(
                fault_config,
                global_config,
                node_name,
                va_factory,
            ));
        }
        drop(unlock_fault_model);
        Self {
            detection_distance: config.detection_distance,
            period: config.period,
            last_time: 0.,
            faults: fault_models,
        }
    }
}

use crate::node::Node;

impl Sensor for RobotSensor {
    fn init(&mut self, _node: &mut Node) {}

    fn get_observations(&mut self, node: &mut Node, time: f32) -> Vec<Observation> {
        let mut observation_list = Vec::<Observation>::new();
        if (time - self.next_time_step()).abs() > TIME_ROUND / 2. {
            return observation_list;
        }
        debug!("Start looking for nodes");
        let state = if let Some(arc_physic) = node.physics() {
            let physic = arc_physic.read().unwrap();
            physic.state(time).clone()
        } else {
            State::new() // 0
        };

        let rotation_matrix =
            nalgebra::geometry::Rotation3::from_euler_angles(0., 0., state.pose.z);
        debug!("Rotation matrix: {}", rotation_matrix);

        for other_node_name in node.other_node_names.iter() {
            debug!("Sensing node {}", other_node_name);
            assert!(*other_node_name != node.name());

            let service_manager = node.service_manager();
            if let Some(other_state) = service_manager.read().unwrap().get_real_state(
                other_node_name.to_string(),
                node,
                time,
            ) {
                let d = ((other_state.pose.x - state.pose.x).powi(2)
                    + (other_state.pose.y - state.pose.y).powi(2))
                .sqrt();
                debug!("Distance is {d}");
                if d <= self.detection_distance {
                    observation_list.push(Observation::OrientedRobot(OrientedRobotObservation {
                        name: other_node_name.clone(),
                        pose: rotation_matrix.transpose() * (other_state.pose - state.pose),
                    }));
                }
            };
        }
        for fault_model in self.faults.lock().unwrap().iter() {
            fault_model.add_faults(
                time,
                self.period,
                &mut observation_list,
                Observation::OrientedRobot(OrientedRobotObservation::default()),
            );
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

impl Stateful<SensorRecord> for RobotSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::RobotSensor(RobotSensorRecord {
            last_time: self.last_time,
        })
    }

    fn from_record(&mut self, record: SensorRecord) {
        if let SensorRecord::RobotSensor(node_sensor_record) = record {
            self.last_time = node_sensor_record.last_time;
        }
    }
}
