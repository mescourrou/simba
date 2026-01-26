/*!
Provides a [`Sensor`] which can observe the other nodes in the frame of the ego node.
*/

use super::fault_models::fault_model::{FaultModel, FaultModelConfig};
use super::{Sensor, SensorObservation, SensorRecord};

use crate::constants::TIME_ROUND;

use crate::errors::SimbaErrorTypes;
#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::logger::is_enabled;
use crate::networking::service_manager::ServiceError;
use crate::plugin_api::PluginAPI;
use crate::recordable::Recordable;
use crate::sensors::fault_models::fault_model::make_fault_model_from_config;
use crate::sensors::sensor_filters::{
    SensorFilter, SensorFilterConfig, make_sensor_filter_from_config,
};
use crate::simulator::SimulatorConfig;
use crate::state_estimators::State;
use crate::utils::SharedMutex;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::maths::round_precision;
use serde_derive::{Deserialize, Serialize};

use log::debug;
extern crate nalgebra as na;
use na::Vector3;
use simba_macros::config_derives;

use std::fmt;
use std::sync::{Arc, Mutex};

/// Configuration of the [`RobotSensor`].
#[config_derives]
pub struct RobotSensorConfig {
    /// Max distance of detection.
    #[check[ge(0.)]]
    pub detection_distance: f32,
    /// Observation period of the sensor.
    pub period: Option<f32>,
    #[check]
    pub faults: Vec<FaultModelConfig>,
    #[check]
    pub filters: Vec<SensorFilterConfig>,
}

impl Default for RobotSensorConfig {
    fn default() -> Self {
        Self {
            detection_distance: 5.0,
            period: Some(0.1),
            faults: Vec::new(),
            filters: Vec::new(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for RobotSensorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Robot sensor")
            .id_salt(format!("robot-sensor-{}", unique_id))
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
                    if let Some(p) = &mut self.period {
                        if *p < TIME_ROUND {
                            *p = TIME_ROUND;
                        }
                        ui.add(egui::DragValue::new(p).max_decimals((1. / TIME_ROUND) as usize));
                        if ui.button("X").clicked() {
                            self.period = None;
                        }
                    } else if ui.button("+").clicked() {
                        self.period = Some(Self::default().period.unwrap());
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
        egui::CollapsingHeader::new("Robot sensor")
            .id_salt(format!("robot-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(format!("Detection distance: {}", self.detection_distance));
                });

                ui.horizontal(|ui| {
                    if let Some(p) = &self.period {
                        ui.label(format!("Period: {}", p));
                    } else {
                        ui.label("Period: None");
                    }
                });

                SensorFilterConfig::show_filters(&self.filters, ui, ctx, unique_id);

                FaultModelConfig::show_faults(&self.faults, ui, ctx, unique_id);
            });
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

#[cfg(feature = "gui")]
impl UIComponent for RobotSensorRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.label(format!("Last time: {}", self.last_time));
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

        const FIELDS: &[&str] = &["name", "x", "y", "theta"];
        deserializer.deserialize_struct("OrientedRobot", FIELDS, OrientedRobotVisitor)
    }
}

/// Observation of an [`OrientedRobot`].
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct OrientedRobotObservation {
    /// Name of the Robot
    pub name: String,
    pub labels: Vec<String>,
    /// Pose of the Robot
    pub pose: Vector3<f32>,
    pub applied_faults: Vec<FaultModelConfig>,
}

impl Recordable<OrientedRobotObservationRecord> for OrientedRobotObservation {
    fn record(&self) -> OrientedRobotObservationRecord {
        OrientedRobotObservationRecord {
            name: self.name.clone(),
            labels: self.labels.clone(),
            pose: self.pose.to_owned().into(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct OrientedRobotObservationRecord {
    /// Name of the Robot
    pub name: String,
    pub labels: Vec<String>,
    /// Pose of the Robot
    pub pose: [f32; 3],
}

#[cfg(feature = "gui")]
impl UIComponent for OrientedRobotObservationRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!("Name: {}", self.name));
            ui.label("Labels:");
            for label in &self.labels {
                ui.label(format!("- {}", label));
            }
            ui.label(format!(
                "Pose: ({}, {}, {})",
                self.pose[0], self.pose[1], self.pose[2]
            ));
        });
    }
}

/// Sensor which observe the other Robots.
#[derive(Debug)]
pub struct RobotSensor {
    /// Detection distance
    detection_distance: f32,
    /// Observation period
    period: Option<f32>,
    /// Last observation time.
    last_time: f32,
    faults: SharedMutex<Vec<Box<dyn FaultModel>>>,
    filters: SharedMutex<Vec<Box<dyn SensorFilter>>>,
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
            0.0,
        )
    }

    /// Makes a new [`RobotSensor`] from the given config.
    ///
    /// The map path is relative to the config path of the simulator.
    pub fn from_config(
        config: &RobotSensorConfig,
        _plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        node_name: &String,
        va_factory: &DeterministRandomVariableFactory,
        initial_time: f32,
    ) -> Self {
        if let Some(p) = config.period {
            assert!(p != 0.);
        }
        let fault_models = Arc::new(Mutex::new(Vec::new()));
        let mut unlock_fault_model = fault_models.lock().unwrap();
        for fault_config in &config.faults {
            unlock_fault_model.push(make_fault_model_from_config(
                fault_config,
                global_config,
                node_name,
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

        Self {
            detection_distance: config.detection_distance,
            period: config.period,
            last_time: initial_time,
            faults: fault_models,
            filters,
        }
    }
}

impl Default for RobotSensor {
    fn default() -> Self {
        Self::new()
    }
}

use crate::node::Node;

impl Sensor for RobotSensor {
    fn init(&mut self, _node: &mut Node) {}

    fn get_observations(&mut self, node: &mut Node, time: f32) -> Vec<SensorObservation> {
        let mut observation_list = Vec::<SensorObservation>::new();
        if (time - self.last_time).abs() < TIME_ROUND {
            return observation_list;
        }
        if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
            debug!("Start looking for nodes");
        }
        let state = if let Some(arc_physics) = node.physics() {
            let physics = arc_physics.read().unwrap();
            physics.state(time).clone()
        } else {
            State::new() // 0
        };

        let rotation_matrix =
            nalgebra::geometry::Rotation3::from_euler_angles(0., 0., state.pose.z);
        if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
            debug!("Rotation matrix: {}", rotation_matrix);
        }

        for (i, other_node_name) in node.other_node_names().iter().enumerate() {
            if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
                debug!("Sensing node {}", other_node_name);
            }
            assert!(*other_node_name != node.name());

            let service_manager = node.service_manager();
            match service_manager.read().unwrap().get_real_state(
                &other_node_name.to_string(),
                node,
                time,
            ) {
                Ok(other_state) => {
                    let d = ((other_state.pose.x - state.pose.x).powi(2)
                        + (other_state.pose.y - state.pose.y).powi(2))
                    .sqrt();
                    if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
                        debug!("Distance is {d}");
                    }
                    if d <= self.detection_distance {
                        let robot_seed =
                            1. / (100. * self.period.unwrap_or(TIME_ROUND)) * (i as f32);
                        let pose = rotation_matrix.transpose() * (other_state.pose - state.pose);
                        let mut new_obs = Vec::new();
                        let labels = node
                            .meta_data_list()
                            .unwrap()
                            .read()
                            .unwrap()
                            .get(other_node_name)
                            .map_or(Vec::new(), |md| md.read().unwrap().labels.clone());
                        let obs = SensorObservation::OrientedRobot(OrientedRobotObservation {
                            name: other_node_name.clone(),
                            labels,
                            pose,
                            applied_faults: Vec::new(),
                        });

                        if let Some(observation) = self
                            .filters
                            .lock()
                            .unwrap()
                            .iter()
                            .try_fold(obs, |obs, filter| {
                                filter.filter(time, obs, &state, Some(&other_state))
                            })
                        {
                            new_obs.push(observation); // Not adding directly to observation_list to apply faults only once
                            for fault_model in self.faults.lock().unwrap().iter_mut() {
                                fault_model.add_faults(
                                    time,
                                    time + robot_seed,
                                    self.period.unwrap_or(TIME_ROUND),
                                    &mut new_obs,
                                    SensorObservation::OrientedRobot(
                                        OrientedRobotObservation::default(),
                                    ),
                                );
                            }
                            observation_list.extend(new_obs);
                        } else if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
                            debug!(
                                "Observation of node {} was filtered out",
                                &other_node_name.to_string()
                            );
                        }
                    };
                }
                Err(e) => {
                    match e.error_type() {
                        SimbaErrorTypes::ServiceError(
                            ServiceError::Unavailable | ServiceError::Closed,
                        ) => (),
                        _ => log::error!(
                            "Error trying to get real state of node {}: {}",
                            &other_node_name.to_string(),
                            e.detailed_error()
                        ),
                    };
                }
            };
        }
        self.last_time = time;
        observation_list
    }

    /// Get the next observation time.
    fn next_time_step(&self) -> f32 {
        if let Some(p) = &self.period {
            round_precision(self.last_time + p, TIME_ROUND).unwrap()
        } else {
            f32::INFINITY
        }
    }
}

impl Recordable<SensorRecord> for RobotSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::RobotSensor(RobotSensorRecord {
            last_time: self.last_time,
        })
    }
}
