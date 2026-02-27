/*!
This module provides [`Sensor management`](sensor_manager::SensorManager) and
[`Sensor Implementation`](sensor::Sensor).

The [`SensorManager`](sensor_manager::SensorManager) manages all the sensors of a robot.

## How to add a new sensor ?

To add a new [`Sensor`](sensor::Sensor), you should implement the
[`Sensor`](sensor::Sensor) trait, and add your new implementation to the
[`SensorConfig`](sensor::SensorConfig) and the
[`SensorRecord`](sensor::SensorRecord) enumarations.
*/

pub mod displacement_sensor;
pub mod external_sensor;
pub mod gnss_sensor;
pub mod oriented_landmark_sensor;
pub mod robot_sensor;
pub mod sensor_manager;
pub mod speed_sensor;

pub mod fault_models;
pub mod sensor_filters;

extern crate confy;

use serde_derive::{Deserialize, Serialize};
use simba_macros::config_derives;

use {
    gnss_sensor::{GNSSObservation, GNSSObservationRecord},
    oriented_landmark_sensor::{OrientedLandmarkObservation, OrientedLandmarkObservationRecord},
    robot_sensor::{OrientedRobotObservation, OrientedRobotObservationRecord},
    speed_sensor::{SpeedObservation, SpeedObservationRecord},
};

use crate::{
    errors::SimbaResult,
    node::Node,
    recordable::Recordable,
    sensors::{
        displacement_sensor::{DisplacementObservation, DisplacementObservationRecord},
        external_sensor::{ExternalObservation, ExternalObservationRecord},
    },
};
#[cfg(feature = "gui")]
use crate::{
    gui::{UIComponent, utils::string_combobox},
    simulator::SimulatorConfig,
    utils::enum_tools::ToVec,
};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Observation {
    pub sensor_name: String,
    pub observer: String,
    pub time: f32,
    pub sensor_observation: SensorObservation,
}

impl Observation {
    pub fn new() -> Self {
        Self {
            sensor_name: "sensor".to_string(),
            observer: "someone".to_string(),
            time: 0.,
            sensor_observation: SensorObservation::Speed(SpeedObservation::default()),
        }
    }
}

impl Default for Observation {
    fn default() -> Self {
        Self::new()
    }
}

impl Recordable<ObservationRecord> for Observation {
    fn record(&self) -> ObservationRecord {
        ObservationRecord {
            sensor_name: self.sensor_name.clone(),
            observer: self.observer.clone(),
            time: self.time,
            sensor_observation: self.sensor_observation.record(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ObservationRecord {
    pub sensor_name: String,
    pub observer: String,
    pub time: f32,
    pub sensor_observation: SensorObservationRecord,
}

// Implementation of traits needed for sorting observations, first by time, then by sensor name, then by observer.
impl PartialEq for ObservationRecord {
    fn eq(&self, other: &Self) -> bool {
        self.time == other.time
            && self.sensor_name == other.sensor_name
            && self.observer == other.observer
    }
}

impl Eq for ObservationRecord {}

impl PartialOrd for ObservationRecord {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for ObservationRecord {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        match self.time.partial_cmp(&other.time) {
            Some(std::cmp::Ordering::Equal) => match self.sensor_name.cmp(&other.sensor_name) {
                std::cmp::Ordering::Equal => self.observer.cmp(&other.observer),
                other => other,
            },
            other => other.unwrap_or(std::cmp::Ordering::Equal),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ObservationRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.label(format!("Sensor name: {}", self.sensor_name));
        ui.label(format!("Observer: {}", self.observer));
        ui.label(format!("Time: {}", self.time));
        self.sensor_observation.show(ui, ctx, unique_id);
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SensorObservation {
    OrientedLandmark(OrientedLandmarkObservation),
    Speed(SpeedObservation),
    Displacement(DisplacementObservation),
    GNSS(GNSSObservation),
    OrientedRobot(OrientedRobotObservation),
    External(ExternalObservation),
}

impl Recordable<SensorObservationRecord> for SensorObservation {
    fn record(&self) -> SensorObservationRecord {
        match self {
            SensorObservation::OrientedLandmark(o) => {
                SensorObservationRecord::OrientedLandmark(o.record())
            }
            SensorObservation::Speed(o) => SensorObservationRecord::Speed(o.record()),
            SensorObservation::Displacement(o) => SensorObservationRecord::Displacement(o.record()),
            SensorObservation::GNSS(o) => SensorObservationRecord::GNSS(o.record()),
            SensorObservation::OrientedRobot(o) => {
                SensorObservationRecord::OrientedRobot(o.record())
            }
            SensorObservation::External(o) => SensorObservationRecord::External(o.record()),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum SensorObservationRecord {
    OrientedLandmark(OrientedLandmarkObservationRecord),
    Speed(SpeedObservationRecord),
    Displacement(DisplacementObservationRecord),
    GNSS(GNSSObservationRecord),
    OrientedRobot(OrientedRobotObservationRecord),
    External(ExternalObservationRecord),
}

#[cfg(feature = "gui")]
impl UIComponent for SensorObservationRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| match self {
            Self::OrientedLandmark(r) => r.show(ui, ctx, unique_id),
            Self::Speed(r) => r.show(ui, ctx, unique_id),
            Self::Displacement(r) => r.show(ui, ctx, unique_id),
            Self::GNSS(r) => r.show(ui, ctx, unique_id),
            Self::OrientedRobot(r) => r.show(ui, ctx, unique_id),
            Self::External(r) => r.show(ui, ctx, unique_id),
        });
    }
}

/// Enumerates all the possible sensors configurations.
#[config_derives]
pub enum SensorConfig {
    OrientedLandmarkSensor(oriented_landmark_sensor::OrientedLandmarkSensorConfig),
    SpeedSensor(speed_sensor::SpeedSensorConfig),
    DisplacementSensor(displacement_sensor::DisplacementSensorConfig),
    GNSSSensor(gnss_sensor::GNSSSensorConfig),
    RobotSensor(robot_sensor::RobotSensorConfig),
    External(external_sensor::ExternalSensorConfig),
}

#[cfg(feature = "gui")]
impl UIComponent for SensorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        let mut current_str = self.to_string();
        ui.horizontal(|ui| {
            ui.label("Sensor:");
            string_combobox(
                ui,
                &SensorConfig::to_vec()
                    .iter()
                    .map(|x: &&str| String::from(*x))
                    .collect(),
                &mut current_str,
                format!("sensor-choice-{}", unique_id),
            );
        });
        if current_str != self.to_string() {
            match current_str.as_str() {
                "OrientedLandmarkSensor" => {
                    *self = SensorConfig::OrientedLandmarkSensor(
                        oriented_landmark_sensor::OrientedLandmarkSensorConfig::default(),
                    )
                }
                "SpeedSensor" => {
                    *self = SensorConfig::SpeedSensor(speed_sensor::SpeedSensorConfig::default())
                }
                "DisplacementSensor" => {
                    *self = SensorConfig::DisplacementSensor(
                        displacement_sensor::DisplacementSensorConfig::default(),
                    )
                }
                "GNSSSensor" => {
                    *self = SensorConfig::GNSSSensor(gnss_sensor::GNSSSensorConfig::default())
                }
                "RobotSensor" => {
                    *self = SensorConfig::RobotSensor(robot_sensor::RobotSensorConfig::default())
                }
                "External" => {
                    *self = SensorConfig::External(external_sensor::ExternalSensorConfig::default())
                }
                _ => panic!("Where did you find this value?"),
            };
        }
        match self {
            SensorConfig::OrientedLandmarkSensor(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            SensorConfig::SpeedSensor(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            SensorConfig::DisplacementSensor(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            SensorConfig::GNSSSensor(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            SensorConfig::RobotSensor(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            SensorConfig::External(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
        }
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.horizontal(|ui| {
            ui.label(format!("Sensor: {}", self));
        });
        match self {
            SensorConfig::OrientedLandmarkSensor(c) => c.show(ui, ctx, unique_id),
            SensorConfig::SpeedSensor(c) => c.show(ui, ctx, unique_id),
            SensorConfig::DisplacementSensor(c) => c.show(ui, ctx, unique_id),
            SensorConfig::GNSSSensor(c) => c.show(ui, ctx, unique_id),
            SensorConfig::RobotSensor(c) => c.show(ui, ctx, unique_id),
            SensorConfig::External(c) => c.show(ui, ctx, unique_id),
        }
    }
}

/// Enumerates all the sensor records.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum SensorRecord {
    OrientedLandmarkSensor(oriented_landmark_sensor::OrientedLandmarkSensorRecord),
    SpeedSensor(speed_sensor::SpeedSensorRecord),
    DisplacementSensor(displacement_sensor::DisplacementSensorRecord),
    GNSSSensor(gnss_sensor::GNSSSensorRecord),
    RobotSensor(robot_sensor::RobotSensorRecord),
    External(external_sensor::ExternalSensorRecord),
}

#[cfg(feature = "gui")]
impl UIComponent for SensorRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| match self {
            Self::OrientedLandmarkSensor(r) => {
                egui::CollapsingHeader::new("OrientedLandmark").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
            Self::SpeedSensor(r) => {
                egui::CollapsingHeader::new("SpeedSensor").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
            Self::DisplacementSensor(r) => {
                egui::CollapsingHeader::new("DisplacementSensor").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
            Self::GNSSSensor(r) => {
                egui::CollapsingHeader::new("GNSSSensor").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
            Self::RobotSensor(r) => {
                egui::CollapsingHeader::new("RobotSensor").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
            Self::External(r) => {
                egui::CollapsingHeader::new("ExternalSensor").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
        });
    }
}

/// Sensor trait which need to be implemented by each sensors.
pub trait Sensor:
    std::fmt::Debug + std::marker::Send + std::marker::Sync + Recordable<SensorRecord>
{
    /// Initialize the [`Sensor`]. Should be called at the beginning of the run, after
    /// the initialization of the modules.
    fn post_init(&mut self, _node: &mut Node, _initial_time: f32) -> SimbaResult<()> {
        Ok(())
    }

    /// Get the observations available at the given `time`.
    ///
    /// ## Arguments
    /// * `node` - Reference to the node to access the modules.
    /// * `time` - Time at which the observations are taken.
    ///
    /// ## Return
    /// List of [`SensorObservation`]s, could be empty if no [`Sensor`] provided observation
    /// at this `time`.
    fn get_observations(&mut self, node: &mut Node, time: f32) -> Vec<SensorObservation>;

    /// Get the time of the next observation.
    fn next_time_step(&self) -> f32;
}
