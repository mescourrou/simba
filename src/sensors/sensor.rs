/*!
Provides the [`Sensor`] trait, which is the interface for all the sensors.
*/

extern crate confy;
use std::sync::{Arc, RwLock};

use config_checker::macros::Check;
use serde_derive::{Deserialize, Serialize};
use simba_macros::{EnumToString, ToVec};

use super::{
    gnss_sensor::{self, GNSSObservation, GNSSObservationRecord},
    odometry_sensor::{self, OdometryObservation, OdometryObservationRecord},
    oriented_landmark_sensor::{
        self, OrientedLandmarkObservation, OrientedLandmarkObservationRecord,
    },
    robot_sensor::{self, OrientedRobotObservation, OrientedRobotObservationRecord},
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
            sensor_observation: SensorObservation::Odometry(OdometryObservation::default()),
        }
    }
}

impl Stateful<ObservationRecord> for Observation {
    fn from_record(&mut self, record: ObservationRecord) {
        self.observer = record.observer;
        self.sensor_name = record.sensor_name;
        self.time = record.time;
        self.sensor_observation
            .from_record(record.sensor_observation);
    }

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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SensorObservation {
    OrientedLandmark(OrientedLandmarkObservation),
    Odometry(OdometryObservation),
    GNSS(GNSSObservation),
    OrientedRobot(OrientedRobotObservation),
}

impl Stateful<SensorObservationRecord> for SensorObservation {
    fn record(&self) -> SensorObservationRecord {
        match self {
            SensorObservation::OrientedLandmark(o) => {
                SensorObservationRecord::OrientedLandmark(o.record())
            }
            SensorObservation::Odometry(o) => SensorObservationRecord::Odometry(o.record()),
            SensorObservation::GNSS(o) => SensorObservationRecord::GNSS(o.record()),
            SensorObservation::OrientedRobot(o) => {
                SensorObservationRecord::OrientedRobot(o.record())
            }
        }
    }

    fn from_record(&mut self, record: SensorObservationRecord) {
        match record {
            SensorObservationRecord::OrientedLandmark(o) => {
                if let SensorObservation::OrientedLandmark(ref mut obs) = self {
                    obs.from_record(o);
                }
            }
            SensorObservationRecord::Odometry(o) => {
                if let SensorObservation::Odometry(ref mut obs) = self {
                    obs.from_record(o);
                }
            }
            SensorObservationRecord::GNSS(o) => {
                if let SensorObservation::GNSS(ref mut obs) = self {
                    obs.from_record(o);
                }
            }
            SensorObservationRecord::OrientedRobot(o) => {
                if let SensorObservation::OrientedRobot(ref mut obs) = self {
                    obs.from_record(o);
                }
            }
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum SensorObservationRecord {
    OrientedLandmark(OrientedLandmarkObservationRecord),
    Odometry(OdometryObservationRecord),
    GNSS(GNSSObservationRecord),
    OrientedRobot(OrientedRobotObservationRecord),
}

/// Enumerates all the possible sensors configurations.
#[derive(Serialize, Deserialize, Debug, Clone, Check, EnumToString, ToVec)]
#[serde(deny_unknown_fields)]
pub enum SensorConfig {
    OrientedLandmarkSensor(oriented_landmark_sensor::OrientedLandmarkSensorConfig),
    OdometrySensor(odometry_sensor::OdometrySensorConfig),
    GNSSSensor(gnss_sensor::GNSSSensorConfig),
    RobotSensor(robot_sensor::RobotSensorConfig),
}

#[cfg(feature = "gui")]
impl UIComponent for SensorConfig {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::HashMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        let mut current_str = self.to_string();
        ui.horizontal(|ui| {
            ui.label("Sensor:");
            string_combobox(
                ui,
                &SensorConfig::to_vec()
                    .iter()
                    .map(|x| String::from(*x))
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
                "OdometrySensor" => {
                    *self = SensorConfig::OdometrySensor(
                        odometry_sensor::OdometrySensorConfig::default(),
                    )
                }
                "GNSSSensor" => {
                    *self = SensorConfig::GNSSSensor(gnss_sensor::GNSSSensorConfig::default())
                }
                "RobotSensor" => {
                    *self = SensorConfig::RobotSensor(robot_sensor::RobotSensorConfig::default())
                }
                _ => panic!("Where did you find this value?"),
            };
        }
        match self {
            SensorConfig::OrientedLandmarkSensor(c) => c.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            SensorConfig::OdometrySensor(c) => c.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            SensorConfig::GNSSSensor(c) => c.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            SensorConfig::RobotSensor(c) => c.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
        }
    }
}

/// Enumerates all the sensor records.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum SensorRecord {
    OrientedLandmarkSensor(oriented_landmark_sensor::OrientedLandmarkSensorRecord),
    OdometrySensor(odometry_sensor::OdometrySensorRecord),
    GNSSSensor(gnss_sensor::GNSSSensorRecord),
    RobotSensor(robot_sensor::RobotSensorRecord),
}

#[cfg(feature = "gui")]
use crate::gui::{utils::string_combobox, UIComponent};
use crate::{
    node::Node,
    simulator::SimulatorConfig,
    stateful::Stateful,
    utils::enum_tools::ToVec,
};

/// Sensor trait which need to be implemented by each sensors.
pub trait Sensor:
    std::fmt::Debug + std::marker::Send + std::marker::Sync + Stateful<SensorRecord>
{
    /// Initialize the [`Sensor`]. Should be called at the beginning of the run, after
    /// the initialization of the modules.
    fn init(&mut self, node: &mut Node);

    /// Get the observations available at the given `time`.
    ///
    /// ## Arguments
    /// * `node` - Reference to the node to access the modules.
    /// * `time` - Time at which the observations are taken.
    ///
    /// ## Return
    /// List of [`GenericObservation`]s, could be empty if no [`Sensor`] provided observation
    /// at this `time`.
    fn get_observations(&mut self, node: &mut Node, time: f32) -> Vec<SensorObservation>;

    /// Get the time of the next observation.
    fn next_time_step(&self) -> f32;

    /// Period of the [`Sensor`].
    fn period(&self) -> f32;
}
