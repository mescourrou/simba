/*!
Provides the [`Sensor`] trait, which is the interface for all the sensors.
*/

extern crate confy;

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

#[cfg(feature = "gui")]
use crate::{
    gui::{utils::string_combobox, UIComponent},
    simulator::SimulatorConfig,
    utils::enum_tools::ToVec,
};
use crate::{node::Node, recordable::Recordable};

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


#[cfg(feature = "gui")]
impl UIComponent for ObservationRecord {
    fn show(
            &self,
            ui: &mut egui::Ui,
            ctx: &egui::Context,
            unique_id: &String,
        ) {
        ui.label(format!("Sensor name: {}", self.sensor_name));
        ui.label(format!("Observer: {}", self.observer));
        ui.label(format!("Time: {}", self.time));
        self.sensor_observation.show(ui, ctx, unique_id);
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SensorObservation {
    OrientedLandmark(OrientedLandmarkObservation),
    Odometry(OdometryObservation),
    GNSS(GNSSObservation),
    OrientedRobot(OrientedRobotObservation),
}

impl Recordable<SensorObservationRecord> for SensorObservation {
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
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum SensorObservationRecord {
    OrientedLandmark(OrientedLandmarkObservationRecord),
    Odometry(OdometryObservationRecord),
    GNSS(GNSSObservationRecord),
    OrientedRobot(OrientedRobotObservationRecord),
}


#[cfg(feature = "gui")]
impl UIComponent for SensorObservationRecord {
    fn show(
            &self,
            ui: &mut egui::Ui,
            ctx: &egui::Context,
            unique_id: &String,
        ) {
        ui.vertical(|ui| {
            match self {
                Self::OrientedLandmark(r) => r.show(ui, ctx, unique_id),
                Self::Odometry(r) => r.show(ui, ctx, unique_id),
                Self::GNSS(r) => r.show(ui, ctx, unique_id),
                Self::OrientedRobot(r) => r.show(ui, ctx, unique_id),

            }
        });
    }
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
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
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
            SensorConfig::OrientedLandmarkSensor(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            SensorConfig::OdometrySensor(c) => c.show_mut(
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
        }
    }

    fn show(
        &self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        unique_id: &String,
    ) {
        ui.horizontal(|ui| {
            ui.label(format!("Sensor: {}", self.to_string()));
        });
        match self {
            SensorConfig::OrientedLandmarkSensor(c) => c.show(
                ui,
                ctx,
                unique_id,
            ),
            SensorConfig::OdometrySensor(c) => c.show(
                ui,
                ctx,
                unique_id,
            ),
            SensorConfig::GNSSSensor(c) => c.show(
                ui,
                ctx,
                unique_id,
            ),
            SensorConfig::RobotSensor(c) => c.show(
                ui,
                ctx,
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
impl UIComponent for SensorRecord {
    fn show(
            &self,
            ui: &mut egui::Ui,
            ctx: &egui::Context,
            unique_id: &String,
        ) {
        ui.vertical(|ui| {
            match self {
                Self::OrientedLandmarkSensor(r) => {
                    egui::CollapsingHeader::new("OrientedLandmark").show(ui, |ui| {
                        r.show(ui, ctx, unique_id);
                    });
                },
                Self::OdometrySensor(r) => {
                    egui::CollapsingHeader::new("OdometrySensor").show(ui, |ui| {
                        r.show(ui, ctx, unique_id);
                    });
                },
                Self::GNSSSensor(r) => {
                    egui::CollapsingHeader::new("GNSSSensor").show(ui, |ui| {
                        r.show(ui, ctx, unique_id);
                    });
                },
                Self::RobotSensor(r) => {
                    egui::CollapsingHeader::new("RobotSensor").show(ui, |ui| {
                        r.show(ui, ctx, unique_id);
                    });
                },

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
    fn init(&mut self, node: &mut Node);

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

    /// Period of the [`Sensor`].
    fn period(&self) -> f32;
}
