/*!
Provides the [`Sensor`] trait, which is the interface for all the sensors.
*/

extern crate confy;
use std::sync::{Arc, RwLock};

use pyo3::pyclass;
use serde_derive::{Deserialize, Serialize};

use super::{
    gnss_sensor::{self, GNSSObservation, GNSSObservationRecord},
    odometry_sensor::{self, OdometryObservation, OdometryObservationRecord},
    oriented_landmark_sensor::{
        self, OrientedLandmarkObservation, OrientedLandmarkObservationRecord,
    },
    robot_sensor::{self, OrientedRobotObservation, OrientedRobotObservationRecord},
};

/// Generic trait for the observations. Contains no information, the observation
/// need to be tested for type after.
#[derive(Debug, Clone)]
pub enum Observation {
    OrientedLandmark(OrientedLandmarkObservation),
    Odometry(OdometryObservation),
    GNSS(GNSSObservation),
    OrientedRobot(OrientedRobotObservation),
}

impl Stateful<ObservationRecord> for Observation {
    fn record(&self) -> ObservationRecord {
        match self {
            Observation::OrientedLandmark(o) => ObservationRecord::OrientedLandmark(o.record()),
            Observation::Odometry(o) => ObservationRecord::Odometry(o.record()),
            Observation::GNSS(o) => ObservationRecord::GNSS(o.record()),
            Observation::OrientedRobot(o) => ObservationRecord::OrientedRobot(o.record()),
        }
    }

    fn from_record(&mut self, record: ObservationRecord) {
        match record {
            ObservationRecord::OrientedLandmark(o) => {
                if let Observation::OrientedLandmark(ref mut obs) = self {
                    obs.from_record(o);
                }
            }
            ObservationRecord::Odometry(o) => {
                if let Observation::Odometry(ref mut obs) = self {
                    obs.from_record(o);
                }
            }
            ObservationRecord::GNSS(o) => {
                if let Observation::GNSS(ref mut obs) = self {
                    obs.from_record(o);
                }
            }
            ObservationRecord::OrientedRobot(o) => {
                if let Observation::OrientedRobot(ref mut obs) = self {
                    obs.from_record(o);
                }
            }
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass(get_all)]
pub enum ObservationRecord {
    OrientedLandmark(OrientedLandmarkObservationRecord),
    Odometry(OdometryObservationRecord),
    GNSS(GNSSObservationRecord),
    OrientedRobot(OrientedRobotObservationRecord),
}

/// Enumerates all the possible sensors configurations.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum SensorConfig {
    OrientedLandmarkSensor(Box<oriented_landmark_sensor::OrientedLandmarkSensorConfig>),
    OdometrySensor(odometry_sensor::OdometrySensorConfig),
    GNSSSensor(gnss_sensor::GNSSSensorConfig),
    RobotSensor(robot_sensor::RobotSensorConfig),
}

/// Enumerates all the sensor records.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass(get_all)]
pub enum SensorRecord {
    OrientedLandmarkSensor(oriented_landmark_sensor::OrientedLandmarkSensorRecord),
    OdometrySensor(odometry_sensor::OdometrySensorRecord),
    GNSSSensor(gnss_sensor::GNSSSensorRecord),
    RobotSensor(robot_sensor::RobotSensorRecord),
}

use crate::{robot::Robot, stateful::Stateful};

/// Sensor trait which need to be implemented by each sensors.
pub trait Sensor:
    std::fmt::Debug + std::marker::Send + std::marker::Sync + Stateful<SensorRecord>
{
    /// Initialize the [`Sensor`]. Should be called at the beginning of the run, after
    /// the initialization of the modules.
    fn init(
        &mut self,
        robot: &mut Robot,
        robot_list: &Arc<RwLock<Vec<Arc<RwLock<Robot>>>>>,
        robot_idx: usize,
    );

    /// Get the observations available at the given `time`.
    ///
    /// ## Arguments
    /// * `robot` - Reference to the robot to access the modules.
    /// * `time` - Time at which the observations are taken.
    ///
    /// ## Return
    /// List of [`GenericObservation`]s, could be empty if no [`Sensor`] provided observation
    /// at this `time`.
    fn get_observations(&mut self, robot: &mut Robot, time: f32) -> Vec<Observation>;

    /// Get the time of the next observation.
    fn next_time_step(&self) -> f32;

    /// Period of the [`Sensor`].
    fn period(&self) -> f32;
}
