/*!
Provides the [`Sensor`] trait, which is the interface for all the sensors.
*/

extern crate confy;
use serde_derive::{Deserialize, Serialize};

use super::{odometry_sensor, oriented_landmark_sensor};

/// Generic trait for the observations. Contains no information, the observation
/// need to be tested for type after.
pub trait GenericObservation: std::fmt::Debug {}

/// Enumerates all the possible sensors configurations.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum SensorConfig {
    OrientedLandmarkSensor(Box<oriented_landmark_sensor::OrientedLandmarkSensorConfig>),
    OdometrySensor(odometry_sensor::OdometrySensorConfig),
}

/// Enumerates all the sensor records.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum SensorRecord {
    OrientedLandmarkSensor(oriented_landmark_sensor::OrientedLandmarkSensorRecord),
    OdometrySensor(odometry_sensor::OdometrySensorRecord),
}

use crate::{stateful::Stateful, turtlebot::Turtlebot};

/// Sensor trait which need to be implemented by each sensors.
pub trait Sensor:
    std::fmt::Debug + std::marker::Send + std::marker::Sync + Stateful<SensorRecord>
{
    /// Initialize the [`Sensor`]. Should be called at the beginning of the run, after
    /// the initialization of the modules.
    fn init(&mut self, turtle: &mut Turtlebot);

    /// Get the observations available at the given `time`.
    ///
    /// ## Arguments
    /// * `turtle` - Reference to the robot to access the modules.
    /// * `time` - Time at which the observations are taken.
    ///
    /// ## Return
    /// List of [`GenericObservation`]s, could be empty if no [`Sensor`] provided observation
    /// at this `time`.
    fn get_observations(
        &mut self,
        turtle: &mut Turtlebot,
        time: f32,
    ) -> Vec<Box<dyn GenericObservation>>;

    /// Get the time of the next observation.
    fn next_time_step(&self) -> f32;

    /// Period of the [`Sensor`].
    fn period(&self) -> f32;
}
