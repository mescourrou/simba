extern crate confy;
use serde_derive::{Serialize, Deserialize};

use crate::physics::physic::Physic;
use super::oriented_landmark_sensor;

pub trait GenericObservation : std::fmt::Debug {}


#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum SensorConfig {
    OrientedLandmarkSensor(Box<oriented_landmark_sensor::OrientedLandmarkSensorConfig>)
}

#[derive(Serialize, Deserialize, Debug)]
pub enum SensorRecord {
    OrientedLandmarkSensor(oriented_landmark_sensor::OrientedLandmarkSensorRecord)
}

pub trait Sensor : std::fmt::Debug + std::marker::Send + std::marker::Sync {
    fn get_observations(&mut self, physic: &dyn Physic, time: f32) -> Vec<Box<dyn GenericObservation>>;
    fn record(&self) ->  SensorRecord;
    fn next_time_step(&self) -> f32;
    fn period(&self) -> f32;
}


