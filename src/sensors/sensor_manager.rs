extern crate confy;
use serde_derive::{Serialize, Deserialize};

use crate::physics::physic::Physic;

use super::{sensor::{Sensor, SensorConfig, GenericObservation}, oriented_landmark_sensor::OrientedLandmarkSensor};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SensorManagerConfig {
    sensors: Vec<SensorConfig>
}

impl Default for SensorManagerConfig {
    fn default() -> Self {
        Self {
            sensors: Vec::new()
        }
    }
}

#[derive(Debug)]
pub struct SensorManager {
    sensors: Vec<Box<dyn Sensor>>,
    next_time: f32
}

impl SensorManager {
    pub fn new() -> Self {
        Self {
            sensors: Vec::new(),
            next_time: 0.
        }
    }

    pub fn from_config(config: &SensorManagerConfig) -> Self {
        let mut manager = Self::new();
        for sensor_config in &config.sensors {
            manager.sensors.push(Box::new(
                match &sensor_config {
                    SensorConfig::OrientedLandmarkSensor(c) => OrientedLandmarkSensor::from_config(c)
                }));
        }
        manager
    }

    pub fn get_observations(&mut self, physic: &dyn Physic, time: f32) -> Vec<Box<dyn GenericObservation>> {
        let mut observations = Vec::<Box<dyn GenericObservation>>::new();
        let mut min_next_time = f32::INFINITY;
        for sensor in &mut self.sensors {
            let sensor_observations = sensor.get_observations(physic, time);
            for obs in sensor_observations {
                observations.push(obs);
            }
            min_next_time = min_next_time.min(sensor.next_time_step());
        }
        self.next_time = min_next_time;
        observations
    }

    pub fn next_time_step(&self) -> f32 {
        self.next_time
    }
}