/*!
Provide the [`SensorManager`], which owns the different [`Sensor`]s and get the
available observations.
*/

extern crate confy;
use serde_derive::{Deserialize, Serialize};
use std::sync::{Arc, RwLock};

use crate::turtlebot::Turtlebot;
use crate::{simulator::SimulatorMetaConfig, stateful::Stateful};

use super::{
    oriented_landmark_sensor::OrientedLandmarkSensor,
    sensor::{GenericObservation, Sensor, SensorConfig, SensorRecord},
};
use crate::plugin_api::PluginAPI;

/// Configuration listing all the [`SensorConfig`]s.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SensorManagerConfig {
    pub sensors: Vec<SensorConfig>,
}

impl Default for SensorManagerConfig {
    fn default() -> Self {
        Self {
            sensors: Vec::new(),
        }
    }
}

/// Record listing all the [`SensorRecord`]s and the next observation time.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SensorManagerRecord {
    pub sensors: Vec<SensorRecord>,
    pub next_time: f32,
}

/// Sensor manager which manages all the robot's [`Sensor`]s.
#[derive(Debug)]
pub struct SensorManager {
    sensors: Vec<Arc<RwLock<Box<dyn Sensor>>>>,
    next_time: f32,
}

impl SensorManager {
    /// Makes a new [`SensorManager`] without any [`Sensor`].
    pub fn new() -> Self {
        Self {
            sensors: Vec::new(),
            next_time: f32::INFINITY,
        }
    }

    /// Makes a new [`SensorManager`] from the given config.
    ///
    /// ## Arguments
    /// * `config` - Config of the [`SensorManager`].
    /// * `plugin_api` - Not used yet, but will be used for external [`Sensor`]s.
    /// * `meta_config` - Simulator meta config.
    pub fn from_config(
        config: &SensorManagerConfig,
        plugin_api: &Option<Box<dyn PluginAPI>>,
        meta_config: SimulatorMetaConfig,
    ) -> Self {
        let mut manager = Self::new();
        for sensor_config in &config.sensors {
            manager
                .sensors
                .push(Arc::new(RwLock::new(Box::new(match &sensor_config {
                    SensorConfig::OrientedLandmarkSensor(c) => {
                        OrientedLandmarkSensor::from_config(c, plugin_api, meta_config.clone())
                    }
                }))));
        }
        manager.next_time = f32::INFINITY;
        for sensor in &manager.sensors {
           manager.next_time = manager.next_time.min(sensor.read().unwrap().next_time_step());
        }
        manager
    }

    /// Get the observations at the given `time`.
    pub fn get_observations(
        &mut self,
        turtle: &mut Turtlebot,
        time: f32,
    ) -> Vec<Box<dyn GenericObservation>> {
        let mut observations = Vec::<Box<dyn GenericObservation>>::new();
        let mut min_next_time = f32::INFINITY;
        for sensor in &mut self.sensors {
            let sensor_observations = sensor.write().unwrap().get_observations(turtle, time);
            for obs in sensor_observations {
                observations.push(obs);
            }
            min_next_time = min_next_time.min(sensor.read().unwrap().next_time_step());
        }
        self.next_time = min_next_time;
        observations
    }

    /// Get the time of the next observation.
    pub fn next_time_step(&self) -> f32 {
        self.next_time
    }
}

impl Stateful<SensorManagerRecord> for SensorManager {
    fn record(&self) -> SensorManagerRecord {
        let mut record = SensorManagerRecord {
            next_time: self.next_time,
            sensors: Vec::new(),
        };
        for sensor in &self.sensors {
            record.sensors.push(sensor.read().unwrap().record());
        }
        record
    }

    fn from_record(&mut self, record: SensorManagerRecord) {
        self.next_time = record.next_time;
        for (i, sensor) in self.sensors.iter_mut().enumerate() {
            sensor
                .write()
                .unwrap()
                .from_record(record.sensors[i].clone())
        }
    }
}
