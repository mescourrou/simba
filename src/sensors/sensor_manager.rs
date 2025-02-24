/*!
Provide the [`SensorManager`], which owns the different [`Sensor`]s and get the
available observations.
*/

extern crate confy;
use core::f32;
use config_checker::macros::Check;
use pyo3::pyclass;
use serde_derive::{Deserialize, Serialize};
use std::sync::{Arc, RwLock};

use crate::robot::Robot;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::{simulator::SimulatorConfig, stateful::Stateful};

use super::gnss_sensor::GNSSSensor;
use super::odometry_sensor::OdometrySensor;
use super::robot_sensor::RobotSensor;
use super::sensor::ObservationRecord;
use super::{
    oriented_landmark_sensor::OrientedLandmarkSensor,
    sensor::{Observation, Sensor, SensorConfig, SensorRecord},
};
use crate::plugin_api::PluginAPI;

/// Configuration listing all the [`SensorConfig`]s.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct SensorManagerConfig {
    #[check]
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
#[pyclass(get_all)]
pub struct SensorManagerRecord {
    pub sensors: Vec<SensorRecord>,
    pub next_time: Option<f32>,
    pub last_observations: Vec<ObservationRecord>,
}

/// Sensor manager which manages all the robot's [`Sensor`]s.
#[derive(Debug)]
pub struct SensorManager {
    sensors: Vec<Arc<RwLock<Box<dyn Sensor>>>>,
    next_time: Option<f32>,
    last_observations: Vec<ObservationRecord>,
}

impl SensorManager {
    /// Makes a new [`SensorManager`] without any [`Sensor`].
    pub fn new() -> Self {
        Self {
            sensors: Vec::new(),
            next_time: None,
            last_observations: Vec::new(),
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
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        robot_name: &String,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        let mut manager = Self::new();
        for sensor_config in &config.sensors {
            manager
                .sensors
                .push(Arc::new(RwLock::new(match &sensor_config {
                    SensorConfig::OrientedLandmarkSensor(c) => {
                        Box::new(OrientedLandmarkSensor::from_config(
                            c,
                            plugin_api,
                            global_config,
                            robot_name,
                            va_factory,
                        )) as Box<dyn Sensor>
                    }
                    SensorConfig::OdometrySensor(c) => Box::new(OdometrySensor::from_config(
                        c,
                        plugin_api,
                        global_config,
                        robot_name,
                        va_factory,
                    )) as Box<dyn Sensor>,
                    SensorConfig::GNSSSensor(c) => Box::new(GNSSSensor::from_config(
                        c,
                        plugin_api,
                        global_config,
                        robot_name,
                        va_factory,
                    )) as Box<dyn Sensor>,
                    SensorConfig::RobotSensor(c) => Box::new(RobotSensor::from_config(
                        c,
                        plugin_api,
                        global_config,
                        robot_name,
                        va_factory,
                    )) as Box<dyn Sensor>,
                })));
        }
        manager.next_time = None;
        for sensor in &manager.sensors {
            manager.next_time = Some(
                manager
                    .next_time
                    .unwrap_or(f32::INFINITY)
                    .min(sensor.read().unwrap().next_time_step()),
            );
        }
        manager
    }

    /// Initialize the [`Sensor`]s. Should be called at the beginning of the run, after
    /// the initialization of the modules.
    pub fn init(
        &mut self,
        robot: &mut Robot,
        robot_list: &Arc<RwLock<Vec<Arc<RwLock<Robot>>>>>,
        robot_idx: usize,
    ) {
        for sensor in &mut self.sensors {
            sensor.write().unwrap().init(robot, robot_list, robot_idx);
        }
    }

    /// Get the observations at the given `time`.
    pub fn get_observations(&mut self, robot: &mut Robot, time: f32) -> Vec<Observation> {
        let mut observations = Vec::<Observation>::new();
        let mut min_next_time = None;
        for sensor in &mut self.sensors {
            let sensor_observations = sensor.write().unwrap().get_observations(robot, time);
            for obs in sensor_observations {
                observations.push(obs);
            }
            min_next_time = Some(
                min_next_time
                    .unwrap_or(f32::INFINITY)
                    .min(sensor.read().unwrap().next_time_step()),
            );
        }
        self.next_time = min_next_time;
        self.last_observations = observations.iter().map(|obs| obs.record()).collect();
        observations
    }

    /// Get the time of the next observation.
    pub fn next_time_step(&self) -> Option<f32> {
        self.next_time
    }
}

impl Stateful<SensorManagerRecord> for SensorManager {
    fn record(&self) -> SensorManagerRecord {
        let mut record = SensorManagerRecord {
            next_time: self.next_time,
            sensors: Vec::new(),
            last_observations: self.last_observations.clone(),
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
