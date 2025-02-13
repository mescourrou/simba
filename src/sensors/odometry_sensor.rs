/*!
Provides a [`Sensor`] which can provide linear velocity and angular velocity.
*/

use std::sync::{Arc, RwLock};

use super::fault_models::fault_model::{FaultModel, FaultModelConfig};
use super::sensor::{Observation, Sensor, SensorRecord};

use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::state_estimator::{State, StateRecord};
use crate::stateful::Stateful;
use crate::utils::determinist_random_variable::{
    DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig,
};
use pyo3::pyclass;
use serde_derive::{Deserialize, Serialize};

extern crate nalgebra as na;

/// Configuration of the [`OdometrySensor`].
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct OdometrySensorConfig {
    /// Observation period of the sensor.
    pub period: f32,
    pub faults: Vec<FaultModelConfig>,
}

impl Default for OdometrySensorConfig {
    fn default() -> Self {
        Self {
            period: 0.1,
            faults: Vec::new(),
        }
    }
}

/// Record of the [`OdometrySensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass(get_all)]
pub struct OdometrySensorRecord {
    last_time: f32,
    last_state: StateRecord,
}

impl Default for OdometrySensorRecord {
    fn default() -> Self {
        Self {
            last_time: 0.,
            last_state: StateRecord::default(),
        }
    }
}

/// Observation of the odometry.
#[derive(Serialize, Deserialize, Debug)]
pub struct OdometryObservation {
    pub linear_velocity: f32,
    pub angular_velocity: f32,
}

impl Stateful<OdometryObservationRecord> for OdometryObservation {
    fn record(&self) -> OdometryObservationRecord {
        OdometryObservationRecord {
            linear_velocity: self.linear_velocity,
            angular_velocity: self.angular_velocity,
        }
    }

    fn from_record(&mut self, record: OdometryObservationRecord) {
        self.linear_velocity = record.linear_velocity;
        self.angular_velocity = record.angular_velocity;
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
#[pyclass(get_all)]
pub struct OdometryObservationRecord {
    pub linear_velocity: f32,
    pub angular_velocity: f32,
}

/// Sensor which observes the robot's odometry
#[derive(Debug)]
pub struct OdometrySensor {
    /// Last state to compute the velocity.
    last_state: State,
    /// Observation period
    period: f32,
    /// Last observation time.
    last_time: f32,
    faults: Vec<FaultModel>,
}

impl OdometrySensor {
    /// Makes a new [`OdometrySensor`].
    pub fn new() -> Self {
        OdometrySensor::from_config(
            &OdometrySensorConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Makes a new [`OdometrySensor`] from the given config.
    pub fn from_config(
        config: &OdometrySensorConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        _global_config: &SimulatorConfig,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        let mut fault_models = Vec::new();
        for fault_config in &config.faults {
            fault_models.push(FaultModel::from_config(&fault_config, va_factory));
        }
        Self {
            last_state: State::new(),
            period: config.period,
            last_time: 0.,
            faults: fault_models,
        }
    }
}

use crate::robot::Robot;

impl Sensor for OdometrySensor {
    fn init(
        &mut self,
        robot: &mut Robot,
        _robot_list: &Arc<RwLock<Vec<Arc<RwLock<Robot>>>>>,
        _robot_idx: usize,
    ) {
        self.last_state = robot.physics().read().unwrap().state(0.).clone();
    }

    fn get_observations(&mut self, robot: &mut Robot, time: f32) -> Vec<Observation> {
        let arc_physic = robot.physics();
        let physic = arc_physic.read().unwrap();
        let mut observation_list = Vec::<Observation>::new();
        if time < self.next_time_step() {
            return observation_list;
        }
        let state = physic.state(time);

        let dt = time - self.last_time;

        observation_list.push(Observation::Odometry(OdometryObservation {
            linear_velocity: state.velocity,
            angular_velocity: (state.pose.z - self.last_state.pose.z) / dt,
        }));
        for fault_model in &self.faults {
            fault_model.add_fault(time, observation_list.last_mut().unwrap());
        }

        self.last_time = time;
        self.last_state = state.clone();
        observation_list
    }

    fn next_time_step(&self) -> f32 {
        self.last_time + self.period
    }

    fn period(&self) -> f32 {
        self.period
    }
}

impl Stateful<SensorRecord> for OdometrySensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::OdometrySensor(OdometrySensorRecord {
            last_time: self.last_time,
            last_state: self.last_state.record(),
        })
    }

    fn from_record(&mut self, record: SensorRecord) {
        if let SensorRecord::OdometrySensor(odometry_record) = record {
            self.last_time = odometry_record.last_time;
            self.last_state.from_record(odometry_record.last_state);
        }
    }
}
