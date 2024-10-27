/*!
Provides a [`Sensor`] which can provide linear velocity and angular velocity.
*/

use std::sync::{Arc, RwLock};

use super::sensor::{Observation, Sensor, SensorRecord};

use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorMetaConfig;
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
    pub angular_velocity_noise: RandomVariableTypeConfig,
    pub linear_velocity_noise: RandomVariableTypeConfig,
}

impl Default for OdometrySensorConfig {
    fn default() -> Self {
        Self {
            period: 0.1,
            angular_velocity_noise: RandomVariableTypeConfig::None,
            linear_velocity_noise: RandomVariableTypeConfig::None,
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
#[derive(Debug)]
pub struct OdometryObservation {
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
    gen_angular_velocity: Box<dyn DeterministRandomVariable>,
    gen_linear_velocity: Box<dyn DeterministRandomVariable>,
}

impl OdometrySensor {
    /// Makes a new [`OdometrySensor`].
    pub fn new() -> Self {
        OdometrySensor::from_config(
            &OdometrySensorConfig::default(),
            &None,
            SimulatorMetaConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Makes a new [`OdometrySensor`] from the given config.
    pub fn from_config(
        config: &OdometrySensorConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        _meta_config: SimulatorMetaConfig,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        Self {
            last_state: State::new(),
            period: config.period,
            last_time: 0.,
            gen_angular_velocity: va_factory.make_variable(config.angular_velocity_noise.clone()),
            gen_linear_velocity: va_factory.make_variable(config.linear_velocity_noise.clone()),
        }
    }
}

use crate::turtlebot::Turtlebot;

impl Sensor for OdometrySensor {
    fn init(
        &mut self,
        turtle: &mut Turtlebot,
        _turtle_list: &Arc<RwLock<Vec<Arc<RwLock<Turtlebot>>>>>,
        _turtle_idx: usize,
    ) {
        self.last_state = turtle.physics().read().unwrap().state(0.).clone();
    }

    fn get_observations(&mut self, turtle: &mut Turtlebot, time: f32) -> Vec<Observation> {
        let arc_physic = turtle.physics();
        let physic = arc_physic.read().unwrap();
        let mut observation_list = Vec::<Observation>::new();
        if time < self.next_time_step() {
            return observation_list;
        }
        let state = physic.state(time);

        let dt = time - self.last_time;

        observation_list.push(Observation::Odometry(OdometryObservation {
            linear_velocity: state.velocity + self.gen_linear_velocity.gen(time),
            angular_velocity: (state.pose.z - self.last_state.pose.z) / dt
                + self.gen_angular_velocity.gen(time),
        }));

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
