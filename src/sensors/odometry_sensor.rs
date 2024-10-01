/*!
Provides a [`Sensor`] which can provide linear velocity and angular velocity.
*/

use super::sensor::{GenericObservation, Sensor, SensorRecord};

use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorMetaConfig;
use crate::state_estimators::state_estimator::{State, StateRecord};
use crate::stateful::Stateful;
use serde_derive::{Deserialize, Serialize};

extern crate nalgebra as na;

/// Configuration of the [`OdometrySensor`].
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct OdometrySensorConfig {
    /// Observation period of the sensor.
    pub period: f32,
}

impl Default for OdometrySensorConfig {
    fn default() -> Self {
        Self { period: 0.1 }
    }
}

/// Record of the [`OdometrySensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone)]
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

impl GenericObservation for OdometryObservation {}

/// Sensor which observes the robot's odometry
#[derive(Debug)]
pub struct OdometrySensor {
    /// Last state to compute the velocity.
    last_state: State,
    /// Observation period
    period: f32,
    /// Last observation time.
    last_time: f32,
}

impl OdometrySensor {
    /// Makes a new [`OdometrySensor`].
    pub fn new() -> Self {
        OdometrySensor::from_config(
            &OdometrySensorConfig::default(),
            &None,
            SimulatorMetaConfig::default(),
        )
    }

    /// Makes a new [`OdometrySensor`] from the given config.
    pub fn from_config(
        config: &OdometrySensorConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        _meta_config: SimulatorMetaConfig,
    ) -> Self {
        Self {
            last_state: State::new(),
            period: config.period,
            last_time: 0.,
        }
    }
}

use crate::turtlebot::Turtlebot;

impl Sensor for OdometrySensor {
    fn init(&mut self, turtle: &mut Turtlebot) {
        self.last_state = turtle.physics().read().unwrap().state(0.).clone();
    }

    fn get_observations(
        &mut self,
        turtle: &mut Turtlebot,
        time: f32,
    ) -> Vec<Box<dyn GenericObservation>> {
        let arc_physic = turtle.physics();
        let physic = arc_physic.read().unwrap();
        let mut observation_list = Vec::<Box<dyn GenericObservation>>::new();
        if time < self.next_time_step() {
            return observation_list;
        }
        let state = physic.state(time);

        let dt = time - self.last_time;

        observation_list.push(Box::new(OdometryObservation {
            linear_velocity: state.velocity,
            angular_velocity: (state.pose.z - self.last_state.pose.z) / dt,
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
