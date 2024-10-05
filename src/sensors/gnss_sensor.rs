/*!
Provides a [`Sensor`] which can provide position and velocity in the global frame.
*/

use std::sync::{Arc, RwLock};

use super::sensor::{GenericObservation, Sensor, SensorRecord};

use crate::plugin_api::PluginAPI;
use crate::simulator::{Simulator, SimulatorMetaConfig};
use crate::state_estimators::state_estimator::{State, StateRecord};
use crate::stateful::Stateful;
use crate::utils::determinist_random_variable::{
    DeterministFixedRandomVariable, DeterministRandomVariable, DeterministRandomVariableFactory,
    FixedRandomVariableConfig, RandomVariableTypeConfig,
};
use nalgebra::Vector2;
use serde_derive::{Deserialize, Serialize};

extern crate nalgebra as na;

/// Configuration of the [`GNSSSensor`].
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct GNSSSensorConfig {
    /// Observation period of the sensor.
    pub period: f32,
    pub pose_x_noise: RandomVariableTypeConfig,
    pub pose_y_noise: RandomVariableTypeConfig,
    pub velocity_x_noise: RandomVariableTypeConfig,
    pub velocity_y_noise: RandomVariableTypeConfig,
}

impl Default for GNSSSensorConfig {
    fn default() -> Self {
        Self {
            period: 1.,
            pose_x_noise: RandomVariableTypeConfig::None,
            pose_y_noise: RandomVariableTypeConfig::None,
            velocity_x_noise: RandomVariableTypeConfig::None,
            velocity_y_noise: RandomVariableTypeConfig::None,
        }
    }
}

/// Record of the [`GNSSSensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct GNSSSensorRecord {
    last_time: f32,
}

impl Default for GNSSSensorRecord {
    fn default() -> Self {
        Self { last_time: 0. }
    }
}

/// Observation of the odometry.
#[derive(Debug)]
pub struct GNSSObservation {
    pub position: Vector2<f32>,
    pub velocity: Vector2<f32>,
}

impl GenericObservation for GNSSObservation {}

/// Sensor which observes the robot's odometry
#[derive(Debug)]
pub struct GNSSSensor {
    /// Observation period
    period: f32,
    /// Last observation time.
    last_time: f32,
    gen_x_pose: Box<dyn DeterministRandomVariable>,
    gen_y_pose: Box<dyn DeterministRandomVariable>,
    gen_x_velocity: Box<dyn DeterministRandomVariable>,
    gen_y_velocity: Box<dyn DeterministRandomVariable>,
}

impl GNSSSensor {
    /// Makes a new [`GNSSSensor`].
    pub fn new() -> Self {
        GNSSSensor::from_config(
            &GNSSSensorConfig::default(),
            &None,
            SimulatorMetaConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Makes a new [`GNSSSensor`] from the given config.
    pub fn from_config(
        config: &GNSSSensorConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        _meta_config: SimulatorMetaConfig,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        Self {
            period: config.period,
            last_time: 0.,
            gen_x_pose: va_factory.make_variable(config.pose_x_noise.clone()),
            gen_y_pose: va_factory.make_variable(config.pose_y_noise.clone()),
            gen_x_velocity: va_factory.make_variable(config.velocity_x_noise.clone()),
            gen_y_velocity: va_factory.make_variable(config.velocity_y_noise.clone()),
        }
    }
}

use crate::turtlebot::Turtlebot;

impl Sensor for GNSSSensor {
    fn init(&mut self, turtle: &mut Turtlebot) {
        
    }

    fn get_observations(
        &mut self,
        turtle: &mut Turtlebot,
        time: f32,
        turtle_list: &Arc<RwLock<Vec<Arc<RwLock<Turtlebot>>>>>,
    ) -> Vec<Box<dyn GenericObservation>> {
        let arc_physic = turtle.physics();
        let physic = arc_physic.read().unwrap();
        let mut observation_list = Vec::<Box<dyn GenericObservation>>::new();
        if time < self.next_time_step() {
            return observation_list;
        }
        let state = physic.state(time);

        let pose_noise = Vector2::<f32>::from_vec(vec![
            self.gen_x_pose.gen(time),
            self.gen_y_pose.gen(time)
        ]);

        let velocity_noise = Vector2::<f32>::from_vec(vec![
            self.gen_x_velocity.gen(time),
            self.gen_y_velocity.gen(time)
        ]);

        let velocity = Vector2::<f32>::from_vec(vec![
            state.velocity * state.pose.z.cos(),
            state.velocity * state.pose.z.sin(),
        ]);

        observation_list.push(Box::new(GNSSObservation {
            position: state.pose.fixed_view(0, 0) + pose_noise,
            velocity: velocity + velocity_noise,
        }));

        observation_list
    }

    fn next_time_step(&self) -> f32 {
        self.last_time + self.period
    }

    fn period(&self) -> f32 {
        self.period
    }
}

impl Stateful<SensorRecord> for GNSSSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::GNSSSensor(GNSSSensorRecord {
            last_time: self.last_time,
        })
    }

    fn from_record(&mut self, record: SensorRecord) {
        if let SensorRecord::GNSSSensor(gnss_record) = record {
            self.last_time = gnss_record.last_time;
        }
    }
}
