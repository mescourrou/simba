extern crate nalgebra as na;
use na::{SVector};

extern crate confy;
use serde_derive::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct StateConfig {
    pose: Vec<f32>,
    velocity: f32
}

impl Default for StateConfig {
    fn default() -> Self {
        Self {
            pose: vec![0., 0., 0.],
            velocity: 0.
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct StateRecord {
    pose: Vec<f32>,
    velocity: f32
}


#[derive(Debug, Clone)]
pub struct State {
    pub pose: SVector<f32, 3>,
    pub velocity: f32
}

impl State {
    pub fn new() -> Self {
        Self {
            pose: SVector::<f32,3>::new(0., 0., 0.),
            velocity: 0.
        }
    }

    pub fn from_config(config: &StateConfig) -> Self {
        let mut state = Self::new();
        
        let mut i:usize = 0;
        for coord in &config.pose {
            if i >= 3 {
                break;
            }
            state.pose[i] = *coord;
            i += 1;
        }
        state.velocity = config.velocity;
        return state;
    }

    pub fn record(&self) -> StateRecord {
        StateRecord {
            pose: {
                let mut ve: Vec<f32> = vec![];
                for coord in &self.pose {
                    ve.push(*coord);
                }
                ve
            },
            velocity: self.velocity
        }
    }
}

use std::fmt;

impl fmt::Display for State {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(formatter, "pose: [{}, {}, {}], v: {}", self.pose.x, self.pose.y, self.pose.z, self.velocity)?;
        Ok(())
    }
}


use super::{perfect_estimator, external_estimator};
use crate::turtlebot::Turtlebot;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum StateEstimatorConfig {
    Perfect(Box<perfect_estimator::PerfectEstimatorConfig>),
    External(Box<external_estimator::ExternalEstimatorConfig>)
}

#[derive(Serialize, Deserialize, Debug)]
pub enum StateEstimatorRecord {
    Perfect(perfect_estimator::PerfectEstimatorRecord),
    External(external_estimator::ExternalEstimatorRecord)
}
use crate::physics::physic::Physic;

use crate::sensors::sensor::GenericObservation;

pub trait StateEstimator : std::fmt::Debug  + std::marker::Send + std::marker::Sync {
    fn prediction_step(&mut self, turtle: &mut Turtlebot, time: f32);
    fn correction_step(&mut self, turtle: &mut Turtlebot, observations: Vec<Box<dyn GenericObservation>>, time: f32);
    fn state(&self) -> State;
    fn next_time_step(&self) -> f32;
    fn record(&self) ->  StateEstimatorRecord;
}