// Configuration for PerfectPhysic
use serde_derive::{Serialize, Deserialize};
use super::state_estimator::{State, StateConfig};

#[derive(Serialize, Deserialize, Debug)]
#[serde(default)]
pub struct PerfectEstimatorConfig {
    update_period: f32
}

impl Default for PerfectEstimatorConfig {
    fn default() -> Self {
        Self {
            update_period: 0.1
        }
    }
}



use crate::physics::physic::{Command, Physic};

#[derive(Debug)]
pub struct PerfectEstimator {
    state: State,
    update_period: f32,
    last_time_update: f32
}

impl PerfectEstimator {
    pub fn new() -> Self {
        Self::from_config(&PerfectEstimatorConfig::default())
    }

    pub fn from_config(config: &PerfectEstimatorConfig) -> Self {
        Self {
            update_period: config.update_period,
            state: State::new(),
            last_time_update: 0.
        }
    }

    
}

use super::state_estimator::StateEstimator;

impl StateEstimator for PerfectEstimator {
    fn update_estimation(&mut self, time: f32, physic: &dyn Physic) {
        self.state = physic.state(time).clone();
        self.last_time_update = time;
    }

    fn state(&self) -> &State {
        &self.state
    }

    fn next_time_step(&self) -> f32 {
        self.last_time_update + self.update_period
    }
}