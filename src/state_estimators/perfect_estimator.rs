// Configuration for PerfectPhysic
use serde_derive::{Serialize, Deserialize};
use super::state_estimator::{State, StateConfig};

#[derive(Serialize, Deserialize, Debug)]
#[serde(default)]
pub struct PerfectEstimatorConfig {
    
}

impl Default for PerfectEstimatorConfig {
    fn default() -> Self {
        Self {
        }
    }
}



use crate::physics::physic::{Command, Physic};

#[derive(Debug)]
pub struct PerfectEstimator {
    state: State
}

impl PerfectEstimator {
    pub fn new() -> Self {
        Self::from_config(&PerfectEstimatorConfig::default())
    }

    pub fn from_config(config: &PerfectEstimatorConfig) -> Self {
        Self {
            state: State::new()
        }
    }

    
}

use super::state_estimator::StateEstimator;

impl StateEstimator for PerfectEstimator {
    fn update_estimation(&mut self, time: f32, physic: &dyn Physic) {
        self.state = physic.state(time).clone();
    }

    fn state(&self) -> &State {
        &self.state
    }
}