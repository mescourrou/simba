// Configuration for PerfectPhysic
use serde_derive::{Serialize, Deserialize};

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


use super::state_estimator::State;

#[derive(Debug)]
pub struct PerfectEstimator {
    
}

impl PerfectEstimator {
    pub fn new() -> Self {
        Self::from_config(&PerfectEstimatorConfig::default())
    }

    pub fn from_config(config: &PerfectEstimatorConfig) -> Self {
        Self {
        }
    }
}

use super::state_estimator::StateEstimator;

impl StateEstimator for PerfectEstimator {
    
}