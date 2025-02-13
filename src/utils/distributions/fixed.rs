use serde::{Deserialize, Serialize};

use crate::utils::determinist_random_variable::DeterministRandomVariable;

/// Configuration for a fixed random variable.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct FixedRandomVariableConfig {
    /// Fixed value to return.
    values: Vec<f32>,
}

impl Default for FixedRandomVariableConfig {
    fn default() -> Self {
        Self { 
            values: vec![0.],
        }
    }
}

/// Random variable which always return the same value.
#[derive(Debug)]
pub struct DeterministFixedRandomVariable {
    values: Vec<f32>,
}

impl DeterministFixedRandomVariable {
    pub fn from_config(_global_seed: f32, config: FixedRandomVariableConfig) -> Self {
        Self {
            values: config.values,
        }
    }
}

impl DeterministRandomVariable for DeterministFixedRandomVariable {
    fn gen(&self, _time: f32) -> Vec<f32> {
        self.values.clone()
    }

    fn dim(&self) -> usize {
        self.values.len()
    }
}
