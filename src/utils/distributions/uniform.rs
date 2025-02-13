use std::iter::zip;

use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};

use crate::utils::determinist_random_variable::DeterministRandomVariable;

/// Configuration for a uniform random variable.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct UniformRandomVariableConfig {
    /// Random seed for this random variable.
    unique_seed: f32,
    /// Minimum value of the uniform distribution.
    min: Vec<f32>,
    /// Maximum value of the uniform distribution.
    max: Vec<f32>,
}

impl Default for UniformRandomVariableConfig {
    fn default() -> Self {
        Self {
            unique_seed: 0.,
            min: vec![-1.],
            max: vec![1.],
        }
    }
}

/// Random variable which return a random value between a min and a max, with a uniform distribution.
#[derive(Debug)]
pub struct DeterministUniformRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed from the configuration.
    global_seed: f32,
    /// Minimum value of the uniform distribution.
    min: Vec<f32>,
    /// Maximum value of the uniform distribution.
    max: Vec<f32>,
}

impl DeterministUniformRandomVariable {
    pub fn from_config(global_seed: f32, config: UniformRandomVariableConfig) -> Self {
        assert!(config.min <= config.max);
        Self {
            global_seed: global_seed + config.unique_seed,
            min: config.min,
            max: config.max,
        }
    }
}

impl DeterministRandomVariable for DeterministUniformRandomVariable {
    fn gen(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.global_seed + time).to_bits() as u64);
        let mut v = Vec::new();
        for (min, max) in zip(&self.min, &self.max) {
            v.push(min + rng.gen::<f32>() * (max - min));
        } 
        v
    }
}
