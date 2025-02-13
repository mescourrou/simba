use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};

use crate::utils::determinist_random_variable::DeterministRandomVariable;

/// Configuration for a uniform random variable.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct BernouilliRandomVariableConfig {
    /// Random seed for this random variable.
    pub unique_seed: f32,
    /// Probabilities of the random variable
    pub probability: Vec<f32>,
}

impl Default for BernouilliRandomVariableConfig {
    fn default() -> Self {
        Self {
            unique_seed: 0.,
            probability: vec![1.],
        }
    }
}

#[derive(Debug)]
pub struct DeterministBernouilliRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed from the configuration.
    global_seed: f32,
    /// Probability of the bernouilli distribution.
    probability: Vec<f32>,
}

impl DeterministBernouilliRandomVariable {
    pub fn from_config(global_seed: f32, config: BernouilliRandomVariableConfig) -> Self {
        assert!(config.probability.iter().all(|x| *x <= 1.));
        assert!(config.probability.iter().all(|x| *x >= 0.));
        Self {
            global_seed: global_seed + config.unique_seed,
            probability: config.probability,
        }
    }
}

impl DeterministRandomVariable for DeterministBernouilliRandomVariable {
    fn gen(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.global_seed + time).to_bits() as u64);
        let mut v = Vec::new();
        for p in &self.probability {
            v.push(if rng.gen::<f32>() <= *p { 
                1.
            } else {
                0.
            });
        } 
        v
    }
}
