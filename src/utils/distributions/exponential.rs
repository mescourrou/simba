use config_checker::macros::Check;
use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};
use statrs::distribution::Exp;

use crate::utils::determinist_random_variable::DeterministRandomVariable;

/// Configuration for a uniform random variable.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct ExponentialRandomVariableConfig {
    /// Random seed for this random variable.
    pub unique_seed: f32,
    /// Probabilities of the random variable
    pub lambda: Vec<f64>,
}

impl Default for ExponentialRandomVariableConfig {
    fn default() -> Self {
        Self {
            unique_seed: 0.,
            lambda: vec![1.],
        }
    }
}

#[derive(Debug)]
pub struct DeterministExponentialRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed from the configuration.
    global_seed: f32,
    /// Probability of the Exponential distribution.
    exponential: Vec<Exp>,
}

impl DeterministExponentialRandomVariable {
    pub fn from_config(global_seed: f32, config: ExponentialRandomVariableConfig) -> Self {
        assert!(config.lambda.iter().all(|x| *x >= 0.));
        Self {
            global_seed: global_seed + config.unique_seed,
            exponential: config
                .lambda
                .iter()
                .map(|lambda| Exp::new(*lambda).unwrap())
                .collect(),
        }
    }
}

impl DeterministRandomVariable for DeterministExponentialRandomVariable {
    fn gen(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.global_seed + time).to_bits() as u64);
        let mut v = Vec::new();
        for p in &self.exponential {
            v.push(p.sample(&mut rng) as f32);
        }
        v
    }

    fn dim(&self) -> usize {
        self.exponential.len()
    }
}
