use config_checker::macros::Check;
use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};
use statrs::distribution::Poisson;

use crate::utils::determinist_random_variable::DeterministRandomVariable;

/// Configuration for a uniform random variable.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
pub struct PoissonRandomVariableConfig {
    /// Random seed for this random variable.
    pub unique_seed: f32,
    /// Probabilities of the random variable
    pub lambda: Vec<f64>,
}

impl Default for PoissonRandomVariableConfig {
    fn default() -> Self {
        Self {
            unique_seed: 0.,
            lambda: vec![1.],
        }
    }
}

#[derive(Debug)]
pub struct DeterministPoissonRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed from the configuration.
    global_seed: f32,
    /// Probability of the Poisson distribution.
    poisson: Vec<Poisson>,
}

impl DeterministPoissonRandomVariable {
    pub fn from_config(global_seed: f32, config: PoissonRandomVariableConfig) -> Self {
        assert!(config.lambda.iter().all(|x| *x >= 0.));
        Self {
            global_seed: global_seed + config.unique_seed,
            poisson: config
                .lambda
                .iter()
                .map(|lambda| Poisson::new(*lambda).unwrap())
                .collect(),
        }
    }
}

impl DeterministRandomVariable for DeterministPoissonRandomVariable {
    fn gen(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.global_seed + time).to_bits() as u64);
        let mut v = Vec::new();
        for p in &self.poisson {
            v.push(p.sample(&mut rng) as f32);
        }
        v
    }

    fn dim(&self) -> usize {
        self.poisson.len()
    }
}
