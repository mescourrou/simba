use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};
use statrs::{distribution::MultivariateNormal, statistics::MeanN};

use crate::utils::determinist_random_variable::DeterministRandomVariable;

/// Configuration for a normal random variable.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct NormalRandomVariableConfig {
    /// Random seed for this random variable.
    unique_seed: f32,
    /// Mean of the normal distribution.
    mean: Vec<f64>,
    /// Variance of the normal distribution.
    covariance: Vec<f64>,
}

impl Default for NormalRandomVariableConfig {
    fn default() -> Self {
        Self {
            unique_seed: 0.,
            mean: vec![0.],
            covariance: vec![1.],
        }
    }
}

/// Random variable which return a random value following a normal distribution.
#[derive(Debug)]
pub struct DeterministNormalRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed from the configuration.
    global_seed: f32,
    /// Normal distribution.
    nd: MultivariateNormal,
}

impl DeterministNormalRandomVariable {
    pub fn from_config(global_seed: f32, config: NormalRandomVariableConfig) -> Self {
        assert!(
            config.mean.len().pow(2) == config.covariance.len(),
            "The length of the covariance vector should be the square of the means' one."
        );
        Self {
            global_seed: global_seed + config.unique_seed,
            nd: MultivariateNormal::new(config.mean, config.covariance)
                .expect("Impossible to create the normal distribution"),
        }
    }
}

impl DeterministRandomVariable for DeterministNormalRandomVariable {
    fn gen(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.global_seed + time).to_bits() as u64);
        self.nd.sample(&mut rng).iter().map(|x| *x as f32).collect()
    }

    fn dim(&self) -> usize {
        self.nd.mean().unwrap().len()
    }
}
