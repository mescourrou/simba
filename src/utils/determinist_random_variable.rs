/*!
# Determinist Random Variable

This module provides a way to create random variables with a deterministic behavior. This is
useful to have a reproducible behavior.

This module proposes a factory to create random variables with a deterministic behavior. The
factory is created with a global seed, and each random variable created with this factory will
have a unique seed, to have a different series of random numbers.

The random variables can be of different types:
- Fixed: always return the same value
- Uniform: return a random value between a min and a max, with a uniform distribution
- Normal: return a random value following a normal distribution
Other types can be added in the future.

 */

use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};
use statrs::distribution::Normal;

/// Factory to create random variables with a deterministic behavior, using a global seed.
pub struct DeterministRandomVariableFactory {
    /// Global run seed.
    pub global_seed: f32,
}

impl DeterministRandomVariableFactory {
    /// Create a new factory with the given `global_seed`.
    pub fn new(global_seed: f32) -> Self {
        Self { global_seed }
    }

    /// Create a new random variable with the given configuration.
    pub fn make_variable(
        &self,
        config: RandomVariableTypeConfig,
    ) -> Box<dyn DeterministRandomVariable> {
        match config {
            RandomVariableTypeConfig::None => {
                Box::new(DeterministFixedRandomVariable::from_config(
                    self.global_seed,
                    FixedRandomVariableConfig::default(),
                ))
            }
            RandomVariableTypeConfig::Fixed(c) => Box::new(
                DeterministFixedRandomVariable::from_config(self.global_seed, c),
            ),
            RandomVariableTypeConfig::Uniform(c) => Box::new(
                DeterministUniformRandomVariable::from_config(self.global_seed, c),
            ),
            RandomVariableTypeConfig::Normal(c) => Box::new(
                DeterministNormalRandomVariable::from_config(self.global_seed, c),
            ),
        }
    }
}

impl Default for DeterministRandomVariableFactory {
    fn default() -> Self {
        Self { global_seed: 0. }
    }
}

/// Trait for a random variable with a deterministic behavior.
pub trait DeterministRandomVariable:
    std::fmt::Debug + std::marker::Send + std::marker::Sync
{
    fn gen(&self, time: f32) -> f32;
}

/// Configuration of the random variable: fixed, uniform or normal.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum RandomVariableTypeConfig {
    /// No random variable
    None,
    /// Fixed value
    Fixed(FixedRandomVariableConfig),
    /// Uniform distribution
    Uniform(UniformRandomVariableConfig),
    /// Normal distribution
    Normal(NormalRandomVariableConfig),
}

/*******************************************************************
 * Fixed
*******************************************************************/

/// Configuration for a fixed random variable.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct FixedRandomVariableConfig {
    /// Fixed value to return.
    value: f32,
}

impl Default for FixedRandomVariableConfig {
    fn default() -> Self {
        Self { value: 0. }
    }
}

/// Random variable which always return the same value.
#[derive(Debug)]
pub struct DeterministFixedRandomVariable {
    value: f32,
}

impl DeterministFixedRandomVariable {
    pub fn from_config(_global_seed: f32, config: FixedRandomVariableConfig) -> Self {
        Self {
            value: config.value,
        }
    }
}

impl DeterministRandomVariable for DeterministFixedRandomVariable {
    fn gen(&self, _time: f32) -> f32 {
        self.value
    }
}

/*******************************************************************
 * Uniform
 *******************************************************************/

/// Configuration for a uniform random variable.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct UniformRandomVariableConfig {
    /// Random seed for this random variable.
    unique_seed: f32,
    /// Minimum value of the uniform distribution.
    min: f32,
    /// Maximum value of the uniform distribution.
    max: f32,
}

impl Default for UniformRandomVariableConfig {
    fn default() -> Self {
        Self {
            unique_seed: 0.,
            min: -1.,
            max: 1.,
        }
    }
}

/// Random variable which return a random value between a min and a max, with a uniform distribution.
#[derive(Debug)]
pub struct DeterministUniformRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed from the configuration.
    global_seed: f32,
    /// Minimum value of the uniform distribution.
    min: f32,
    /// Maximum value of the uniform distribution.
    max: f32,
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
    fn gen(&self, time: f32) -> f32 {
        let mut rng = ChaCha8Rng::seed_from_u64((self.global_seed + time).to_bits() as u64);
        self.min + rng.gen::<f32>() * (self.max - self.min)
    }
}

/*******************************************************************
 * Normal
 *******************************************************************/

 /// Configuration for a normal random variable.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct NormalRandomVariableConfig {
    /// Random seed for this random variable.
    unique_seed: f32,
    /// Mean of the normal distribution.
    mean: f32,
    /// Variance of the normal distribution.
    variance: f32,
}

impl Default for NormalRandomVariableConfig {
    fn default() -> Self {
        Self {
            unique_seed: 0.,
            mean: 0.,
            variance: 1.,
        }
    }
}

/// Random variable which return a random value following a normal distribution.
#[derive(Debug)]
pub struct DeterministNormalRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed from the configuration.
    global_seed: f32,
    /// Normal distribution.
    nd: Normal,
}

impl DeterministNormalRandomVariable {
    pub fn from_config(global_seed: f32, config: NormalRandomVariableConfig) -> Self {
        Self {
            global_seed: global_seed + config.unique_seed,
            nd: Normal::new(config.mean.into(), config.variance.sqrt().into())
                .expect("Impossible to create the normal distribution"),
        }
    }
}

impl DeterministRandomVariable for DeterministNormalRandomVariable {
    fn gen(&self, time: f32) -> f32 {
        let mut rng = ChaCha8Rng::seed_from_u64((self.global_seed + time).to_bits() as u64);
        self.nd.sample(&mut rng) as f32
    }
}
