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

use config_checker::macros::Check;
use serde::{Deserialize, Serialize};

use super::distributions::{
    exponential::{DeterministExponentialRandomVariable, ExponentialRandomVariableConfig},
    fixed::{DeterministFixedRandomVariable, FixedRandomVariableConfig},
    normal::{DeterministNormalRandomVariable, NormalRandomVariableConfig},
    poisson::{DeterministPoissonRandomVariable, PoissonRandomVariableConfig},
    uniform::{DeterministUniformRandomVariable, UniformRandomVariableConfig},
};

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
            RandomVariableTypeConfig::Poisson(c) => Box::new(
                DeterministPoissonRandomVariable::from_config(self.global_seed, c),
            ),
            RandomVariableTypeConfig::Exponential(c) => Box::new(
                DeterministExponentialRandomVariable::from_config(self.global_seed, c),
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
    fn gen(&self, time: f32) -> Vec<f32>;
    fn dim(&self) -> usize;
}

/// Configuration of the random variable: fixed, uniform or normal.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(deny_unknown_fields)]
pub enum RandomVariableTypeConfig {
    /// No random variable
    None,
    /// Fixed value
    Fixed(FixedRandomVariableConfig),
    /// Uniform distribution
    Uniform(UniformRandomVariableConfig),
    /// Normal distribution
    Normal(NormalRandomVariableConfig),
    Poisson(PoissonRandomVariableConfig),
    Exponential(ExponentialRandomVariableConfig),
}
