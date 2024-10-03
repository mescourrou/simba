use nalgebra::Norm;
use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};
use statrs::distribution::Normal;

pub struct DeterministRandomVariableFactory {
    pub global_seed: f32,
}

impl DeterministRandomVariableFactory {
    pub fn new(global_seed: f32) -> Self {
        Self { global_seed }
    }

    pub fn make_variable(
        &self,
        config: RandomVariableTypeConfig,
    ) -> Box<dyn DeterministRandomVariable> {
        match config {
            RandomVariableTypeConfig::Uniform(c) => Box::new(
                DeterministUniformRandomVariable::from_config(self.global_seed, c),
            )
                as Box<dyn DeterministRandomVariable>,
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

pub trait DeterministRandomVariable {
    fn gen(&self, time: f32) -> f32;
}

#[derive(Serialize, Deserialize, Debug)]
pub enum RandomVariableTypeConfig {
    Uniform(UniformRandomVariableConfig),
    Normal(NormalRandomVariableConfig),
}

/*******************************************************************
 * Uniform
 *******************************************************************/

#[derive(Serialize, Deserialize, Debug)]
pub struct UniformRandomVariableConfig {
    min: f32,
    max: f32,
}

impl Default for UniformRandomVariableConfig {
    fn default() -> Self {
        Self { min: 0., max: 1. }
    }
}

pub struct DeterministUniformRandomVariable {
    global_seed: f32,
    min: f32,
    max: f32,
}

impl DeterministUniformRandomVariable {
    pub fn from_config(global_seed: f32, config: UniformRandomVariableConfig) -> Self {
        assert!(config.min <= config.max);
        Self {
            global_seed,
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

#[derive(Serialize, Deserialize, Debug)]
pub struct NormalRandomVariableConfig {
    mean: f32,
    variance: f32,
}

impl Default for NormalRandomVariableConfig {
    fn default() -> Self {
        Self {
            mean: 0.,
            variance: 1.,
        }
    }
}

pub struct DeterministNormalRandomVariable {
    global_seed: f32,
    nd: Normal,
}

impl DeterministNormalRandomVariable {
    pub fn from_config(global_seed: f32, config: NormalRandomVariableConfig) -> Self {
        Self {
            global_seed,
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
