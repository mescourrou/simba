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

pub trait DeterministRandomVariable:
    std::fmt::Debug + std::marker::Send + std::marker::Sync
{
    fn gen(&self, time: f32) -> f32;
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum RandomVariableTypeConfig {
    None,
    Fixed(FixedRandomVariableConfig),
    Uniform(UniformRandomVariableConfig),
    Normal(NormalRandomVariableConfig),
}

/*******************************************************************
 * Fixed
*******************************************************************/

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct FixedRandomVariableConfig {
    value: f32,
}

impl Default for FixedRandomVariableConfig {
    fn default() -> Self {
        Self { value: 0. }
    }
}

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

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct UniformRandomVariableConfig {
    unique_seed: f32,
    min: f32,
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

#[derive(Debug)]
pub struct DeterministUniformRandomVariable {
    global_seed: f32,
    min: f32,
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

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct NormalRandomVariableConfig {
    unique_seed: f32,
    mean: f32,
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

#[derive(Debug)]
pub struct DeterministNormalRandomVariable {
    global_seed: f32,
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
