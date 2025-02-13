use std::{
    fmt::Debug,
    sync::{Arc, Mutex},
};

use serde::{Deserialize, Serialize};

use crate::{sensors::sensor::Observation, simulator::SimulatorConfig, utils::{determinist_random_variable::{DeterministRandomVariable, DeterministRandomVariableFactory}, distributions::bernouilli::{BernouilliRandomVariableConfig, DeterministBernouilliRandomVariable}}};

use super::{
    noise::{NoiseFault, NoiseFaultConfig},
};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum FaultTypesConfig {
    Noise(NoiseFaultConfig),
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(default)]
pub struct FaultModelConfig {
    pub apparition: BernouilliRandomVariableConfig,
    pub fault: FaultTypesConfig,
}

impl Default for FaultModelConfig {
    fn default() -> Self {
        Self {
            apparition: BernouilliRandomVariableConfig {
                probability: vec![1.0],
                ..Default::default()
            },
            fault: FaultTypesConfig::Noise(NoiseFaultConfig::default()),
        }
    }
}

#[derive(Debug)]
pub struct FaultModel {
    apparition: DeterministBernouilliRandomVariable,
    fault: Arc<Mutex<Box<dyn FaultType>>>,
}

impl FaultModel {
    pub fn from_config(config: &FaultModelConfig, va_factory: &DeterministRandomVariableFactory) -> Self {
        Self {
            apparition: DeterministBernouilliRandomVariable::from_config(va_factory.global_seed, config.apparition.clone()),
            fault: Arc::new(Mutex::new(match &config.fault {
                FaultTypesConfig::Noise(cfg) => {
                    Box::new(NoiseFault::from_config(&cfg, va_factory)) as Box<dyn FaultType>
                }
            })),
        }
    }

    pub fn add_fault(&self, time: f32, obs: &mut Observation) {
        if self.apparition.gen(time)[0] > 0. {
            self.fault.lock().unwrap().add_fault(time, obs);
        }
    }
}

pub trait FaultType: Debug + Sync + Send {
    fn add_fault(&self, time: f32, obs: &mut Observation);
}
