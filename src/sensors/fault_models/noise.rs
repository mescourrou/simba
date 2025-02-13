use std::sync::{Arc, Mutex};

use serde::{Deserialize, Serialize};

use crate::{sensors::sensor::Observation, simulator::SimulatorConfig, utils::{determinist_random_variable::{DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig}, distributions::normal::NormalRandomVariableConfig}};

use super::{
    fault_model::FaultType,
};

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(default)]
pub struct NoiseFaultConfig {
    pub distribution: RandomVariableTypeConfig,
}

impl Default for NoiseFaultConfig {
    fn default() -> Self {
        Self {
            distribution: RandomVariableTypeConfig::Normal(
                NormalRandomVariableConfig::default(),
            ),
        }
    }
}

#[derive(Debug)]
pub struct NoiseFault {
    distribution: Arc<Mutex<Box<dyn DeterministRandomVariable>>>,
}

impl NoiseFault {
    pub fn from_config(config: &NoiseFaultConfig, va_factory: &DeterministRandomVariableFactory) -> Self {
        Self {
            distribution: Arc::new(Mutex::new(va_factory.make_variable(config.distribution.clone()))),
        }
    }
}

impl FaultType for NoiseFault {
    fn add_fault(&self, time: f32, obs: &mut Observation) {
        let random_sample = self.distribution.lock().unwrap().gen(time);
        match obs {
            Observation::OrientedRobot(o) => {
                o.pose.x += random_sample[0];
                o.pose.y += random_sample[1];
                o.pose.z += random_sample[2];
            },
            _ => {},
        }
    }
}

pub struct NoiseRecord {}
