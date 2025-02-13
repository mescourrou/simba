//! TODO: Lots of code duplication between fault models, need to find a way to factorize

use std::{
    fmt::Debug,
    sync::{Arc, Mutex},
};

use serde::{Deserialize, Serialize};

use crate::{sensors::sensor::Observation, simulator::SimulatorConfig, utils::{determinist_random_variable::{DeterministRandomVariable, DeterministRandomVariableFactory}, distributions::bernouilli::{BernouilliRandomVariableConfig, DeterministBernouilliRandomVariable}}};

use super::{additive_observation_centered_polar::{AdditiveObservationCenteredPolarFault, AdditiveObservationCenteredPolarFaultConfig}, additive_robot_centered::{AdditiveRobotCenteredFault, AdditiveRobotCenteredFaultConfig}, additive_robot_centered_polar::{AdditiveRobotCenteredPolarFault, AdditiveRobotCenteredPolarFaultConfig}};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum FaultModelConfig {
    AdditiveRobotCentered(AdditiveRobotCenteredFaultConfig),
    AdditiveRobotCenteredPolar(AdditiveRobotCenteredPolarFaultConfig),
    AdditiveObservationCenteredPolar(AdditiveObservationCenteredPolarFaultConfig),
}

pub fn make_fault_model_from_config(config: &FaultModelConfig, va_factory: &DeterministRandomVariableFactory) -> Box<dyn FaultModel> {
    match &config {
        FaultModelConfig::AdditiveRobotCentered(cfg) => {
            Box::new(AdditiveRobotCenteredFault::from_config(&cfg, va_factory)) as Box<dyn FaultModel>
        },
        FaultModelConfig::AdditiveRobotCenteredPolar(cfg) => {
            Box::new(AdditiveRobotCenteredPolarFault::from_config(&cfg, va_factory)) as Box<dyn FaultModel>
        }
        FaultModelConfig::AdditiveObservationCenteredPolar(cfg) => {
            Box::new(AdditiveObservationCenteredPolarFault::from_config(&cfg, va_factory)) as Box<dyn FaultModel>
        }
    }
}

pub trait FaultModel: Debug + Sync + Send {
    fn add_faults(&self, time: f32, period: f32, obs_list: &mut Vec<Observation>);
}
