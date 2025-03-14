//! Misdetection faults
//!
//! Remark: the order of the application of the random value is alphabetical on the name of the observation variables if no order is specified.

use config_checker::macros::Check;
use log::debug;
use serde::{Deserialize, Serialize};

use crate::{
    sensors::sensor::Observation,
    utils::{
        determinist_random_variable::{
            DeterministRandomVariable, DeterministRandomVariableFactory,
        },
        distributions::bernouilli::{
            BernouilliRandomVariableConfig, DeterministBernouilliRandomVariable,
        },
    },
};

use super::fault_model::FaultModel;

#[derive(Debug, Serialize, Deserialize, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct MisdetectionFaultConfig {
    #[check(eq(self.apparition.probability.len(), 1))]
    pub apparition: BernouilliRandomVariableConfig,
}

impl Default for MisdetectionFaultConfig {
    fn default() -> Self {
        Self {
            apparition: BernouilliRandomVariableConfig {
                probability: vec![0.1],
                ..Default::default()
            },
        }
    }
}

#[derive(Debug)]
pub struct MisdetectionFault {
    apparition: DeterministBernouilliRandomVariable,
}

impl MisdetectionFault {
    pub fn from_config(
        config: &MisdetectionFaultConfig,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        Self {
            apparition: DeterministBernouilliRandomVariable::from_config(
                va_factory.global_seed,
                config.apparition.clone(),
            ),
        }
    }
}

impl FaultModel for MisdetectionFault {
    fn add_faults(
        &self,
        time: f32,
        period: f32,
        obs_list: &mut Vec<Observation>,
        _obs_type: Observation,
    ) {
        let obs_seed_increment = 1. / (100. * period);
        let mut seed = time;
        for i in (0..obs_list.len()).rev() {
            seed += obs_seed_increment;
            if self.apparition.gen(seed)[0] > 0. {
                debug!("Remove observation {i}");
                obs_list.remove(i);
            }
        }
    }
}

pub struct MisdetectionRecord {}
