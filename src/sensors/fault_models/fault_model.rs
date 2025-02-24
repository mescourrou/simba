//! TODO: Lots of code duplication between fault models, need to find a way to factorize

use std::{
    fmt::Debug,
    sync::{Arc, Mutex},
};

use config_checker::macros::Check;
use serde::{Deserialize, Serialize};

use crate::{
    sensors::sensor::Observation,
    simulator::SimulatorConfig,
    utils::{
        determinist_random_variable::{
            DeterministRandomVariable, DeterministRandomVariableFactory,
        },
        distributions::bernouilli::{
            BernouilliRandomVariableConfig, DeterministBernouilliRandomVariable,
        },
    },
};

use super::{
    additive_observation_centered_polar::{
        AdditiveObservationCenteredPolarFault, AdditiveObservationCenteredPolarFaultConfig,
    },
    additive_robot_centered::{AdditiveRobotCenteredFault, AdditiveRobotCenteredFaultConfig},
    additive_robot_centered_polar::{
        AdditiveRobotCenteredPolarFault, AdditiveRobotCenteredPolarFaultConfig,
    },
    clutter::{ClutterFault, ClutterFaultConfig},
    misassociation::{MisassociationFault, MisassociationFaultConfig},
    misdetection::{MisdetectionFault, MisdetectionFaultConfig},
};

#[derive(Debug, Serialize, Deserialize, Clone, Check)]
#[serde(deny_unknown_fields)]
pub enum FaultModelConfig {
    AdditiveRobotCentered(AdditiveRobotCenteredFaultConfig),
    AdditiveRobotCenteredPolar(AdditiveRobotCenteredPolarFaultConfig),
    AdditiveObservationCenteredPolar(AdditiveObservationCenteredPolarFaultConfig),
    Clutter(ClutterFaultConfig),
    Misdetection(MisdetectionFaultConfig),
    Misassociation(MisassociationFaultConfig),
}

pub fn make_fault_model_from_config(
    config: &FaultModelConfig,
    global_config: &SimulatorConfig,
    robot_name: &String,
    va_factory: &DeterministRandomVariableFactory,
) -> Box<dyn FaultModel> {
    match &config {
        FaultModelConfig::AdditiveRobotCentered(cfg) => {
            Box::new(AdditiveRobotCenteredFault::from_config(&cfg, va_factory))
                as Box<dyn FaultModel>
        }
        FaultModelConfig::AdditiveRobotCenteredPolar(cfg) => Box::new(
            AdditiveRobotCenteredPolarFault::from_config(&cfg, va_factory),
        ) as Box<dyn FaultModel>,
        FaultModelConfig::AdditiveObservationCenteredPolar(cfg) => Box::new(
            AdditiveObservationCenteredPolarFault::from_config(&cfg, va_factory),
        ) as Box<dyn FaultModel>,
        FaultModelConfig::Clutter(cfg) => {
            Box::new(ClutterFault::from_config(&cfg, va_factory)) as Box<dyn FaultModel>
        }
        FaultModelConfig::Misdetection(cfg) => {
            Box::new(MisdetectionFault::from_config(&cfg, va_factory)) as Box<dyn FaultModel>
        }
        FaultModelConfig::Misassociation(cfg) => Box::new(MisassociationFault::from_config(
            &cfg,
            global_config,
            robot_name,
            va_factory,
        )) as Box<dyn FaultModel>,
    }
}

pub trait FaultModel: Debug + Sync + Send {
    fn add_faults(
        &self,
        time: f32,
        period: f32,
        obs_list: &mut Vec<Observation>,
        obs_type: Observation,
    );
}
