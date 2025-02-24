//! AdditiveRobotCenteredPolarPolar faults
//!
//! Remark: the order of the application of the random value is alphabetical on the name of the observation variables if no order is specified.

use std::sync::{Arc, Mutex};

use config_checker::macros::Check;
use libm::atan2f;
use rand::random;
use serde::{Deserialize, Serialize};

use crate::{
    sensors::sensor::Observation,
    simulator::SimulatorConfig,
    utils::{
        determinist_random_variable::{
            DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig,
        },
        distributions::{
            bernouilli::{BernouilliRandomVariableConfig, DeterministBernouilliRandomVariable},
            normal::NormalRandomVariableConfig,
        },
        geometry::mod2pi,
    },
};

use super::fault_model::FaultModel;

#[derive(Debug, Serialize, Deserialize, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct AdditiveRobotCenteredPolarFaultConfig {
    #[check(eq(self.apparition.probability.len(), 1))]
    #[check]
    pub apparition: BernouilliRandomVariableConfig,
    #[check]
    pub distributions: Vec<RandomVariableTypeConfig>,
    pub variable_order: Vec<String>,
}

impl Default for AdditiveRobotCenteredPolarFaultConfig {
    fn default() -> Self {
        Self {
            apparition: BernouilliRandomVariableConfig {
                probability: vec![1.0],
                ..Default::default()
            },
            distributions: vec![RandomVariableTypeConfig::Normal(
                NormalRandomVariableConfig::default(),
            )],
            variable_order: Vec::new(),
        }
    }
}

#[derive(Debug)]
pub struct AdditiveRobotCenteredPolarFault {
    apparition: DeterministBernouilliRandomVariable,
    distributions: Arc<Mutex<Vec<Box<dyn DeterministRandomVariable>>>>,
    variable_order: Vec<String>,
}

impl AdditiveRobotCenteredPolarFault {
    pub fn from_config(
        config: &AdditiveRobotCenteredPolarFaultConfig,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        let distributions = Arc::new(Mutex::new(
            config
                .distributions
                .iter()
                .map(|conf| va_factory.make_variable(conf.clone()))
                .collect::<Vec<Box<dyn DeterministRandomVariable>>>(),
        ));
        if config.variable_order.len() != 0 {
            assert!(
                config.variable_order.len()
                    == distributions
                        .lock()
                        .unwrap()
                        .iter()
                        .map(|d| d.dim())
                        .sum::<usize>(),
                "If variable order is given, its length must match the distribution dimension."
            );
        }
        Self {
            apparition: DeterministBernouilliRandomVariable::from_config(
                va_factory.global_seed,
                config.apparition.clone(),
            ),
            distributions,
            variable_order: config.variable_order.clone(),
        }
    }
}

impl FaultModel for AdditiveRobotCenteredPolarFault {
    fn add_faults(
        &self,
        time: f32,
        period: f32,
        obs_list: &mut Vec<Observation>,
        obs_type: Observation,
    ) {
        let obs_seed_increment = 1. / (100. * period);
        let mut seed = time;
        for obs in obs_list {
            seed += obs_seed_increment;
            if self.apparition.gen(seed)[0] < 1. {
                continue;
            }
            let mut random_sample = Vec::new();
            for d in self.distributions.lock().unwrap().iter() {
                random_sample.extend_from_slice(&d.gen(seed));
            }
            match obs {
                Observation::OrientedRobot(o) => {
                    let mut r_add = 0.;
                    let mut z_add = 0.;
                    let mut theta_add = 0.;
                    if self.variable_order.len() > 0 {
                        for i in 0..self.variable_order.len() {
                            match self.variable_order[i].as_str() {
                                "r" => r_add = random_sample[i],
                                "theta" => theta_add = random_sample[i],
                                "z | orientation" => z_add = random_sample[i],
                                &_ => panic!("Unknown variable name: '{}'. Available variable names: [r, theta, z | orientation]", self.variable_order[i])
                            }
                        }
                    } else {
                        assert!(random_sample.len() >= 3, "The distribution of an AdditiveRobotCenteredPolar fault for OrientedRobot observation need to be of dimension 3.");
                        r_add = random_sample[0];
                        theta_add = random_sample[1];
                        z_add = random_sample[2];
                    }
                    let current_r = (o.pose.x.powi(2) + o.pose.y.powi(2)).sqrt();
                    let current_dir = atan2f(o.pose.y, o.pose.x);
                    let r = current_r + r_add;
                    let theta = current_dir + theta_add;
                    let z = o.pose.z + z_add;
                    o.pose.x = r * theta.cos();
                    o.pose.y = r * theta.sin();
                    o.pose.z = mod2pi(z);
                }
                Observation::GNSS(o) => {
                    panic!("Not implemented yet (need to find a logical way to do it.");
                }
                Observation::Odometry(o) => {
                    panic!("Not implemented (appropriated for this sensor?)");
                }
                Observation::OrientedLandmark(o) => {
                    let mut r_add = 0.;
                    let mut z_add = 0.;
                    let mut theta_add = 0.;
                    if self.variable_order.len() > 0 {
                        for i in 0..self.variable_order.len() {
                            match self.variable_order[i].as_str() {
                                "r" => r_add = random_sample[i],
                                "theta" => theta_add = random_sample[i],
                                "z | orientation" => z_add = random_sample[i],
                                &_ => panic!("Unknown variable name: '{}'. Available variable names: [r, theta, z | orientation]", self.variable_order[i])
                            }
                        }
                    } else {
                        assert!(random_sample.len() >= 3, "The distribution of an AdditiveRobotCenteredPolar fault for OrientedLandmark observation need to be of dimension 3.");
                        r_add = random_sample[0];
                        theta_add = random_sample[1];
                        z_add = random_sample[2];
                    }
                    let current_r = (o.pose.x.powi(2) + o.pose.y.powi(2)).sqrt();
                    let current_dir = atan2f(o.pose.y, o.pose.x);
                    let r = current_r + r_add;
                    let theta = current_dir + theta_add;
                    let z = o.pose.z + z_add;
                    o.pose.x = r * theta.cos();
                    o.pose.y = r * theta.sin();
                    o.pose.z = mod2pi(z);
                }
            }
        }
    }
}

pub struct AdditiveRobotCenteredPolarRecord {}
