//! AdditiveObservationCenteredPolarPolar faults
//! 
//! Remark: the order of the application of the random value is alphabetical on the name of the observation variables if no order is specified.

use std::sync::{Arc, Mutex};

use libm::atan2f;
use rand::random;
use serde::{Deserialize, Serialize};

use crate::{sensors::sensor::Observation, simulator::SimulatorConfig, utils::{determinist_random_variable::{DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig}, distributions::normal::NormalRandomVariableConfig, geometry::mod2pi}};

use super::{
    fault_model::FaultType,
};

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(default)]
pub struct AdditiveObservationCenteredPolarFaultConfig {
    pub distributions: Vec<RandomVariableTypeConfig>,
    pub variable_order: Vec<String>,
}

impl Default for AdditiveObservationCenteredPolarFaultConfig {
    fn default() -> Self {
        Self {
            distributions: vec![RandomVariableTypeConfig::Normal(
                NormalRandomVariableConfig::default(),
            )],
            variable_order: Vec::new(),
        }
    }
}

#[derive(Debug)]
pub struct AdditiveObservationCenteredPolarFault {
    distributions: Arc<Mutex<Vec<Box<dyn DeterministRandomVariable>>>>,
    variable_order: Vec<String>,
}

impl AdditiveObservationCenteredPolarFault {
    pub fn from_config(config: &AdditiveObservationCenteredPolarFaultConfig, va_factory: &DeterministRandomVariableFactory) -> Self {
        let distributions = Arc::new(Mutex::new(config.distributions.iter().map(|conf| va_factory.make_variable(conf.clone())).collect::<Vec<Box<dyn DeterministRandomVariable>>>()));
        if config.variable_order.len() != 0 {
            assert!(config.variable_order.len() == distributions.lock().unwrap().iter().map(|d| d.dim()).sum::<usize>(), "If variable order is given, its length must match the distribution dimension.");
        }
        Self {
            distributions,
            variable_order: config.variable_order.clone(),
        }
    }
}

impl FaultType for AdditiveObservationCenteredPolarFault {
    fn add_fault(&self, time: f32, obs: &mut Observation) {
        let mut random_sample = Vec::new();
        for d in self.distributions.lock().unwrap().iter() {
            random_sample.extend_from_slice(&d.gen(time));
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
                    assert!(random_sample.len() >= 3, "The distribution of an AdditiveObservationCenteredPolar fault for OrientedRobot observation need to be of dimension 3.");
                    theta_add = random_sample[0];
                    r_add = random_sample[1];
                    z_add = random_sample[2];
                }
                
                let theta = o.pose.z + theta_add; // 0 of polar angle is the direction of the robot
                o.pose.x += r_add * theta.cos();
                o.pose.y += r_add * theta.sin();
                o.pose.z = o.pose.z + z_add;
                o.pose.z = mod2pi(o.pose.z);
            },
            Observation::GNSS(o) => {
                let mut r_add = 0.;
                let mut theta_add = 0.;
                if self.variable_order.len() > 0 {
                    for i in 0..self.variable_order.len() {
                        match self.variable_order[i].as_str() {
                            "r" => r_add = random_sample[i],
                            "theta" => theta_add = random_sample[i],
                            &_ => panic!("Unknown variable name: '{}'. Available variable names: [r, theta]", self.variable_order[i])
                        }
                    }
                } else {
                    assert!(random_sample.len() >= 3, "The distribution of an AdditiveObservationCenteredPolar fault for OrientedRobot observation need to be of dimension 3.");
                    theta_add = random_sample[0];
                    r_add = random_sample[1];
                }
                
                o.position.x += r_add * theta_add.cos();
                o.position.y += r_add * theta_add.sin();
            },
            Observation::Odometry(o) => {
                panic!("Not implemented (appropriated for this sensor?)");
            },
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
                    assert!(random_sample.len() >= 3, "The distribution of an AdditiveObservationCenteredPolar fault for OrientedLandmark observation need to be of dimension 3.");
                    r_add = random_sample[0];
                    theta_add = random_sample[1];
                    z_add = random_sample[2];
                }
                let theta = o.pose.z + theta_add; // 0 of polar angle is the direction of the landmark
                o.pose.x += r_add * theta.cos();
                o.pose.y += r_add * theta.sin();
                o.pose.z = o.pose.z + z_add;
                o.pose.z = mod2pi(o.pose.z);
            }
        }
    }
}

pub struct AdditiveObservationCenteredPolarRecord {}
