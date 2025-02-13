//! AdditiveRobotCentered faults
//! 
//! Remark: the order of the application of the random value is alphabetical on the name of the observation variables if no order is specified.

use std::sync::{Arc, Mutex};

use rand::random;
use serde::{Deserialize, Serialize};

use crate::{sensors::sensor::Observation, simulator::SimulatorConfig, utils::{determinist_random_variable::{DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig}, distributions::normal::NormalRandomVariableConfig, geometry::mod2pi}};

use super::{
    fault_model::FaultType,
};

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(default)]
pub struct AdditiveRobotCenteredFaultConfig {
    pub distributions: Vec<RandomVariableTypeConfig>,
    pub variable_order: Vec<String>,
}

impl Default for AdditiveRobotCenteredFaultConfig {
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
pub struct AdditiveRobotCenteredFault {
    distributions: Arc<Mutex<Vec<Box<dyn DeterministRandomVariable>>>>,
    variable_order: Vec<String>,
}

impl AdditiveRobotCenteredFault {
    pub fn from_config(config: &AdditiveRobotCenteredFaultConfig, va_factory: &DeterministRandomVariableFactory) -> Self {
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

impl FaultType for AdditiveRobotCenteredFault {
    fn add_fault(&self, time: f32, obs: &mut Observation) {
        let mut random_sample = Vec::new();
        for d in self.distributions.lock().unwrap().iter() {
            random_sample.extend_from_slice(&d.gen(time));
        }
        match obs {
            Observation::OrientedRobot(o) => {
                if self.variable_order.len() > 0 {
                    for i in 0..self.variable_order.len() {
                        match self.variable_order[i].as_str() {
                            "x" => o.pose.x += random_sample[i],
                            "y" => o.pose.y += random_sample[i],
                            "z" | "orientation" => o.pose.z += random_sample[i],
                            &_ => panic!("Unknown variable name: '{}'. Available variable names: [x, y, z | orientation]", self.variable_order[i])
                        }
                    }
                } else {
                    assert!(random_sample.len() >= 3, "The distribution of an AdditiveRobotCentered fault for OrientedRobot observation need to be of dimension 3.");
                    o.pose.x += random_sample[0];
                    o.pose.y += random_sample[1];
                    o.pose.z += random_sample[2];
                    o.pose.z = mod2pi(o.pose.z);
                }
            },
            Observation::GNSS(o) => {
                if self.variable_order.len() > 0 {
                    for i in 0..self.variable_order.len() {
                        match self.variable_order[i].as_str() {
                            "position_x" | "x" => o.position.x += random_sample[i],
                            "position_y" | "y" => o.position.y += random_sample[i],
                            "velocity_x" => o.velocity.x += random_sample[i],
                            "velocity_y" => o.velocity.y += random_sample[i],
                            &_ => panic!("Unknown variable name: '{}'. Available variable names: [position_x | x, position_y | y, velocity_x, velocity_y]", self.variable_order[i])
                        }
                    }
                } else {
                    assert!(random_sample.len() >= 2, "The distribution of an AdditiveRobotCentered fault for GNSS observation need to be at least of dimension 2 (to 4 for velocities).");
                    o.position.x += random_sample[0];
                    o.position.y += random_sample[1];
                    if random_sample.len() >= 3 {
                        o.velocity.x += random_sample[2];
                    }
                    if random_sample.len() >= 4 {
                        o.velocity.y += random_sample[3];
                    }
                }
            },
            Observation::Odometry(o) => {
                if self.variable_order.len() > 0 {
                    for i in 0..self.variable_order.len() {
                        match self.variable_order[i].as_str() {
                            "w" | "angular" | "angular_velocity" => o.angular_velocity += random_sample[i],
                            "v" | "linear" | "linear_velocity" => o.linear_velocity += random_sample[i],
                            &_ => panic!("Unknown variable name: '{}'. Available variable names: [w | angular | angular_velocity, v | linear | linear_velocity]", self.variable_order[i])
                        }
                    }
                } else {
                    assert!(random_sample.len() >= 2, "The distribution of an AdditiveRobotCentered fault for Odometry observation need to be of dimension 2.");
                    o.angular_velocity += random_sample[0];
                    o.linear_velocity += random_sample[1];
                }
            },
            Observation::OrientedLandmark(o) => {
                if self.variable_order.len() > 0 {
                    for i in 0..self.variable_order.len() {
                        match self.variable_order[i].as_str() {
                            "x" => o.pose.x += random_sample[i],
                            "y" => o.pose.y += random_sample[i],
                            "z" | "orientation" => o.pose.z += random_sample[i],
                            &_ => panic!("Unknown variable name: '{}'. Available variable names: [x, y, z | orientation]", self.variable_order[i])
                        }
                    }
                } else {
                    assert!(random_sample.len() >= 3, "The distribution of an AdditiveRobotCentered fault for OrientedLandmark observation need to be of dimension 3.");
                    o.pose.x += random_sample[0];
                    o.pose.y += random_sample[1];
                    o.pose.z += random_sample[2];
                    o.pose.z = mod2pi(o.pose.z);
                }
            }
        }
    }
}

pub struct AdditiveRobotCenteredRecord {}
