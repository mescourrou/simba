//! AdditiveObservationCenteredPolarPolar faults
//!
//! Remark: the order of the application of the random value is alphabetical on the name of the observation variables if no order is specified.

use std::sync::{Arc, Mutex};

use config_checker::macros::Check;
use serde::{Deserialize, Serialize};

#[cfg(feature = "gui")]
use crate::gui::{utils::string_combobox, UIComponent};
use crate::{
    sensors::sensor::SensorObservation,
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
pub struct AdditiveObservationCenteredPolarFaultConfig {
    #[check(eq(self.apparition.probability.len(), 1))]
    #[check]
    pub apparition: BernouilliRandomVariableConfig,
    #[check]
    pub distributions: Vec<RandomVariableTypeConfig>,
    pub variable_order: Vec<String>,
}

impl Default for AdditiveObservationCenteredPolarFaultConfig {
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

#[cfg(feature = "gui")]
impl UIComponent for AdditiveObservationCenteredPolarFaultConfig {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::HashMap<String, String>,
        global_config: &crate::simulator::SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.label("Apparition probability: ");
                self.apparition.show(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );
            });
            RandomVariableTypeConfig::show_vector(
                &mut self.distributions,
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
            let possible_variables = vec!["r", "theta", "orientation"]
                .iter()
                .map(|x| String::from(*x))
                .collect();
            ui.horizontal(|ui| {
                ui.label("Variable order:");
                for (i, var) in self.variable_order.iter_mut().enumerate() {
                    let unique_var_id = format!("variable-{i}-{unique_id}");
                    string_combobox(ui, &possible_variables, var, unique_var_id);
                }
                if self.variable_order.len() > 0 && ui.button("-").clicked() {
                    self.variable_order.pop();
                }
                if ui.button("+").clicked() {
                    self.variable_order.push(
                        possible_variables
                            .get(self.variable_order.len().min(possible_variables.len()))
                            .unwrap()
                            .clone(),
                    );
                }
            });
        });
    }
}

#[derive(Debug)]
pub struct AdditiveObservationCenteredPolarFault {
    apparition: DeterministBernouilliRandomVariable,
    distributions: Arc<Mutex<Vec<Box<dyn DeterministRandomVariable>>>>,
    variable_order: Vec<String>,
}

impl AdditiveObservationCenteredPolarFault {
    pub fn from_config(
        config: &AdditiveObservationCenteredPolarFaultConfig,
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

impl FaultModel for AdditiveObservationCenteredPolarFault {
    fn add_faults(
        &self,
        time: f32,
        period: f32,
        obs_list: &mut Vec<SensorObservation>,
        _obs_type: SensorObservation,
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
                SensorObservation::OrientedRobot(o) => {
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
                }
                SensorObservation::GNSS(o) => {
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
                }
                SensorObservation::Odometry(_) => {
                    panic!("Not implemented (appropriated for this sensor?)");
                }
                SensorObservation::OrientedLandmark(o) => {
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
}

pub struct AdditiveObservationCenteredPolarRecord {}
