//! AdditiveRobotCentered faults
//!
//! Remark: the order of the application of the random value is alphabetical on the name of the observation variables if no order is specified.

use std::sync::{Arc, Mutex};

use config_checker::macros::Check;
use serde::{Deserialize, Serialize};

#[cfg(feature = "gui")]
use crate::gui::{utils::string_combobox, UIComponent};
use crate::{
    sensors::{fault_models::fault_model::FaultModelConfig, sensor::SensorObservation},
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
pub struct AdditiveRobotCenteredFaultConfig {
    #[check(eq(self.apparition.probability.len(), 1))]
    #[check]
    pub apparition: BernouilliRandomVariableConfig,
    #[check]
    pub distributions: Vec<RandomVariableTypeConfig>,
    pub variable_order: Vec<String>,
}

impl Default for AdditiveRobotCenteredFaultConfig {
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
impl UIComponent for AdditiveRobotCenteredFaultConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &crate::simulator::SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.label("Apparition probability: ");
                self.apparition.show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );
            });
            RandomVariableTypeConfig::show_vector_mut(
                &mut self.distributions,
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
            let possible_variables = vec![
                "x",
                "y",
                "orientation",
                "velocity_x",
                "velocity_y",
                "w",
                "v",
            ]
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

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &String) {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.label("Apparition probability: ");
                self.apparition.show(ui, ctx, unique_id);
            });
            RandomVariableTypeConfig::show_vector(&self.distributions, ui, ctx, unique_id);
            ui.horizontal(|ui| {
                ui.label("Variable order:");
                for var in self.variable_order.iter() {
                    ui.label(format!("{}, ", var));
                }
            });
        });
    }
}

#[derive(Debug)]
pub struct AdditiveRobotCenteredFault {
    apparition: DeterministBernouilliRandomVariable,
    distributions: Arc<Mutex<Vec<Box<dyn DeterministRandomVariable>>>>,
    variable_order: Vec<String>,
    config: AdditiveRobotCenteredFaultConfig,
}

impl AdditiveRobotCenteredFault {
    pub fn from_config(
        config: &AdditiveRobotCenteredFaultConfig,
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
                va_factory.global_seed(),
                config.apparition.clone(),
            ),
            distributions,
            variable_order: config.variable_order.clone(),
            config: config.clone(),
        }
    }
}

impl FaultModel for AdditiveRobotCenteredFault {
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
                    }
                    o.pose.z = mod2pi(o.pose.z);
                    o.applied_faults
                        .push(FaultModelConfig::AdditiveRobotCentered(self.config.clone()));
                }
                SensorObservation::GNSS(o) => {
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
                    o.applied_faults
                        .push(FaultModelConfig::AdditiveRobotCentered(self.config.clone()));
                }
                SensorObservation::Odometry(o) => {
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
                    o.applied_faults
                        .push(FaultModelConfig::AdditiveRobotCentered(self.config.clone()));
                }
                SensorObservation::OrientedLandmark(o) => {
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
                    }
                    o.pose.z = mod2pi(o.pose.z);
                    o.applied_faults
                        .push(FaultModelConfig::AdditiveRobotCentered(self.config.clone()));
                }
            }
        }
    }
}

pub struct AdditiveRobotCenteredRecord {}
