//! AdditiveRobotCentered faults
//!
//! Remark: the order of the application of the random value is alphabetical on the name of the observation variables if no order is specified.

use std::sync::{Arc, Mutex};

use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::string_combobox};
use crate::{
    sensors::{SensorObservation, fault_models::fault_model::FaultModelConfig},
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

#[config_derives]
pub struct AdditiveRobotCenteredFaultConfig {
    #[check(eq(self.apparition.probability.len(), 1))]
    #[check]
    pub apparition: BernouilliRandomVariableConfig,
    #[check]
    pub distributions: Vec<RandomVariableTypeConfig>,
    pub variable_order: Vec<String>,
    pub proportional_to: Option<String>,
}

impl Default for AdditiveRobotCenteredFaultConfig {
    fn default() -> Self {
        Self {
            apparition: BernouilliRandomVariableConfig {
                probability: vec![1.0],
            },
            distributions: vec![RandomVariableTypeConfig::Normal(
                NormalRandomVariableConfig::default(),
            )],
            variable_order: Vec::new(),
            proportional_to: None,
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
        unique_id: &str,
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
            let possible_variables = [
                "x",
                "y",
                "orientation",
                "velocity_x",
                "velocity_y",
                "w",
                "v",
                "width",
                "height",
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
                if !self.variable_order.is_empty() && ui.button("-").clicked() {
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
            let possible_variables = ["t", "d", "time", "distance"]
                .iter()
                .map(|x| String::from(*x))
                .collect();
            ui.horizontal(|ui| {
                ui.label("Proportional to:");
                if let Some(variable) = &mut self.proportional_to {
                    let unique_var_id = format!("proportional-to-{unique_id}");
                    string_combobox(ui, &possible_variables, variable, unique_var_id);
                    if ui.button("-").clicked() {
                        self.proportional_to = None;
                    }
                } else {
                    if ui.button("+").clicked() {
                        self.proportional_to = Some(possible_variables.get(0).unwrap().clone());
                    }
                }
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
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

            ui.horizontal(|ui| {
                ui.label("Proportional to: ");
                if let Some(variable) = &self.proportional_to {
                    ui.label(variable);
                } else {
                    ui.label("None");
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
    last_time: f32,
    config: AdditiveRobotCenteredFaultConfig,
}

impl AdditiveRobotCenteredFault {
    pub fn from_config(
        config: &AdditiveRobotCenteredFaultConfig,
        va_factory: &DeterministRandomVariableFactory,
        initial_time: f32,
    ) -> Self {
        let distributions = Arc::new(Mutex::new(
            config
                .distributions
                .iter()
                .map(|conf| va_factory.make_variable(conf.clone()))
                .collect::<Vec<Box<dyn DeterministRandomVariable>>>(),
        ));
        if !config.variable_order.is_empty() {
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
            last_time: initial_time,
        }
    }
}

impl FaultModel for AdditiveRobotCenteredFault {
    fn add_faults(
        &mut self,
        time: f32,
        seed: f32,
        period: f32,
        obs_list: &mut Vec<SensorObservation>,
        _obs_type: SensorObservation,
    ) {
        let dt = time - self.last_time;
        let obs_seed_increment = 1. / (100. * period);
        let mut seed = seed;
        for obs in obs_list {
            seed += obs_seed_increment;
            if self.apparition.generate(seed)[0] < 1. {
                continue;
            }
            let mut random_sample = Vec::new();
            for d in self.distributions.lock().unwrap().iter() {
                random_sample.extend_from_slice(&d.generate(seed));
            }
            match obs {
                SensorObservation::OrientedRobot(o) => {
                    if !self.variable_order.is_empty() {
                        for (i, variable) in self.variable_order.iter().enumerate() {
                            match variable.as_str() {
                                "x" => o.pose.x += random_sample[i],
                                "y" => o.pose.y += random_sample[i],
                                "z" | "orientation" => o.pose.z += random_sample[i],
                                &_ => panic!(
                                    "Unknown variable name: '{}'. Available variable names: [x, y, z | orientation]",
                                    variable
                                ),
                            }
                        }
                    } else {
                        assert!(
                            random_sample.len() >= 3,
                            "The distribution of an AdditiveRobotCentered fault for OrientedRobot observation need to be of dimension 3."
                        );
                        o.pose.x += random_sample[0];
                        o.pose.y += random_sample[1];
                        o.pose.z += random_sample[2];
                    }
                    if self.config.proportional_to.is_some() {
                        todo!(
                            "Proportional_to is not implemented yet for AdditiveRobotCenteredFault for OrientedRobot observations"
                        );
                    }
                    o.pose.z = mod2pi(o.pose.z);
                    o.applied_faults
                        .push(FaultModelConfig::AdditiveRobotCentered(self.config.clone()));
                }
                SensorObservation::GNSS(o) => {
                    if !self.variable_order.is_empty() {
                        for (i, variable) in self.variable_order.iter().enumerate() {
                            match variable.as_str() {
                                "position_x" | "x" => o.pose.x += random_sample[i],
                                "position_y" | "y" => o.pose.y += random_sample[i],
                                "orientation" | "z" => o.pose.z += random_sample[i],
                                "velocity_x" => o.velocity.x += random_sample[i],
                                "velocity_y" => o.velocity.y += random_sample[i],
                                &_ => panic!(
                                    "Unknown variable name: '{}'. Available variable names: [position_x | x, position_y | y, orientation | z, velocity_x, velocity_y]",
                                    variable
                                ),
                            }
                        }
                    } else {
                        assert!(
                            random_sample.len() >= 2,
                            "The distribution of an AdditiveRobotCentered fault for GNSS observation need to be at least of dimension 2 (to 5 for orientation and velocities)."
                        );
                        o.pose.x += random_sample[0];
                        o.pose.y += random_sample[1];
                        if random_sample.len() >= 3 {
                            o.pose.z += random_sample[2];
                        }
                        if random_sample.len() >= 4 {
                            o.velocity.x += random_sample[3];
                        }
                        if random_sample.len() >= 5 {
                            o.velocity.y += random_sample[4];
                        }
                    }
                    if self.config.proportional_to.is_some() {
                        todo!(
                            "Proportional_to is not implemented yet for AdditiveRobotCenteredFault for GNSS observations"
                        );
                    }
                    o.applied_faults
                        .push(FaultModelConfig::AdditiveRobotCentered(self.config.clone()));
                }
                #[allow(deprecated)]
                SensorObservation::Speed(o) | SensorObservation::Odometry(o) => {
                    if !self.variable_order.is_empty() {
                        for (i, variable) in self.variable_order.iter().enumerate() {
                            match variable.as_str() {
                                "w" | "angular" | "angular_velocity" => {
                                    o.angular_velocity += random_sample[i]
                                }
                                "v" | "linear" | "linear_velocity" => {
                                    o.linear_velocity += random_sample[i]
                                }
                                &_ => panic!(
                                    "Unknown variable name: '{}'. Available variable names: [w | angular | angular_velocity, v | linear | linear_velocity]",
                                    variable
                                ),
                            }
                        }
                    } else {
                        assert!(
                            random_sample.len() >= 2,
                            "The distribution of an AdditiveRobotCentered fault for Odometry observation need to be of dimension 2."
                        );
                        o.angular_velocity += random_sample[0];
                        o.linear_velocity += random_sample[1];
                    }
                    if self.config.proportional_to.is_some() {
                        todo!(
                            "Proportional_to is not implemented yet for AdditiveRobotCenteredFault for Speed observations"
                        );
                    }
                    o.applied_faults
                        .push(FaultModelConfig::AdditiveRobotCentered(self.config.clone()));
                }
                SensorObservation::Displacement(o) => {
                    let mut dx = 0.;
                    let mut dy = 0.;
                    let mut dr = 0.;
                    if !self.variable_order.is_empty() {
                        for (i, variable) in self.variable_order.iter().enumerate() {
                            match variable.as_str() {
                                "dx" | "x" => dx += random_sample[i],
                                "dy" | "y" => dy += random_sample[i],
                                "r" | "rotation" => dr += random_sample[i],
                                "translation" => {
                                    let v = o.translation.clone().normalize() * (random_sample[i]);
                                    dx += v.x;
                                    dy += v.y;
                                }
                                &_ => panic!(
                                    "Unknown variable name: '{}'. Available variable names: [dx | x, dy | y, r | rotation, translation]",
                                    variable
                                ),
                            }
                        }
                    } else {
                        assert!(
                            random_sample.len() >= 2,
                            "The distribution of an AdditiveRobotCentered fault for Displacement observation need to be at least of dimension 2 (to 3 for rotation)."
                        );
                        dx += random_sample[0];
                        dy += random_sample[1];
                        if random_sample.len() >= 3 {
                            dr += random_sample[2];
                        }
                    }
                    if let Some(variable) = &self.config.proportional_to {
                        let factor = match variable.as_str() {
                            "t" | "time" => dt,
                            "d" | "distance" => o.translation.norm(),
                            &_ => panic!(
                                "Unknown proportional_to variable name: '{}'. Available variable names: [t | time, d | distance]",
                                variable
                            ),
                        };
                        dx *= factor;
                        dy *= factor;
                        dr *= factor;
                    }
                    o.translation.x += dx;
                    o.translation.y += dy;
                    o.rotation += dr;
                    o.rotation = mod2pi(o.rotation);
                    o.applied_faults
                        .push(FaultModelConfig::AdditiveRobotCentered(self.config.clone()));
                }
                SensorObservation::OrientedLandmark(o) => {
                    if !self.variable_order.is_empty() {
                        for (i, variable) in self.variable_order.iter().enumerate() {
                            match variable.as_str() {
                                "x" => o.pose.x += random_sample[i],
                                "y" => o.pose.y += random_sample[i],
                                "z" | "orientation" => o.pose.z += random_sample[i],
                                "width" => o.width += random_sample[i],
                                "height" => o.height += random_sample[i],
                                &_ => panic!(
                                    "Unknown variable name: '{}'. Available variable names: [x, y, z | orientation, width, height]",
                                    variable
                                ),
                            }
                        }
                    } else {
                        assert!(
                            random_sample.len() >= 3,
                            "The distribution of an AdditiveRobotCentered fault for OrientedLandmark observation need to be of dimension 3."
                        );
                        o.pose.x += random_sample[0];
                        o.pose.y += random_sample[1];
                        o.pose.z += random_sample[2];
                    }
                    if self.config.proportional_to.is_some() {
                        todo!(
                            "Proportional_to is not implemented yet for AdditiveRobotCenteredFault for OrientedLandmark observations"
                        );
                    }
                    o.pose.z = mod2pi(o.pose.z);
                    o.width = o.width.max(0.0);
                    o.height = o.height.max(0.0);
                    o.applied_faults
                        .push(FaultModelConfig::AdditiveRobotCentered(self.config.clone()));
                }
                SensorObservation::External(_) => {
                    panic!("AdditiveRobotCenteredFault cannot fault ExternalObservation");
                }
            }
        }
        self.last_time = time;
    }
}

pub struct AdditiveRobotCenteredRecord {}
