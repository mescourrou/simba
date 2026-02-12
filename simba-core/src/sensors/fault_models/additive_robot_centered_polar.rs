//! AdditiveRobotCenteredPolarPolar faults
//!
//! Remark: the order of the application of the random value is alphabetical on the name of the observation variables if no order is specified.

use std::sync::{Arc, Mutex};

use libm::atan2f;
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::string_combobox};
use crate::{
    sensors::{SensorObservation, fault_models::fault_model::FaultModelConfig},
    utils::{
        SharedMutex,
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
            },
            distributions: vec![RandomVariableTypeConfig::Normal(
                NormalRandomVariableConfig::default(),
            )],
            variable_order: Vec::new(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for AdditiveRobotCenteredPolarFaultConfig {
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
            let possible_variables = ["r", "theta", "orientation", "width", "height"]
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
        });
    }
}

#[derive(Debug)]
pub struct AdditiveRobotCenteredPolarFault {
    apparition: DeterministBernouilliRandomVariable,
    distributions: SharedMutex<Vec<Box<dyn DeterministRandomVariable>>>,
    variable_order: Vec<String>,
    config: AdditiveRobotCenteredPolarFaultConfig,
}

impl AdditiveRobotCenteredPolarFault {
    pub fn from_config(
        config: &AdditiveRobotCenteredPolarFaultConfig,
        va_factory: &DeterministRandomVariableFactory,
        _initial_time: f32,
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
        }
    }
}

impl FaultModel for AdditiveRobotCenteredPolarFault {
    fn add_faults(
        &mut self,
        _time: f32,
        seed: f32,
        period: f32,
        obs_list: &mut Vec<SensorObservation>,
        _obs_type: SensorObservation,
    ) {
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
                    let mut r_add = 0.;
                    let mut z_add = 0.;
                    let mut theta_add = 0.;
                    if !self.variable_order.is_empty() {
                        for (i, variable) in self.variable_order.iter().enumerate() {
                            match variable.as_str() {
                                "r" => r_add = random_sample[i],
                                "theta" => theta_add = random_sample[i],
                                "z" | "orientation" => z_add = random_sample[i],
                                &_ => panic!(
                                    "Unknown variable name: '{}'. Available variable names: [r, theta, z | orientation]",
                                    variable
                                ),
                            }
                        }
                    } else {
                        assert!(
                            random_sample.len() >= 3,
                            "The distribution of an AdditiveRobotCenteredPolar fault for OrientedRobot observation need to be of dimension 3."
                        );
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
                    o.applied_faults
                        .push(FaultModelConfig::AdditiveRobotCenteredPolar(
                            self.config.clone(),
                        ));
                }
                SensorObservation::GNSS(_) => {
                    panic!("Not implemented yet (need to find a logical way to do it.");
                }
                SensorObservation::Speed(_) => {
                    panic!("Not implemented (appropriated for this sensor?)");
                }
                SensorObservation::Displacement(o) => {
                    let mut r_add = 0.;
                    let mut z_add = 0.;
                    let mut theta_add = 0.;
                    if !self.variable_order.is_empty() {
                        for (i, variable) in self.variable_order.iter().enumerate() {
                            match variable.as_str() {
                                "r" => r_add = random_sample[i],
                                "theta" => theta_add = random_sample[i],
                                "z" | "orientation" => z_add = random_sample[i],
                                &_ => panic!(
                                    "Unknown variable name: '{}'. Available variable names: [r, theta, z | orientation]",
                                    variable
                                ),
                            }
                        }
                    } else {
                        assert!(
                            random_sample.len() >= 3,
                            "The distribution of an AdditiveRobotCenteredPolar fault for Displacement observation need to be of dimension 3."
                        );
                        r_add = random_sample[0];
                        theta_add = random_sample[1];
                        z_add = random_sample[2];
                    }
                    let current_r = (o.translation.x.powi(2) + o.translation.y.powi(2)).sqrt();
                    let current_dir = atan2f(o.translation.y, o.translation.x);
                    let r = current_r + r_add;
                    let theta = current_dir + theta_add;
                    let z = o.rotation + z_add;
                    o.translation.x = r * theta.cos();
                    o.translation.y = r * theta.sin();
                    o.rotation = mod2pi(z);
                    o.applied_faults
                        .push(FaultModelConfig::AdditiveRobotCenteredPolar(
                            self.config.clone(),
                        ));
                }
                SensorObservation::OrientedLandmark(o) => {
                    let mut r_add = 0.;
                    let mut z_add = 0.;
                    let mut theta_add = 0.;
                    let mut width_add = 0.;
                    let mut height_add = 0.;
                    if !self.variable_order.is_empty() {
                        for (i, variable) in self.variable_order.iter().enumerate() {
                            match variable.as_str() {
                                "r" => r_add = random_sample[i],
                                "theta" => theta_add = random_sample[i],
                                "z" | "orientation" => z_add = random_sample[i],
                                "width" => width_add = random_sample[i],
                                "height" => height_add = random_sample[i],
                                &_ => panic!(
                                    "Unknown variable name: '{}'. Available variable names: [r, theta, z | orientation, width, height]",
                                    variable
                                ),
                            }
                        }
                    } else {
                        assert!(
                            random_sample.len() >= 3,
                            "The distribution of an AdditiveRobotCenteredPolar fault for OrientedLandmark observation need to be of dimension 3."
                        );
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
                    o.width += width_add;
                    o.height += height_add;
                    o.width = o.width.max(0.0);
                    o.height = o.height.max(0.0);
                    o.applied_faults
                        .push(FaultModelConfig::AdditiveRobotCenteredPolar(
                            self.config.clone(),
                        ));
                }
                SensorObservation::External(_) => {
                    panic!("AdditiveRobotCenteredPolarFault cannot fault ExternalObservation");
                }
            }
        }
    }
}

pub struct AdditiveRobotCenteredPolarRecord {}
