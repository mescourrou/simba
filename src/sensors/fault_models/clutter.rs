use std::sync::{Arc, Mutex};

use config_checker::macros::Check;
use serde::{Deserialize, Serialize};

use crate::{
    gui::{
        utils::{string_combobox, text_singleline_with_apply},
        UIComponent,
    },
    sensors::sensor::SensorObservation,
    utils::{
        determinist_random_variable::{
            DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig,
        },
        distributions::{
            poisson::PoissonRandomVariableConfig, uniform::UniformRandomVariableConfig,
        },
        geometry::mod2pi,
    },
};

use super::fault_model::FaultModel;

#[derive(Debug, Serialize, Deserialize, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct ClutterFaultConfig {
    // #[check(eq(self.apparition.probability.len(), 1))]
    #[check]
    pub apparition: RandomVariableTypeConfig,
    #[check]
    pub distributions: Vec<RandomVariableTypeConfig>,
    pub variable_order: Vec<String>,
    pub observation_id: String,
}

impl Default for ClutterFaultConfig {
    fn default() -> Self {
        Self {
            apparition: RandomVariableTypeConfig::Poisson(PoissonRandomVariableConfig {
                lambda: vec![10.],
                unique_seed: 0.,
            }),
            distributions: vec![RandomVariableTypeConfig::Uniform(
                UniformRandomVariableConfig {
                    unique_seed: 0.,
                    min: vec![-10., -10.],
                    max: vec![10., 10.],
                },
            )],
            variable_order: Vec::new(),
            observation_id: "clutter".to_string(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ClutterFaultConfig {
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
            let buffer_key = format!("observation-id-{unique_id}");
            text_singleline_with_apply(
                ui,
                buffer_key.as_str(),
                buffer_stack,
                &mut self.observation_id,
            );
        });
    }
}

#[derive(Debug)]
pub struct ClutterFault {
    apparition: Arc<Mutex<Box<dyn DeterministRandomVariable>>>,
    distributions: Arc<Mutex<Vec<Box<dyn DeterministRandomVariable>>>>,
    variable_order: Vec<String>,
    observation_id: String,
}

impl ClutterFault {
    pub fn from_config(
        config: &ClutterFaultConfig,
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
        let apparition_distrib = Arc::new(Mutex::new(
            va_factory.make_variable(config.apparition.clone()),
        ));
        assert!(
            apparition_distrib.lock().unwrap().dim() == 1,
            "The dimension of the apparition distribution should be 1."
        );
        Self {
            apparition: apparition_distrib,
            distributions,
            variable_order: config.variable_order.clone(),
            observation_id: config.observation_id.clone(),
        }
    }
}

impl FaultModel for ClutterFault {
    fn add_faults(
        &self,
        time: f32,
        period: f32,
        obs_list: &mut Vec<SensorObservation>,
        obs_type: SensorObservation,
    ) {
        let obs_seed_increment = 1. / (100. * period);
        let mut seed = time;

        let n_obs = self.apparition.lock().unwrap().gen(time)[0].abs().floor() as usize;
        for _ in 0..n_obs {
            seed += obs_seed_increment;
            let mut random_sample = Vec::new();
            for d in self.distributions.lock().unwrap().iter() {
                random_sample.extend_from_slice(&d.gen(seed));
            }
            let mut new_obs = obs_type.clone();
            match &mut new_obs {
                SensorObservation::OrientedRobot(o) => {
                    if self.variable_order.len() > 0 {
                        for i in 0..self.variable_order.len() {
                            match self.variable_order[i].as_str() {
                                "x" => o.pose.x = random_sample[i],
                                "y" => o.pose.y = random_sample[i],
                                "z" | "orientation" => o.pose.z = random_sample[i],
                                &_ => panic!("Unknown variable name: '{}'. Available variable names: [x, y, z | orientation]", self.variable_order[i])
                            }
                        }
                    } else {
                        assert!(random_sample.len() >= 3, "The distribution of an Clutter fault for OrientedRobot observation need to be of dimension 3.");
                        o.pose.x = random_sample[0];
                        o.pose.y = random_sample[1];
                        o.pose.z = random_sample[2];
                    }
                    o.pose.z = mod2pi(o.pose.z);
                    o.name = self.observation_id.clone();
                }
                SensorObservation::GNSS(o) => {
                    if self.variable_order.len() > 0 {
                        for i in 0..self.variable_order.len() {
                            match self.variable_order[i].as_str() {
                                "position_x" | "x" => o.position.x = random_sample[i],
                                "position_y" | "y" => o.position.y = random_sample[i],
                                "velocity_x" => o.velocity.x = random_sample[i],
                                "velocity_y" => o.velocity.y = random_sample[i],
                                &_ => panic!("Unknown variable name: '{}'. Available variable names: [position_x | x, position_y | y, velocity_x, velocity_y]", self.variable_order[i])
                            }
                        }
                    } else {
                        assert!(random_sample.len() >= 2, "The distribution of an Clutter fault for GNSS observation need to be at least of dimension 2 (to 4 for velocities).");
                        o.position.x = random_sample[0];
                        o.position.y = random_sample[1];
                        if random_sample.len() >= 3 {
                            o.velocity.x = random_sample[2];
                        }
                        if random_sample.len() >= 4 {
                            o.velocity.y = random_sample[3];
                        }
                    }
                }
                SensorObservation::Odometry(o) => {
                    if self.variable_order.len() > 0 {
                        for i in 0..self.variable_order.len() {
                            match self.variable_order[i].as_str() {
                                "w" | "angular" | "angular_velocity" => o.angular_velocity = random_sample[i],
                                "v" | "linear" | "linear_velocity" => o.linear_velocity = random_sample[i],
                                &_ => panic!("Unknown variable name: '{}'. Available variable names: [w | angular | angular_velocity, v | linear | linear_velocity]", self.variable_order[i])
                            }
                        }
                    } else {
                        assert!(random_sample.len() >= 2, "The distribution of an Clutter fault for Odometry observation need to be of dimension 2.");
                        o.angular_velocity = random_sample[0];
                        o.linear_velocity = random_sample[1];
                    }
                }
                SensorObservation::OrientedLandmark(o) => {
                    if self.variable_order.len() > 0 {
                        for i in 0..self.variable_order.len() {
                            match self.variable_order[i].as_str() {
                                "x" => o.pose.x = random_sample[i],
                                "y" => o.pose.y = random_sample[i],
                                "z" | "orientation" => o.pose.z = random_sample[i],
                                &_ => panic!("Unknown variable name: '{}'. Available variable names: [x, y, z | orientation]", self.variable_order[i])
                            }
                        }
                    } else {
                        assert!(random_sample.len() >= 3, "The distribution of an Clutter fault for OrientedLandmark observation need to be of dimension 3.");
                        o.pose.x = random_sample[0];
                        o.pose.y = random_sample[1];
                        o.pose.z = random_sample[2];
                    }
                    o.pose.z = mod2pi(o.pose.z);
                    o.id = self.observation_id.parse().unwrap_or(-1);
                }
            }
            obs_list.push(new_obs);
        }
    }
}
