//! Additive faults

use std::sync::{Arc, Mutex};

use config_checker::macros::Check;
use serde::{Deserialize, Serialize};

#[cfg(feature = "gui")]
use crate::gui::{utils::string_combobox, UIComponent};
use crate::{
    physics::fault_models::fault_model::PhysicsFaultModel,
    state_estimators::state_estimator::State,
    utils::{
        determinist_random_variable::{
            DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig,
        },
        distributions::{
            bernouilli::BernouilliRandomVariableConfig, normal::NormalRandomVariableConfig,
        },
        geometry::mod2pi,
    },
};

#[derive(Debug, Serialize, Deserialize, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct AdditiveRobotCenteredPhysicsFaultConfig {
    #[check]
    pub distributions: Vec<RandomVariableTypeConfig>,
    pub variable_order: Vec<String>,
}

impl Default for AdditiveRobotCenteredPhysicsFaultConfig {
    fn default() -> Self {
        Self {
            distributions: vec![RandomVariableTypeConfig::Normal(
                NormalRandomVariableConfig::default(),
            )],
            variable_order: Vec::new(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for AdditiveRobotCenteredPhysicsFaultConfig {
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
            RandomVariableTypeConfig::show_vector_mut(
                &mut self.distributions,
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
            let possible_variables = vec!["x", "y", "orientation", "velocity"]
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
            use crate::utils::determinist_random_variable::RandomVariableTypeConfig;

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
pub struct AdditiveRobotCenteredPhysicsFault {
    distributions: Arc<Mutex<Vec<Box<dyn DeterministRandomVariable>>>>,
    variable_order: Vec<String>,
    last_time_draw: Mutex<f32>,
}

impl AdditiveRobotCenteredPhysicsFault {
    pub fn from_config(
        config: &AdditiveRobotCenteredPhysicsFaultConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
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
            distributions,
            variable_order: config.variable_order.clone(),
            last_time_draw: Mutex::new(0.),
        }
    }
}

impl PhysicsFaultModel for AdditiveRobotCenteredPhysicsFault {
    fn add_faults(&self, time: f32, state: &mut State) {
        let mut last_time_draw = self.last_time_draw.lock().unwrap();
        let delta_time = time - *last_time_draw;
        let mut random_sample = Vec::new();
        for d in self.distributions.lock().unwrap().iter() {
            random_sample.extend_from_slice(&d.gen(time));
        }

        if self.variable_order.len() > 0 {
            for i in 0..self.variable_order.len() {
                match self.variable_order[i].as_str() {
                    "x" => state.pose.x += random_sample[i] * delta_time,
                    "y" => state.pose.y += random_sample[i] * delta_time,
                    "z" | "orientation" => state.pose.z += random_sample[i] * delta_time,
                    "v" | "velocity" => state.velocity += random_sample[i] * delta_time,
                    &_ => panic!("Unknown variable name: '{}'. Available variable names: [x, y, z | orientation, v | velocity]", self.variable_order[i])
                }
            }
        } else {
            assert!(random_sample.len() >= 4, "The distribution of an AdditiveRobotCentered physics fault needs to be of dimension 4 (x, y, orientation, velocity).");
            state.pose.x += random_sample[0] * delta_time;
            state.pose.y += random_sample[1] * delta_time;
            state.pose.z += random_sample[2] * delta_time;
            state.velocity += random_sample[3] * delta_time;
        }
        state.pose.z = mod2pi(state.pose.z);
        *last_time_draw = time;
    }
}
