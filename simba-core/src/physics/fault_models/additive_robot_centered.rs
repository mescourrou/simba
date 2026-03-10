//! Additive faults

use std::sync::{Arc, Mutex};

use simba_macros::{config_derives, enum_variables};

#[cfg(feature = "gui")]
use crate::{gui::{UIComponent, utils::{enum_combobox, string_combobox}}, utils::enum_tools::ToVec};
use crate::{
    physics::{fault_models::fault_model::PhysicsFaultModel, robot_models::RobotModelConfig},
    state_estimators::State,
    utils::{
        SharedMutex,
        determinist_random_variable::{
            DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig,
        },
        distributions::normal::NormalRandomVariableConfig,
        geometry::mod2pi,
    },
};

enum_variables!(
    PhysicsVariables;
    X, "x";
    Y, "y";
    Orientation, "orientation", "z";
    Velocity, "velocity", "v";
    VelocityX, "velocity_x", "vx";
    VelocityY, "velocity_y", "vy";
);

#[config_derives]
pub struct AdditiveRobotCenteredPhysicsFaultConfig {
    #[check]
    pub distributions: Vec<RandomVariableTypeConfig>,
    pub variable_order: Vec<PhysicsVariables>,
    /// If some, the faults will be proportional to the robot velocity times this factor.
    /// If none, the faults will be independant of the robot velocity.
    pub proportional_to: Option<PhysicsVariables>,
    pub proportional_factor: Option<f32>,
}

impl Default for AdditiveRobotCenteredPhysicsFaultConfig {
    fn default() -> Self {
        Self {
            distributions: vec![RandomVariableTypeConfig::Normal(
                NormalRandomVariableConfig::default(),
            )],
            variable_order: Vec::new(),
            proportional_to: Some(PhysicsVariables::Velocity),
            proportional_factor: Some(1.0),
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
        unique_id: &str,
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
            let possible_variables: Vec<PhysicsVariables> = PhysicsVariables::to_vec();
            ui.horizontal(|ui| {
                ui.label("Variable order:");
                for (i, var) in self.variable_order.iter_mut().enumerate() {
                    let unique_var_id = format!("variable-{i}-{unique_id}");
                    enum_combobox(ui, var, unique_var_id);
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
    distributions: SharedMutex<Vec<DeterministRandomVariable>>,
    variable_order: Vec<PhysicsVariables>,
    proportionnal_to: Option<PhysicsVariables>,
    proportionnal_factor: Option<f32>,
    last_time_draw: Mutex<f32>,
    robot_model: RobotModelConfig,
}

impl AdditiveRobotCenteredPhysicsFault {
    pub fn from_config(
        config: &AdditiveRobotCenteredPhysicsFaultConfig,
        robot_model: RobotModelConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> Self {
        let mut config = config.clone();
        let distributions = Arc::new(Mutex::new(
            config
                .distributions
                .iter()
                .map(|conf| va_factory.make_variable(conf.clone()))
                .collect::<Vec<DeterministRandomVariable>>(),
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
        if config.proportional_to.is_some() && config.proportional_factor.is_none() {
            config.proportional_factor = Some(1.0);
        }
        Self {
            distributions,
            variable_order: config.variable_order.clone(),
            last_time_draw: Mutex::new(initial_time),
            proportionnal_to: config.proportional_to,
            proportionnal_factor: config.proportional_factor,
            robot_model,
        }
    }
}

impl PhysicsFaultModel for AdditiveRobotCenteredPhysicsFault {
    fn add_faults(&self, time: f32, state: &mut State) {
        let mut last_time_draw = self.last_time_draw.lock().unwrap();
        let delta_time = time - *last_time_draw;
        let delta_time = if let Some(prop_var) = &self.proportionnal_to {
            let factor = self.proportionnal_factor.unwrap_or(1.0);
            let factor = match prop_var {
                PhysicsVariables::X => factor * state.pose.x,
                PhysicsVariables::Y => factor * state.pose.y,
                PhysicsVariables::Orientation => factor * state.pose.z,
                PhysicsVariables::Velocity => factor * state.velocity.fixed_rows::<2>(0).norm(),
                PhysicsVariables::VelocityX => {
                    factor * state.velocity.x
                }
                PhysicsVariables::VelocityY => {
                    factor * state.velocity.y
                }
            };
            delta_time * factor
        } else {
            delta_time
        };
        let mut random_sample = Vec::new();
        for d in self.distributions.lock().unwrap().iter() {
            random_sample.extend_from_slice(&d.generate(time));
        }

        if !self.variable_order.is_empty() {
            for (i, variable) in self.variable_order.iter().enumerate() {
                match variable {
                    PhysicsVariables::X => state.pose.x += random_sample[i] * delta_time,
                    PhysicsVariables::Y => state.pose.y += random_sample[i] * delta_time,
                    PhysicsVariables::Orientation => state.pose.z += random_sample[i] * delta_time,
                    PhysicsVariables::Velocity => {
                        let velocity_angle = state.velocity.x.atan2(state.velocity.x);
                        state.velocity.x += random_sample[i] * delta_time * velocity_angle.cos();
                        if let RobotModelConfig::Unicycle(_) = self.robot_model {
                            // Unicycle robot model does not have lateral velocity (velocity_y).
                        } else {
                            state.velocity.y += random_sample[i] * delta_time * velocity_angle.sin();
                        }
                    }
                    PhysicsVariables::VelocityX => {
                        state.velocity.x += random_sample[i] * delta_time;
                    }
                    PhysicsVariables::VelocityY => {
                        state.velocity.y += random_sample[i] * delta_time;
                    }
                }
            }
        } else {
            if let RobotModelConfig::Unicycle(_) = self.robot_model {
                assert!(
                    random_sample.len() >= 4,
                    "The distribution of an AdditiveRobotCentered physics fault needs to be of dimension 4 (x, y, orientation, velocity_x)."
                );
            } else {
                assert!(
                    random_sample.len() >= 5,
                    "The distribution of an AdditiveRobotCentered physics fault needs to be of dimension 5 (x, y, orientation, velocity_x, velocity_y)."
                );
                state.velocity.y += random_sample[4] * delta_time;
            }
            state.pose.x += random_sample[0] * delta_time;
            state.pose.y += random_sample[1] * delta_time;
            state.pose.z += random_sample[2] * delta_time;
            state.velocity.x += random_sample[3] * delta_time;
        }
        state.pose.z = mod2pi(state.pose.z);
        *last_time_draw = time;
    }
}
