//! Additive robot-centered physics faults.
//! 
//! These faults try to simulate noise-like perturbations on the robot state variables over time, in the robot-centered frame. They can be used to simulate various types of faults, such as unmodeled dynamics, wheel slip, etc. The configuration allows to define which random distributions are sampled, how sampled dimensions are mapped to physics variables, and whether the faults should be proportional to the robot velocity (to simulate slip-like faults).
//!
//! This module defines a physics fault model that adds noise-like perturbations to
//! robot state variables over time in the robot-centered frame.
//! Configuration is expressed with [`AdditiveRobotCenteredPhysicsFaultConfig`], and runtime
//! behavior is implemented by [`AdditiveRobotCenteredPhysicsFault`].

use std::sync::{Arc, Mutex};

use simba_macros::{config_derives, enum_variables};

#[cfg(feature = "gui")]
use crate::{
    gui::{UIComponent, utils::enum_combobox},
    utils::enum_tools::ToVec,
};
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
    "Variables used by the AdditiveRobotCentered physics fault model."
    PhysicsVariables;
    "X coordinate of the robot pose."
    X, "x";
    "Y coordinate of the robot pose."
    Y, "y";
    "Orientation of the robot pose."
    Orientation, "orientation", "z";
    "Norm of the robot linear velocity."
    Velocity, "velocity", "v";
    "Longitudinal velocity."
    VelocityX, "velocity_x", "vx";
    "Lateral velocity."
    VelocityY, "velocity_y", "vy";
);

/// Configuration for additive robot-centered physics faults.
///
/// This configuration defines which random distributions are sampled and how sampled
/// dimensions are mapped to [`PhysicsVariables`].
///
/// Default values:
/// - `distributions`: one [`RandomVariableTypeConfig::Normal`] using [`NormalRandomVariableConfig::default`]
/// - `variable_order`: empty vector (implicit order)
/// - `proportional_to`: `Some(PhysicsVariables::Velocity)`
/// - `proportional_factor`: `Some(1.0)`
#[config_derives]
pub struct AdditiveRobotCenteredPhysicsFaultConfig {
    /// Random distributions used to sample additive perturbations.
    #[check]
    pub distributions: Vec<RandomVariableTypeConfig>,
    /// Optional explicit mapping from sampled dimensions to physics variables.
    pub variable_order: Vec<PhysicsVariables>,
    /// If some, the faults will be proportional to the robot velocity times this factor.
    /// If none, the faults will be independant of the robot velocity.
    pub proportional_to: Option<PhysicsVariables>,
    /// Multiplicative factor applied to the proportional variable value.
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
/// Runtime additive robot-centered physics fault model.
pub struct AdditiveRobotCenteredPhysicsFault {
    distributions: SharedMutex<Vec<DeterministRandomVariable>>,
    variable_order: Vec<PhysicsVariables>,
    proportionnal_to: Option<PhysicsVariables>,
    proportionnal_factor: Option<f32>,
    last_time_draw: Mutex<f32>,
    robot_model: RobotModelConfig,
}

impl AdditiveRobotCenteredPhysicsFault {
    /// Builds a runtime fault model from [`AdditiveRobotCenteredPhysicsFaultConfig`].
    ///
    /// Initializes deterministic random variables from the provided factory and validates
    /// dimension consistency when an explicit `variable_order` is given.
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
        let variable_order = if !config.variable_order.is_empty() {
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
            config.variable_order.clone()
        } else {
            PhysicsVariables::to_vec()
        };
        if config.proportional_to.is_some() && config.proportional_factor.is_none() {
            config.proportional_factor = Some(1.0);
        }
        Self {
            distributions,
            variable_order,
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
                PhysicsVariables::VelocityX => factor * state.velocity.x,
                PhysicsVariables::VelocityY => factor * state.velocity.y,
            };
            delta_time * factor
        } else {
            delta_time
        };
        let mut random_sample = Vec::new();
        for d in self.distributions.lock().unwrap().iter() {
            random_sample.extend_from_slice(&d.generate(time));
        }

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
                        state.velocity.y +=
                            random_sample[i] * delta_time * velocity_angle.sin();
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
        state.pose.z = mod2pi(state.pose.z);
        *last_time_draw = time;
    }
}
