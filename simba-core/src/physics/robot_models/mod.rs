//! Robot kinematic model abstractions and configuration.
//!
//! This module defines supported robot model configurations, runtime command types,
//! the [`RobotModel`] trait, and a factory helper to instantiate concrete models.

use std::fmt::Debug;

use serde::{Deserialize, Serialize};
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::{gui::UIComponent, simulator::SimulatorConfig};
use crate::{
    physics::robot_models::{
        holonomic::{Holonomic, HolonomicCommand, HolonomicConfig},
        unicycle::{Unicycle, UnicycleCommand, UnicycleConfig},
    },
    state_estimators::State,
};

pub mod holonomic;
pub mod unicycle;

/// Command enum, wrapping the command types of all supported robot models.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum Command {
    /// Command payload for the unicycle model.
    Unicycle(UnicycleCommand),
    /// Command payload for the holonomic model.
    Holonomic(HolonomicCommand),
}

impl Default for Command {
    fn default() -> Self {
        Self::Unicycle(UnicycleCommand::default())
    }
}

#[cfg(feature = "gui")]
impl UIComponent for Command {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        match &self {
            Command::Unicycle(cmd) => cmd.show(ui, ctx, unique_id),
            Command::Holonomic(cmd) => cmd.show(ui, ctx, unique_id),
        };
    }
}

/// Configuration enum selecting the robot kinematic model.
///
/// Please note that the robot model is transfered from the Physics configuration to
/// other modules (e.g. controller) if it differs or is not provided.
///
/// Default value: [`RobotModelConfig::Unicycle`] with [`UnicycleConfig::default`].
#[config_derives]
pub enum RobotModelConfig {
    /// Differential-drive unicycle model configuration.
    #[check]
    Unicycle(UnicycleConfig),
    /// Holonomic model configuration.
    #[check]
    Holonomic(HolonomicConfig),
}

impl Default for RobotModelConfig {
    fn default() -> Self {
        Self::Unicycle(UnicycleConfig::default())
    }
}

#[cfg(feature = "gui")]
impl UIComponent for RobotModelConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        let mut current_str = self.to_string();
        ui.horizontal(|ui| {
            use crate::{gui::utils::string_combobox, utils::enum_tools::ToVec};

            ui.label("Robot model:");
            string_combobox(
                ui,
                &RobotModelConfig::to_vec(),
                &mut current_str,
                format!("robot-model-choice-{}", unique_id),
            );
        });
        if current_str != self.to_string() {
            match current_str.as_str() {
                "Unicycle" => {
                    *self = RobotModelConfig::Unicycle(UnicycleConfig::default());
                }
                "Holonomic" => {
                    *self = RobotModelConfig::Holonomic(HolonomicConfig::default());
                }
                _ => panic!("Where did you find this value?"),
            };
        }
        match self {
            RobotModelConfig::Unicycle(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            RobotModelConfig::Holonomic(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
        }
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.horizontal(|ui| {
            ui.label(format!("Robot model: {}", self));
        });
        match self {
            RobotModelConfig::Unicycle(c) => c.show(ui, ctx, unique_id),
            RobotModelConfig::Holonomic(c) => c.show(ui, ctx, unique_id),
        }
    }
}

/// Trait implemented by all runtime robot kinematic models.
pub trait RobotModel: std::fmt::Debug + std::marker::Send + std::marker::Sync {
    /// Updates the mutable robot state using the provided command and elapsed time.
    fn update_state(&mut self, previous_state: &mut State, command: &Command, delta_time: f32);

    /// Returns the neutral/default command for this model.
    fn default_command(&self) -> Command;
}

/// Instantiates a runtime robot model from [`RobotModelConfig`].
pub fn make_model_from_config(config: &RobotModelConfig) -> Box<dyn RobotModel> {
    match config {
        RobotModelConfig::Unicycle(cfg) => {
            Box::new(Unicycle::from_config(cfg)) as Box<dyn RobotModel>
        }
        RobotModelConfig::Holonomic(cfg) => {
            Box::new(Holonomic::from_config(cfg)) as Box<dyn RobotModel>
        }
    }
}
