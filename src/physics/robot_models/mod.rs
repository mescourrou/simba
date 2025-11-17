use std::fmt::Debug;

use config_checker::macros::Check;
use serde::{Deserialize, Serialize};
use simba_macros::{EnumToString, ToVec};

#[cfg(feature = "gui")]
use crate::{gui::UIComponent, simulator::SimulatorConfig};
use crate::{
    physics::robot_models::unicycle::{Unicycle, UnicycleCommand, UnicycleConfig},
    state_estimators::state_estimator::State,
};

pub mod unicycle;

/// Command struct, to control both wheel speed, in m/s.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Command {
    pub unicycle: Option<UnicycleCommand>,
}

#[cfg(feature = "gui")]
impl UIComponent for Command {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &String) {
        let mut one_command = false;
        if let Some(uc) = &self.unicycle {
            one_command = true;
            egui::CollapsingHeader::new("Unicycle").show(ui, |ui| {
                uc.show(ui, ctx, unique_id);
            });
        }
        if !one_command {
            ui.label("No command");
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, EnumToString, Check, ToVec)]
pub enum RobotModelConfig {
    Unicycle(UnicycleConfig),
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
        unique_id: &String,
    ) {
        let mut current_str = self.to_string();
        ui.horizontal(|ui| {
            use crate::{gui::utils::string_combobox, utils::enum_tools::ToVec};

            ui.label("Robot model:");
            string_combobox(
                ui,
                &RobotModelConfig::to_vec()
                    .iter()
                    .map(|x| String::from(*x))
                    .collect(),
                &mut current_str,
                format!("robot-model-choice-{}", unique_id),
            );
        });
        if current_str != self.to_string() {
            match current_str.as_str() {
                "Unicycle" => {
                    *self = RobotModelConfig::Unicycle(UnicycleConfig::default());
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
        }
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &String) {
        ui.horizontal(|ui| {
            ui.label(format!("Robot model: {}", self.to_string()));
        });
        match self {
            RobotModelConfig::Unicycle(c) => c.show(ui, ctx, unique_id),
        }
    }
}

pub trait RobotModel: std::fmt::Debug + std::marker::Send + std::marker::Sync {
    fn update_state(&mut self, previous_state: &mut State, command: &Command, delta_time: f32);
    fn default_command(&self) -> Command;
}

pub fn make_model_from_config(config: &RobotModelConfig) -> Box<dyn RobotModel> {
    match config {
        RobotModelConfig::Unicycle(cfg) => {
            Box::new(Unicycle::from_config(cfg)) as Box<dyn RobotModel>
        }
    }
}
