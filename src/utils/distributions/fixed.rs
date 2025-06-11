use config_checker::macros::Check;
use serde::{Deserialize, Serialize};

use crate::{gui::UIComponent, utils::determinist_random_variable::DeterministRandomVariable};

/// Configuration for a fixed random variable.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct FixedRandomVariableConfig {
    /// Fixed value to return.
    values: Vec<f32>,
}

impl Default for FixedRandomVariableConfig {
    fn default() -> Self {
        Self { values: vec![0.] }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for FixedRandomVariableConfig {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::HashMap<String, String>,
        global_config: &crate::simulator::SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        ui.horizontal_top(|ui| {
            ui.vertical(|ui| {
                let mut to_remove = None;
                for (i, p) in self.values.iter_mut().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("value {}:", i + 1));
                        ui.add(egui::DragValue::new(p));
                        if ui.button("X").clicked() {
                            to_remove = Some(i);
                        }
                    });
                }
                if let Some(i) = to_remove {
                    self.values.remove(i);
                }
                if ui.button("Add").clicked() {
                    self.values.push(1.0);
                }
            });
        });
    }
}

/// Random variable which always return the same value.
#[derive(Debug)]
pub struct DeterministFixedRandomVariable {
    values: Vec<f32>,
}

impl DeterministFixedRandomVariable {
    pub fn from_config(_global_seed: f32, config: FixedRandomVariableConfig) -> Self {
        Self {
            values: config.values,
        }
    }
}

impl DeterministRandomVariable for DeterministFixedRandomVariable {
    fn gen(&self, _time: f32) -> Vec<f32> {
        self.values.clone()
    }

    fn dim(&self) -> usize {
        self.values.len()
    }
}
