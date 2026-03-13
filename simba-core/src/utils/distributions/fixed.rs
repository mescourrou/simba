//! Fixed distribution random-variable utilities.
//!
//! This module provides configuration and deterministic generation logic for
//! fixed random variables used by the simulator.

use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;

/// Configuration for a fixed random variable.
#[config_derives]
pub struct FixedRandomVariableConfig {
    /// Fixed value to return.
    pub values: Vec<f32>,
}

impl Check for FixedRandomVariableConfig {
    fn do_check(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.values.is_empty() {
            errors.push("Values vector cannot be empty.".to_string());
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

impl Default for FixedRandomVariableConfig {
    fn default() -> Self {
        Self { values: vec![0.] }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for FixedRandomVariableConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &crate::simulator::SimulatorConfig,
        _current_node_name: Option<&String>,
        _unique_id: &str,
    ) {
        ui.horizontal_top(|ui| {
            ui.vertical(|ui| {
                let mut to_remove = None;
                for (i, p) in self.values.iter_mut().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("value {}:", i + 1));
                        ui.add(egui::DragValue::new(p).max_decimals(10));
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

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.horizontal_top(|ui| {
            ui.vertical(|ui| {
                for (i, p) in self.values.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("value {}: {}", i + 1, p));
                    });
                }
            });
        });
    }
}

/// Random variable that always returns the same values.
#[derive(Debug, Clone)]
pub struct DeterministFixedRandomVariable {
    values: Vec<f32>,
}

impl DeterministFixedRandomVariable {
    /// Build a deterministic fixed random variable from configuration.
    ///
    /// `_my_seed` is accepted for API consistency with other random variables
    /// and is not used.
    pub fn from_config(_my_seed: f32, config: FixedRandomVariableConfig) -> Self {
        Self {
            values: config.values,
        }
    }

    /// Generate one sample vector at a given simulation time.
    ///
    /// For fixed variables, the output is constant and independent of time.
    pub fn generate(&self, _time: f32) -> Vec<f32> {
        self.values.clone()
    }

    /// Return the output dimension of the random variable.
    pub fn dim(&self) -> usize {
        self.values.len()
    }
}
