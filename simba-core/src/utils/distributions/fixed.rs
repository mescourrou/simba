use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::utils::determinist_random_variable::DeterministRandomVariable;

/// Configuration for a fixed random variable.
#[config_derives]
pub struct FixedRandomVariableConfig {
    /// Fixed value to return.
    pub values: Vec<f32>,
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

/// Random variable which always return the same value.
#[derive(Debug)]
pub struct DeterministFixedRandomVariable {
    values: Vec<f32>,
}

impl DeterministFixedRandomVariable {
    pub fn from_config(_my_seed: f32, config: FixedRandomVariableConfig) -> Self {
        Self {
            values: config.values,
        }
    }
}

impl DeterministRandomVariable for DeterministFixedRandomVariable {
    fn generate(&self, _time: f32) -> Vec<f32> {
        self.values.clone()
    }

    fn dim(&self) -> usize {
        self.values.len()
    }
}
