use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::utils::determinist_random_variable::DeterministRandomVariable;

/// Configuration for a uniform random variable.
#[config_derives]
pub struct BernouilliRandomVariableConfig {
    /// Probabilities of the random variable
    pub probability: Vec<f32>,
}

impl Default for BernouilliRandomVariableConfig {
    fn default() -> Self {
        Self {
            probability: vec![1.],
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for BernouilliRandomVariableConfig {
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
                for (i, p) in self.probability.iter_mut().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("p {}:", i + 1));
                        ui.add(egui::DragValue::new(p).range(0.0..=1.0).max_decimals(10));
                        if ui.button("X").clicked() {
                            to_remove = Some(i);
                        }
                    });
                }
                if let Some(i) = to_remove {
                    self.probability.remove(i);
                }
                if ui.button("Add").clicked() {
                    self.probability.push(1.0);
                }
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.horizontal_top(|ui| {
            ui.vertical(|ui| {
                for (i, p) in self.probability.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("p {}: {}", i + 1, p));
                    });
                }
            });
        });
    }
}

#[derive(Debug)]
pub struct DeterministBernouilliRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed of this random variable (computed by the factory).
    my_seed: f32,
    /// Probability of the bernouilli distribution.
    probability: Vec<f32>,
}

impl DeterministBernouilliRandomVariable {
    pub fn from_config(my_seed: f32, config: BernouilliRandomVariableConfig) -> Self {
        assert!(config.probability.iter().all(|x| *x <= 1.));
        assert!(config.probability.iter().all(|x| *x >= 0.));
        Self {
            my_seed,
            probability: config.probability,
        }
    }
}

impl DeterministRandomVariable for DeterministBernouilliRandomVariable {
    fn generate(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.my_seed + time).to_bits() as u64);
        let mut v = Vec::new();
        for p in &self.probability {
            v.push(if rng.r#gen::<f32>() <= *p { 1. } else { 0. });
        }
        v
    }

    fn dim(&self) -> usize {
        self.probability.len()
    }
}
