use config_checker::macros::Check;
use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::utils::determinist_random_variable::DeterministRandomVariable;
use crate::utils::format_f32;

/// Configuration for a uniform random variable.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct BernouilliRandomVariableConfig {
    /// Random seed for this random variable.
    #[serde(serialize_with = "format_f32")]
    pub unique_seed: f32,
    /// Probabilities of the random variable
    pub probability: Vec<f32>,
}

impl Default for BernouilliRandomVariableConfig {
    fn default() -> Self {
        Self {
            unique_seed: 0.,
            probability: vec![1.],
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for BernouilliRandomVariableConfig {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &crate::simulator::SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        use crate::utils::determinist_random_variable::seed_generation_component;
        ui.horizontal_top(|ui| {
            ui.vertical(|ui| {
                let mut to_remove = None;
                for (i, p) in self.probability.iter_mut().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("p {}:", i + 1));
                        ui.add(
                            egui::DragValue::new(p)
                                .clamp_range(0.0..=1.0)
                                .max_decimals(10),
                        );
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
            ui.label("Seed: ");
            seed_generation_component(&mut self.unique_seed, ui, buffer_stack, unique_id);
        });
    }
}

#[derive(Debug)]
pub struct DeterministBernouilliRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed from the configuration.
    global_seed: f32,
    /// Probability of the bernouilli distribution.
    probability: Vec<f32>,
}

impl DeterministBernouilliRandomVariable {
    pub fn from_config(global_seed: f32, config: BernouilliRandomVariableConfig) -> Self {
        assert!(config.probability.iter().all(|x| *x <= 1.));
        assert!(config.probability.iter().all(|x| *x >= 0.));
        Self {
            global_seed: global_seed + config.unique_seed,
            probability: config.probability,
        }
    }
}

impl DeterministRandomVariable for DeterministBernouilliRandomVariable {
    fn gen(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.global_seed + time).to_bits() as u64);
        let mut v = Vec::new();
        for p in &self.probability {
            v.push(if rng.gen::<f32>() <= *p { 1. } else { 0. });
        }
        v
    }

    fn dim(&self) -> usize {
        self.probability.len()
    }
}
