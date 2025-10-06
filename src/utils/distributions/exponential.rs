use config_checker::macros::Check;
use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};
use statrs::distribution::Exp;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::utils::determinist_random_variable::DeterministRandomVariable;
use crate::utils::format_f32;

/// Configuration for a uniform random variable.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct ExponentialRandomVariableConfig {
    /// Random seed for this random variable.
    #[serde(serialize_with = "format_f32")]
    pub unique_seed: f32,
    /// Probabilities of the random variable
    pub lambda: Vec<f64>,
}

impl Default for ExponentialRandomVariableConfig {
    fn default() -> Self {
        Self {
            unique_seed:  random(),
            lambda: vec![1.],
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ExponentialRandomVariableConfig {
    fn show_mut(
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
                for (i, p) in self.lambda.iter_mut().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("lambda {}:", i + 1));
                        ui.add(egui::DragValue::new(p).max_decimals(10));
                        if ui.button("X").clicked() {
                            to_remove = Some(i);
                        }
                    });
                }
                if let Some(i) = to_remove {
                    self.lambda.remove(i);
                }
                if ui.button("Add").clicked() {
                    self.lambda.push(1.0);
                }
            });
            ui.label("Seed: ");
            seed_generation_component(&mut self.unique_seed, ui, buffer_stack, unique_id);
        });
    }

    fn show(
        &self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        unique_id: &String,
    ) {
        ui.horizontal_top(|ui| {
            ui.vertical(|ui| {
                for (i, p) in self.lambda.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("lambda {}: {}", i + 1, p));
                    });
                }
            });
            ui.label(format!("Seed: {}", self.unique_seed));
        });
    }
}

#[derive(Debug)]
pub struct DeterministExponentialRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed from the configuration.
    global_seed: f32,
    /// Probability of the Exponential distribution.
    exponential: Vec<Exp>,
}

impl DeterministExponentialRandomVariable {
    pub fn from_config(global_seed: f32, config: ExponentialRandomVariableConfig) -> Self {
        assert!(config.lambda.iter().all(|x| *x >= 0.));
        Self {
            global_seed: global_seed + config.unique_seed,
            exponential: config
                .lambda
                .iter()
                .map(|lambda| Exp::new(*lambda).unwrap())
                .collect(),
        }
    }
}

impl DeterministRandomVariable for DeterministExponentialRandomVariable {
    fn gen(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.global_seed + time).to_bits() as u64);
        let mut v = Vec::new();
        for p in &self.exponential {
            v.push(p.sample(&mut rng) as f32);
        }
        v
    }

    fn dim(&self) -> usize {
        self.exponential.len()
    }
}
