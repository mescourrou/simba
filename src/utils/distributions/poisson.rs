use config_checker::macros::Check;
use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};
use statrs::distribution::Poisson;

use crate::{
    gui::UIComponent,
    utils::determinist_random_variable::{self, DeterministRandomVariable},
};

/// Configuration for a uniform random variable.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct PoissonRandomVariableConfig {
    /// Random seed for this random variable.
    pub unique_seed: f32,
    /// Probabilities of the random variable
    pub lambda: Vec<f64>,
}

impl Default for PoissonRandomVariableConfig {
    fn default() -> Self {
        Self {
            unique_seed: 0.,
            lambda: vec![1.],
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PoissonRandomVariableConfig {
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
                for (i, p) in self.lambda.iter_mut().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("lambda {}:", i + 1));
                        ui.add(egui::DragValue::new(p));
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
            determinist_random_variable::seed_generation_component(
                &mut self.unique_seed,
                ui,
                buffer_stack,
                unique_id,
            );
        });
    }
}

#[derive(Debug)]
pub struct DeterministPoissonRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed from the configuration.
    global_seed: f32,
    /// Probability of the Poisson distribution.
    poisson: Vec<Poisson>,
}

impl DeterministPoissonRandomVariable {
    pub fn from_config(global_seed: f32, config: PoissonRandomVariableConfig) -> Self {
        assert!(config.lambda.iter().all(|x| *x >= 0.));
        Self {
            global_seed: global_seed + config.unique_seed,
            poisson: config
                .lambda
                .iter()
                .map(|lambda| Poisson::new(*lambda).unwrap())
                .collect(),
        }
    }
}

impl DeterministRandomVariable for DeterministPoissonRandomVariable {
    fn gen(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.global_seed + time).to_bits() as u64);
        let mut v = Vec::new();
        for p in &self.poisson {
            v.push(p.sample(&mut rng) as f32);
        }
        v
    }

    fn dim(&self) -> usize {
        self.poisson.len()
    }
}
