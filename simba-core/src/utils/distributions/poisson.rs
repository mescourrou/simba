use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use simba_macros::config_derives;
use statrs::distribution::Poisson;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;

/// Configuration for a uniform random variable.
#[config_derives]
pub struct PoissonRandomVariableConfig {
    /// Probabilities of the random variable
    pub lambda: Vec<f64>,
}

impl Default for PoissonRandomVariableConfig {
    fn default() -> Self {
        Self { lambda: vec![1.] }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PoissonRandomVariableConfig {
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
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.horizontal_top(|ui| {
            ui.vertical(|ui| {
                for (i, p) in self.lambda.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("lambda {}: {}", i + 1, p));
                    });
                }
            });
        });
    }
}

#[derive(Debug, Clone)]
pub struct DeterministPoissonRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed of this random variable (computed by the factory).
    my_seed: f32,
    /// Probability of the Poisson distribution.
    poisson: Vec<Poisson>,
}

impl DeterministPoissonRandomVariable {
    pub fn from_config(my_seed: f32, config: PoissonRandomVariableConfig) -> Self {
        assert!(config.lambda.iter().all(|x| *x >= 0.));
        Self {
            my_seed,
            poisson: config
                .lambda
                .iter()
                .map(|lambda| Poisson::new(*lambda).unwrap())
                .collect(),
        }
    }

    pub fn generate(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.my_seed + time).to_bits() as u64);
        let mut v = Vec::new();
        for p in &self.poisson {
            v.push(p.sample(&mut rng) as f32);
        }
        v
    }

    pub fn dim(&self) -> usize {
        self.poisson.len()
    }
}
