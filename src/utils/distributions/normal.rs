use config_checker::macros::Check;
use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};
use statrs::{distribution::MultivariateNormal, statistics::MeanN};

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::utils::determinist_random_variable::DeterministRandomVariable;
use crate::utils::format_f32;

/// Configuration for a normal random variable.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct NormalRandomVariableConfig {
    /// Random seed for this random variable.
    #[serde(serialize_with = "format_f32")]
    unique_seed: f32,
    /// Mean of the normal distribution.
    mean: Vec<f64>,
    /// Variance of the normal distribution.
    covariance: Vec<f64>,
}

impl Default for NormalRandomVariableConfig {
    fn default() -> Self {
        Self {
            unique_seed: 0.,
            mean: vec![0.],
            covariance: vec![1.],
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for NormalRandomVariableConfig {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &crate::simulator::SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        ui.horizontal_top(|ui| {
            use crate::utils::determinist_random_variable::seed_generation_component;

            ui.vertical(|ui| {
                let mut to_remove = None;
                for (i, p) in self.mean.iter_mut().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("mean {}:", i + 1));
                        ui.add(egui::DragValue::new(p).max_decimals(10));
                        if ui.button("X").clicked() {
                            to_remove = Some(i);
                        }
                    });
                }
                if let Some(i) = to_remove {
                    let previous_size = self.mean.len();
                    self.mean.remove(i);

                    // Remove column
                    for row in (0..previous_size).rev() {
                        self.covariance.remove(row * previous_size + i);
                    }
                    let new_size = self.mean.len();
                    // Remove row (matrix is now not squared)
                    for col in (0..new_size).rev() {
                        self.covariance.remove(i * new_size + col);
                    }
                }
                if ui.button("Add").clicked() {
                    self.mean.push(1.0);
                    let new_size = self.mean.len();
                    // Insert 0 at the end of each row
                    for row in 0..new_size - 1 {
                        self.covariance.insert((row + 1) * new_size - 1, 0.);
                    }
                    // Insert last element of n-1th row
                    self.covariance.push(0.);
                    // Insert last row
                    self.covariance.resize(new_size * new_size, 0.);
                }
            });
            ui.vertical(|ui| {
                ui.label("Covariance: ");
                let size = self.mean.len();
                ui.horizontal(|ui| {
                    let mut row = 0;
                    while row < size {
                        ui.vertical(|ui| {
                            let mut col = 0;
                            while col < size {
                                ui.add(
                                    egui::DragValue::new(
                                        self.covariance.get_mut(row * size + col).unwrap(),
                                    )
                                    .max_decimals(10),
                                );
                                col += 1;
                            }
                        });
                        row += 1;
                    }
                });
            });
            ui.label("Seed: ");
            seed_generation_component(&mut self.unique_seed, ui, buffer_stack, unique_id);
        });
    }
}

/// Random variable which return a random value following a normal distribution.
#[derive(Debug)]
pub struct DeterministNormalRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed from the configuration.
    global_seed: f32,
    /// Normal distribution.
    nd: MultivariateNormal,
}

impl DeterministNormalRandomVariable {
    pub fn from_config(global_seed: f32, config: NormalRandomVariableConfig) -> Self {
        assert!(
            config.mean.len().pow(2) == config.covariance.len(),
            "The length of the covariance vector should be the square of the means' one."
        );
        Self {
            global_seed: global_seed + config.unique_seed,
            nd: MultivariateNormal::new(config.mean, config.covariance)
                .expect("Impossible to create the normal distribution"),
        }
    }
}

impl DeterministRandomVariable for DeterministNormalRandomVariable {
    fn gen(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.global_seed + time).to_bits() as u64);
        self.nd.sample(&mut rng).iter().map(|x| *x as f32).collect()
    }

    fn dim(&self) -> usize {
        self.nd.mean().unwrap().len()
    }
}
