use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use simba_macros::config_derives;
use statrs::{distribution::MultivariateNormal, statistics::MeanN};

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::utils::determinist_random_variable::DeterministRandomVariable;

/// Configuration for a normal random variable.
#[config_derives]
pub struct NormalRandomVariableConfig {
    /// Mean of the normal distribution.
    pub mean: Vec<f64>,
    /// Variance of the normal distribution.
    pub covariance: Vec<f64>,
}

impl Default for NormalRandomVariableConfig {
    fn default() -> Self {
        Self {
            mean: vec![0.],
            covariance: vec![1.],
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for NormalRandomVariableConfig {
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
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.horizontal_top(|ui| {
            ui.vertical(|ui| {
                for (i, p) in self.mean.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("mean {}: {}", i + 1, p));
                    });
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
                                ui.label(format!(
                                    "{}",
                                    self.covariance.get(row * size + col).unwrap()
                                ));
                                col += 1;
                            }
                        });
                        row += 1;
                    }
                });
            });
        });
    }
}

/// Random variable which return a random value following a normal distribution.
#[derive(Debug)]
pub struct DeterministNormalRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed of this random variable (computed by the factory).
    my_seed: f32,
    /// Normal distribution.
    nd: MultivariateNormal,
}

impl DeterministNormalRandomVariable {
    pub fn from_config(my_seed: f32, config: NormalRandomVariableConfig) -> Self {
        assert!(
            config.mean.len().pow(2) == config.covariance.len(),
            "The length of the covariance vector should be the square of the means' one."
        );
        Self {
            my_seed,
            nd: MultivariateNormal::new(config.mean, config.covariance)
                .expect("Impossible to create the normal distribution"),
        }
    }
}

impl DeterministRandomVariable for DeterministNormalRandomVariable {
    fn generate(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.my_seed + time).to_bits() as u64);
        self.nd.sample(&mut rng).iter().map(|x| *x as f32).collect()
    }

    fn dim(&self) -> usize {
        self.nd.mean().unwrap().len()
    }
}
