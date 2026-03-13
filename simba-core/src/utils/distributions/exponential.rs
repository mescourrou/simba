//! Exponential distribution random-variable utilities.
//!
//! This module provides configuration and deterministic sampling utilities for
//! exponential random variables used by the simulator.

use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use simba_macros::config_derives;
use statrs::distribution::Exp;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;

/// Configuration for an exponential random variable.
#[config_derives]
pub struct ExponentialRandomVariableConfig {
    /// Rate parameters (`lambda`) of the exponential distributions.
    pub lambda: Vec<f64>,
}

impl Check for ExponentialRandomVariableConfig {
    fn do_check(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.lambda.is_empty() {
            errors.push("Lambda vector cannot be empty.".to_string());
        }
        if self.lambda.iter().any(|p| *p < 0.) {
            errors.push("Lambdas must be non-negative.".to_string());
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

impl Default for ExponentialRandomVariableConfig {
    fn default() -> Self {
        Self { lambda: vec![1.] }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ExponentialRandomVariableConfig {
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
/// Deterministic exponential random variable generator.
///
/// Sampling is reproducible for the same seed and time input.
pub struct DeterministExponentialRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed of this random variable (computed by the factory).
    my_seed: f32,
    /// Probability of the Exponential distribution.
    exponential: Vec<Exp>,
}

impl DeterministExponentialRandomVariable {
    /// Build a deterministic exponential random variable from configuration.
    ///
    /// `my_seed` should be a deterministic seed component unique to this
    /// variable instance.
    pub fn from_config(my_seed: f32, config: ExponentialRandomVariableConfig) -> Self {
        assert!(config.lambda.iter().all(|x| *x >= 0.));
        Self {
            my_seed,
            exponential: config
                .lambda
                .iter()
                .map(|lambda| Exp::new(*lambda).unwrap())
                .collect(),
        }
    }

    /// Generate one sample vector at a given simulation `time`.
    ///
    /// The produced values are reproducible for the same `(my_seed, time)` pair.
    pub fn generate(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.my_seed + time).to_bits() as u64);
        let mut v = Vec::new();
        for p in &self.exponential {
            v.push(p.sample(&mut rng) as f32);
        }
        v
    }

    /// Return the output dimension of the random variable.
    pub fn dim(&self) -> usize {
        self.exponential.len()
    }
}
