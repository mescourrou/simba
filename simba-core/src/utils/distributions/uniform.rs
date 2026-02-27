use std::iter::zip;

use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;

/// Configuration for a uniform random variable.
#[config_derives]
pub struct UniformRandomVariableConfig {
    /// Minimum value of the uniform distribution.
    pub min: Vec<f32>,
    /// Maximum value of the uniform distribution.
    pub max: Vec<f32>,
}

impl Default for UniformRandomVariableConfig {
    fn default() -> Self {
        Self {
            min: vec![-1.],
            max: vec![1.],
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for UniformRandomVariableConfig {
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
                for (i, (min, max)) in
                    std::iter::zip(self.min.iter_mut(), self.max.iter_mut()).enumerate()
                {
                    ui.horizontal(|ui| {
                        ui.label(format!("{}:", i + 1));
                        ui.add(egui::DragValue::new(min).max_decimals(10));
                        ui.label("-");
                        ui.add(egui::DragValue::new(max).max_decimals(10));
                        if ui.button("X").clicked() {
                            to_remove = Some(i);
                        }
                    });
                }
                if let Some(i) = to_remove {
                    self.min.remove(i);
                    self.max.remove(i);
                }
                if ui.button("Add").clicked() {
                    self.min.push(0.);
                    self.max.push(1.);
                }
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.horizontal_top(|ui| {
            ui.vertical(|ui| {
                for (i, (min, max)) in std::iter::zip(self.min.iter(), self.max.iter()).enumerate()
                {
                    ui.horizontal(|ui| {
                        ui.label(format!("{}: {min} - {max}", i + 1));
                    });
                }
            });
        });
    }
}

/// Random variable which return a random value between a min and a max, with a uniform distribution.
#[derive(Debug, Clone)]
pub struct DeterministUniformRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed of this random variable (computed by the factory).
    my_seed: f32,
    /// Minimum value of the uniform distribution.
    min: Vec<f32>,
    /// Maximum value of the uniform distribution.
    max: Vec<f32>,
}

impl DeterministUniformRandomVariable {
    pub fn from_config(my_seed: f32, config: UniformRandomVariableConfig) -> Self {
        assert!(config.max.len() == config.min.len());
        for (min, max) in zip(&config.min, &config.max) {
            assert!(min <= max);
        }
        Self {
            my_seed,
            min: config.min,
            max: config.max,
        }
    }

    pub fn generate(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.my_seed + time).to_bits() as u64);
        let mut v = Vec::new();
        for (min, max) in zip(&self.min, &self.max) {
            v.push(min + rng.r#gen::<f32>() * (max - min));
        }
        v
    }

    pub fn dim(&self) -> usize {
        self.max.len()
    }
}
