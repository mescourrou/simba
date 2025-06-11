use std::iter::zip;

use config_checker::macros::Check;
use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};

#[cfg(feature = "gui")]
use crate::gui::UIComponent;

use crate::{
    utils::determinist_random_variable::{self, DeterministRandomVariable},
};
use crate::utils::format_f32;

/// Configuration for a uniform random variable.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct UniformRandomVariableConfig {
    /// Random seed for this random variable.
    #[serde(serialize_with = "format_f32")]
    pub unique_seed: f32,
    /// Minimum value of the uniform distribution.
    pub min: Vec<f32>,
    /// Maximum value of the uniform distribution.
    pub max: Vec<f32>,
}

impl Default for UniformRandomVariableConfig {
    fn default() -> Self {
        Self {
            unique_seed: 0.,
            min: vec![-1.],
            max: vec![1.],
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for UniformRandomVariableConfig {
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

/// Random variable which return a random value between a min and a max, with a uniform distribution.
#[derive(Debug)]
pub struct DeterministUniformRandomVariable {
    /// Seed used, which is the global seed from the factory + the unique seed from the configuration.
    global_seed: f32,
    /// Minimum value of the uniform distribution.
    min: Vec<f32>,
    /// Maximum value of the uniform distribution.
    max: Vec<f32>,
}

impl DeterministUniformRandomVariable {
    pub fn from_config(global_seed: f32, config: UniformRandomVariableConfig) -> Self {
        assert!(config.max.len() == config.min.len());
        for (min, max) in zip(&config.min, &config.max) {
            assert!(min <= max);
        }
        Self {
            global_seed: global_seed + config.unique_seed,
            min: config.min,
            max: config.max,
        }
    }
}

impl DeterministRandomVariable for DeterministUniformRandomVariable {
    fn gen(&self, time: f32) -> Vec<f32> {
        let mut rng = ChaCha8Rng::seed_from_u64((self.global_seed + time).to_bits() as u64);
        let mut v = Vec::new();
        for (min, max) in zip(&self.min, &self.max) {
            v.push(min + rng.gen::<f32>() * (max - min));
        }
        v
    }

    fn dim(&self) -> usize {
        self.max.len()
    }
}
