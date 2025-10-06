/*!
# Determinist Random Variable

This module provides a way to create random variables with a deterministic behavior. This is
useful to have a reproducible behavior.

This module proposes a factory to create random variables with a deterministic behavior. The
factory is created with a global seed, and each random variable created with this factory will
have a unique seed, to have a different series of random numbers.

The random variables can be of different types:
- Fixed: always return the same value
- Uniform: return a random value between a min and a max, with a uniform distribution
- Normal: return a random value following a normal distribution
Other types can be added in the future.

 */

use config_checker::macros::Check;
use rand::random;
use serde::{Deserialize, Serialize};
use simba_macros::EnumToString;

#[cfg(feature = "gui")]
use crate::gui::{utils::string_combobox, UIComponent};

use super::distributions::{
    exponential::{DeterministExponentialRandomVariable, ExponentialRandomVariableConfig},
    fixed::{DeterministFixedRandomVariable, FixedRandomVariableConfig},
    normal::{DeterministNormalRandomVariable, NormalRandomVariableConfig},
    poisson::{DeterministPoissonRandomVariable, PoissonRandomVariableConfig},
    uniform::{DeterministUniformRandomVariable, UniformRandomVariableConfig},
};

/// Factory to create random variables with a deterministic behavior, using a global seed.
pub struct DeterministRandomVariableFactory {
    /// Global run seed.
    pub global_seed: f32,
}

impl DeterministRandomVariableFactory {
    /// Create a new factory with the given `global_seed`.
    pub fn new(global_seed: f32) -> Self {
        Self { global_seed }
    }

    /// Create a new random variable with the given configuration.
    pub fn make_variable(
        &self,
        config: RandomVariableTypeConfig,
    ) -> Box<dyn DeterministRandomVariable> {
        match config {
            RandomVariableTypeConfig::None => {
                Box::new(DeterministFixedRandomVariable::from_config(
                    self.global_seed,
                    FixedRandomVariableConfig::default(),
                ))
            }
            RandomVariableTypeConfig::Fixed(c) => Box::new(
                DeterministFixedRandomVariable::from_config(self.global_seed, c),
            ),
            RandomVariableTypeConfig::Uniform(c) => Box::new(
                DeterministUniformRandomVariable::from_config(self.global_seed, c),
            ),
            RandomVariableTypeConfig::Normal(c) => Box::new(
                DeterministNormalRandomVariable::from_config(self.global_seed, c),
            ),
            RandomVariableTypeConfig::Poisson(c) => Box::new(
                DeterministPoissonRandomVariable::from_config(self.global_seed, c),
            ),
            RandomVariableTypeConfig::Exponential(c) => Box::new(
                DeterministExponentialRandomVariable::from_config(self.global_seed, c),
            ),
        }
    }
}

impl Default for DeterministRandomVariableFactory {
    fn default() -> Self {
        Self { global_seed: random() }
    }
}

/// Trait for a random variable with a deterministic behavior.
pub trait DeterministRandomVariable:
    std::fmt::Debug + std::marker::Send + std::marker::Sync
{
    fn gen(&self, time: f32) -> Vec<f32>;
    fn dim(&self) -> usize;
}

/// Configuration of the random variable: fixed, uniform or normal.
#[derive(Serialize, Deserialize, Debug, Clone, Check, EnumToString)]
#[serde(deny_unknown_fields)]
pub enum RandomVariableTypeConfig {
    /// No random variable
    None,
    /// Fixed value
    Fixed(FixedRandomVariableConfig),
    /// Uniform distribution
    Uniform(UniformRandomVariableConfig),
    /// Normal distribution
    Normal(NormalRandomVariableConfig),
    Poisson(PoissonRandomVariableConfig),
    Exponential(ExponentialRandomVariableConfig),
}

#[cfg(feature = "gui")]
impl UIComponent for RandomVariableTypeConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &crate::simulator::SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        let mut current_str = self.to_string();
        let possible_types = vec![
            "None",
            "Fixed",
            "Uniform",
            "Normal",
            "Poisson",
            "Exponential",
        ]
        .iter()
        .map(|x| String::from(*x))
        .collect();
        ui.horizontal(|ui| {
            ui.label("Type:");

            string_combobox(
                ui,
                &possible_types,
                &mut current_str,
                format!("random-variable-choice-{}", unique_id),
            );
        });
        if current_str != self.to_string() {
            match current_str.as_str() {
                "None" => *self = RandomVariableTypeConfig::None,
                "Fixed" => {
                    *self = RandomVariableTypeConfig::Fixed(FixedRandomVariableConfig::default())
                }
                "Uniform" => {
                    *self =
                        RandomVariableTypeConfig::Uniform(UniformRandomVariableConfig::default())
                }
                "Normal" => {
                    *self = RandomVariableTypeConfig::Normal(NormalRandomVariableConfig::default())
                }
                "Poisson" => {
                    *self =
                        RandomVariableTypeConfig::Poisson(PoissonRandomVariableConfig::default())
                }
                "Exponential" => {
                    *self = RandomVariableTypeConfig::Exponential(
                        ExponentialRandomVariableConfig::default(),
                    )
                }
                _ => panic!("Where did you find this value?"),
            };
        }
        match self {
            RandomVariableTypeConfig::None => {
                ui.label("None");
            }
            RandomVariableTypeConfig::Fixed(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            RandomVariableTypeConfig::Uniform(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            RandomVariableTypeConfig::Normal(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            RandomVariableTypeConfig::Poisson(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            RandomVariableTypeConfig::Exponential(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
        };
    }

    fn show(
        &self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        unique_id: &String,
    ) {
        ui.horizontal(|ui| {
            ui.label(format!("Type: {}", self.to_string()));
        });
        
        match self {
            RandomVariableTypeConfig::None => {
                ui.label("None");
            }
            RandomVariableTypeConfig::Fixed(c) => c.show(
                ui,
                ctx,
                unique_id,
            ),
            RandomVariableTypeConfig::Uniform(c) => c.show(
                ui,
                ctx,
                unique_id,
            ),
            RandomVariableTypeConfig::Normal(c) => c.show(
                ui,
                ctx,
                unique_id,
            ),
            RandomVariableTypeConfig::Poisson(c) => c.show(
                ui,
                ctx,
                unique_id,
            ),
            RandomVariableTypeConfig::Exponential(c) => c.show(
                ui,
                ctx,
                unique_id,
            ),
        };
    }
}

#[cfg(feature = "gui")]
impl RandomVariableTypeConfig {
    pub fn show_vector_mut(
        vec: &mut Vec<RandomVariableTypeConfig>,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &crate::simulator::SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        ui.vertical(|ui| {
            ui.label("Distributions: ");
            let mut to_remove = None;
            for (i, dist) in vec.iter_mut().enumerate() {
                ui.horizontal_top(|ui| {
                    let dist_unique_id = format!("distribution-{i}-{unique_id}");
                    dist.show_mut(
                        ui,
                        ctx,
                        buffer_stack,
                        global_config,
                        current_node_name,
                        &dist_unique_id,
                    );
                    if ui.button("X").clicked() {
                        to_remove = Some(i);
                    }
                });
            }
            if let Some(i) = to_remove {
                vec.remove(i);
            }
            if ui.button("Add").clicked() {
                vec.push(Self::None);
            }
        });
    }

    pub fn show_vector(
        vec: &Vec<RandomVariableTypeConfig>,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        unique_id: &String,
    ) {
        ui.vertical(|ui| {
            ui.label("Distributions: ");
            for (i, dist) in vec.iter().enumerate() {
                ui.horizontal_top(|ui| {
                    let dist_unique_id = format!("distribution-{i}-{unique_id}");
                    dist.show(
                        ui,
                        ctx,
                        &dist_unique_id,
                    );
                });
            }
        });
    }
}

#[cfg(feature = "gui")]
pub fn seed_generation_component(
    seed: &mut f32,
    ui: &mut egui::Ui,
    _buffer_stack: &mut std::collections::BTreeMap<String, String>,
    _unique_id: &String,
) {
    use rand::random;

    ui.horizontal(|ui| {
        ui.add(egui::DragValue::new(seed).max_decimals(40));
        if ui.button("Generate").clicked() {
            *seed = random::<f32>() * 1000000.;
        }
    });
}
