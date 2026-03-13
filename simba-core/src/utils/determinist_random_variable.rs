//! Deterministic random-variable factory and runtime wrappers.
//!
//! This module provides reproducible random-variable generation by combining a
//! global seed with per-variable local seeds. It exposes:
//! - a factory to create deterministic random variables,
//! - runtime wrappers over supported distributions,
//! - configuration enums used across the simulator and GUI.

use std::sync::Mutex;

use rand::{Rng, SeedableRng, random};
use rand_chacha::ChaCha8Rng;
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::string_combobox};

use super::distributions::{
    exponential::{DeterministExponentialRandomVariable, ExponentialRandomVariableConfig},
    fixed::{DeterministFixedRandomVariable, FixedRandomVariableConfig},
    normal::{DeterministNormalRandomVariable, NormalRandomVariableConfig},
    poisson::{DeterministPoissonRandomVariable, PoissonRandomVariableConfig},
    uniform::{DeterministUniformRandomVariable, UniformRandomVariableConfig},
};

/// Factory to create random variables with a deterministic behavior, using a global seed.
/// 
/// The seeds of the generated random variables are computed by combining the global seed with a local seed generated for each variable, which ensures that the same sequence of random variables is generated across runs with the same global seed.
pub struct DeterministRandomVariableFactory {
    /// Global run seed.
    global_seed: Mutex<f32>,
    seed_generator: Mutex<ChaCha8Rng>,
}

impl DeterministRandomVariableFactory {
    /// Create a new factory with the given `global_seed`.
    pub fn new(global_seed: f32) -> Self {
        Self {
            global_seed: Mutex::new(global_seed),
            seed_generator: Mutex::new(ChaCha8Rng::seed_from_u64(global_seed.to_bits() as u64)),
        }
    }

    /// Create a new random variable with the given configuration.
    pub fn make_variable(&self, config: RandomVariableTypeConfig) -> DeterministRandomVariable {
        let local_seed = self.seed_generator.lock().unwrap().r#gen::<f32>() * 1000000.;
        match config {
            RandomVariableTypeConfig::None => {
                DeterministRandomVariable::Fixed(DeterministFixedRandomVariable::from_config(
                    local_seed,
                    FixedRandomVariableConfig::default(),
                ))
            }
            RandomVariableTypeConfig::Fixed(c) => DeterministRandomVariable::Fixed(
                DeterministFixedRandomVariable::from_config(local_seed, c),
            ),
            RandomVariableTypeConfig::Uniform(c) => DeterministRandomVariable::Uniform(
                DeterministUniformRandomVariable::from_config(local_seed, c),
            ),
            RandomVariableTypeConfig::Normal(c) => DeterministRandomVariable::Normal(
                DeterministNormalRandomVariable::from_config(local_seed, c),
            ),
            RandomVariableTypeConfig::Poisson(c) => DeterministRandomVariable::Poisson(
                DeterministPoissonRandomVariable::from_config(local_seed, c),
            ),
            RandomVariableTypeConfig::Exponential(c) => DeterministRandomVariable::Exponential(
                DeterministExponentialRandomVariable::from_config(local_seed, c),
            ),
        }
    }

    /// Set the global seed and reset the internal seed generator.
    pub fn set_global_seed(&self, seed: f32) {
        *self.global_seed.lock().unwrap() = seed;
        *self.seed_generator.lock().unwrap() = ChaCha8Rng::seed_from_u64(seed.to_bits() as u64);
    }

    /// Get the current global seed.
    pub fn global_seed(&self) -> f32 {
        *self.global_seed.lock().unwrap()
    }
}

impl Default for DeterministRandomVariableFactory {
    fn default() -> Self {
        let global_seed = random::<f32>() * 1000000.;
        Self {
            global_seed: Mutex::new(global_seed),
            seed_generator: Mutex::new(ChaCha8Rng::seed_from_u64(global_seed.to_bits() as u64)),
        }
    }
}

#[derive(Debug, Clone)]
/// Runtime wrapper around supported deterministic random-variable types.
pub enum DeterministRandomVariable {
    /// Fixed value
    Fixed(DeterministFixedRandomVariable),
    /// Uniform distribution.
    Uniform(DeterministUniformRandomVariable),
    /// Normal distribution.
    Normal(DeterministNormalRandomVariable),
    /// Poisson distribution.
    Poisson(DeterministPoissonRandomVariable),
    /// Exponential distribution.
    Exponential(DeterministExponentialRandomVariable),
}

impl DeterministRandomVariable {
    /// Generate one sample vector for the given simulation `time`.
    pub fn generate(&self, time: f32) -> Vec<f32> {
        match self {
            DeterministRandomVariable::Fixed(v) => v.generate(time),
            DeterministRandomVariable::Uniform(v) => v.generate(time),
            DeterministRandomVariable::Normal(v) => v.generate(time),
            DeterministRandomVariable::Poisson(v) => v.generate(time),
            DeterministRandomVariable::Exponential(v) => v.generate(time),
        }
    }

    /// Return the output dimension of the wrapped random variable.
    pub fn dim(&self) -> usize {
        match self {
            DeterministRandomVariable::Fixed(v) => v.dim(),
            DeterministRandomVariable::Uniform(v) => v.dim(),
            DeterministRandomVariable::Normal(v) => v.dim(),
            DeterministRandomVariable::Poisson(v) => v.dim(),
            DeterministRandomVariable::Exponential(v) => v.dim(),
        }
    }
}

/// Configuration of random-variable type and parameters.
#[config_derives]
pub enum RandomVariableTypeConfig {
    /// No random variable
    None,
    /// Fixed value
    #[check]
    Fixed(FixedRandomVariableConfig),
    /// Uniform distribution
    #[check]
    Uniform(UniformRandomVariableConfig),
    /// Normal distribution
    #[check]
    Normal(NormalRandomVariableConfig),
    /// Poisson distribution.
    #[check]
    Poisson(PoissonRandomVariableConfig),
    /// Exponential distribution.
    #[check]
    Exponential(ExponentialRandomVariableConfig),
}

impl RandomVariableTypeConfig {
    /// Return the output dimension implied by this configuration.
    pub fn dim(&self) -> usize {
        match self {
            RandomVariableTypeConfig::None => 0,
            RandomVariableTypeConfig::Fixed(c) => c.values.len(),
            RandomVariableTypeConfig::Uniform(c) => c.max.len(),
            RandomVariableTypeConfig::Normal(c) => c.mean.len(),
            RandomVariableTypeConfig::Poisson(c) => c.lambda.len(),
            RandomVariableTypeConfig::Exponential(c) => c.lambda.len(),
        }
    }
}

impl Default for RandomVariableTypeConfig {
    fn default() -> Self {
        Self::None
    }
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
        unique_id: &str,
    ) {
        let mut current_str = self.to_string();
        let possible_types = [
            "None",
            "Fixed",
            "Uniform",
            "Normal",
            "Poisson",
            "Exponential",
        ];
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

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.horizontal(|ui| {
            ui.label(format!("Type: {}", self));
        });

        match self {
            RandomVariableTypeConfig::None => {
                ui.label("None");
            }
            RandomVariableTypeConfig::Fixed(c) => c.show(ui, ctx, unique_id),
            RandomVariableTypeConfig::Uniform(c) => c.show(ui, ctx, unique_id),
            RandomVariableTypeConfig::Normal(c) => c.show(ui, ctx, unique_id),
            RandomVariableTypeConfig::Poisson(c) => c.show(ui, ctx, unique_id),
            RandomVariableTypeConfig::Exponential(c) => c.show(ui, ctx, unique_id),
        };
    }
}

#[cfg(feature = "gui")]
impl RandomVariableTypeConfig {
    /// Render a mutable list editor for random-variable configurations in the GUI.
    pub fn show_vector_mut(
        vec: &mut Vec<RandomVariableTypeConfig>,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &crate::simulator::SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
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

    /// Render a read-only list view for random-variable configurations in the GUI.
    pub fn show_vector(
        vec: &[RandomVariableTypeConfig],
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        unique_id: &str,
    ) {
        ui.vertical(|ui| {
            ui.label("Distributions: ");
            for (i, dist) in vec.iter().enumerate() {
                ui.horizontal_top(|ui| {
                    let dist_unique_id = format!("distribution-{i}-{unique_id}");
                    dist.show(ui, ctx, &dist_unique_id);
                });
            }
        });
    }
}

#[cfg(feature = "gui")]
/// Render a GUI widget to display and regenerate a random seed.
pub fn seed_generation_component(
    seed: &mut f32,
    ui: &mut egui::Ui,
    _buffer_stack: &mut std::collections::BTreeMap<String, String>,
    _unique_id: &str,
) {
    use rand::random;

    ui.horizontal(|ui| {
        ui.add(egui::DragValue::new(seed).max_decimals(40));
        if ui.button("Generate").clicked() {
            *seed = random::<f32>() * 1000000.;
        }
    });
}
