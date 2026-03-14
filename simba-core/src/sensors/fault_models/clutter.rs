//! Clutter fault model.
//!
//! This module defines a fault model that injects synthetic observations (clutter)
//! into sensor outputs.
//! The behavior is configured through [`ClutterFaultConfig`], then executed at runtime
//! by [`ClutterFault`].
//! Clutter generation first samples how many synthetic observations must be created,
//! then samples observation values from configured random distributions.

use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::{
    UIComponent,
    utils::{enum_combobox, text_singleline_with_apply},
};
use crate::utils::{
    SharedMutex,
    determinist_random_variable::{
        DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig,
    },
    distributions::{poisson::PoissonRandomVariableConfig, uniform::UniformRandomVariableConfig},
    enum_tools::EnumVariables,
};

/// Configuration of the clutter fault model.
///
/// This configuration controls how many clutter observations are generated and how
/// their variables are sampled.
/// `apparition` defines the number of generated clutter observations per call,
/// `distributions` define sampled values, and `variable_order` maps sampled dimensions
/// to concrete observation variables.
///
/// Default values:
/// - `apparition`: [`RandomVariableTypeConfig::Poisson`] with `lambda = [10.0]`
/// - `distributions`: one [`RandomVariableTypeConfig::Uniform`] with `min = [-10.0, -10.0]` and `max = [10.0, 10.0]`
/// - `variable_order`: empty vector (implicit variable order)
/// - `observation_id`: `"clutter"`
#[config_derives]
pub struct ClutterFaultConfig<SV: EnumVariables> {
    /// Random variable used to sample the number of clutter observations. Should be one-dimensional and return non-negative integer values.
    #[check]
    pub apparition: RandomVariableTypeConfig,
    /// Random-variable list used to sample clutter observation values.
    #[check]
    pub distributions: Vec<RandomVariableTypeConfig>,
    /// Optional explicit mapping order from sampled dimensions to variables.
    pub variable_order: Vec<SV>,
    /// Identifier assigned to generated clutter observations.
    pub observation_id: String,
}

impl<SV: EnumVariables> Check for ClutterFaultConfig<SV> {
    fn do_check(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.apparition.dim() != 1 {
            errors.push(format!(
                "Apparition probability should be of length 1, got {}",
                self.apparition.dim()
            ));
        }
        if !self.variable_order.is_empty()
            && self.distributions.iter().map(|d| d.dim()).sum::<usize>()
                != self.variable_order.len()
        {
            errors.push(format!("If variable order is given, its length should match the total distribution dimension. Got total distribution dimension {} and variable order length {}.",
                self.distributions.iter().map(|d| d.dim()).sum::<usize>(),
                self.variable_order.len()
            ));
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

impl<SV: EnumVariables> Default for ClutterFaultConfig<SV> {
    fn default() -> Self {
        Self {
            apparition: RandomVariableTypeConfig::Poisson(PoissonRandomVariableConfig {
                lambda: vec![10.],
            }),
            distributions: vec![RandomVariableTypeConfig::Uniform(
                UniformRandomVariableConfig {
                    min: vec![-10., -10.],
                    max: vec![10., 10.],
                },
            )],
            variable_order: Vec::new(),
            observation_id: "clutter".to_string(),
        }
    }
}

#[cfg(feature = "gui")]
impl<SV: EnumVariables> UIComponent for ClutterFaultConfig<SV> {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &crate::simulator::SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.label("Apparition probability: ");
                self.apparition.show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );
            });
            RandomVariableTypeConfig::show_vector_mut(
                &mut self.distributions,
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
            let possible_variables: Vec<SV> = SV::to_vec();
            ui.horizontal(|ui| {
                ui.label("Variable order:");
                for (i, var) in self.variable_order.iter_mut().enumerate() {
                    let unique_var_id = format!("variable-{i}-{unique_id}");
                    enum_combobox(ui, var, unique_var_id);
                }
                if !self.variable_order.is_empty() && ui.button("-").clicked() {
                    self.variable_order.pop();
                }
                if ui.button("+").clicked() {
                    self.variable_order.push(
                        possible_variables
                            .get(self.variable_order.len().min(possible_variables.len()))
                            .unwrap()
                            .clone(),
                    );
                }
            });
            let buffer_key = format!("observation-id-{unique_id}");
            text_singleline_with_apply(
                ui,
                buffer_key.as_str(),
                buffer_stack,
                &mut self.observation_id,
            );
        });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.label("Apparition probability: ");
                self.apparition.show(ui, ctx, unique_id);
            });
            RandomVariableTypeConfig::show_vector(&self.distributions, ui, ctx, unique_id);

            ui.horizontal(|ui| {
                ui.label("Variable order: ");
                for var in self.variable_order.iter() {
                    ui.label(format!("{}, ", var));
                }
            });

            ui.label(format!("Observation id: {}", self.observation_id));
        });
    }
}

/// Runtime clutter fault model.
///
/// This type uses deterministic random variables to generate additional synthetic
/// observations represented as `(id, variable_map)` pairs.
#[derive(Debug)]
pub struct ClutterFault<SV: EnumVariables> {
    apparition: SharedMutex<DeterministRandomVariable>,
    distributions: SharedMutex<Vec<DeterministRandomVariable>>,
    variable_order: Vec<SV>,
    observation_id: String,
    config: ClutterFaultConfig<SV>,
}

impl<SV: EnumVariables> ClutterFault<SV> {
    /// Builds a runtime clutter fault model from [`ClutterFaultConfig`].
    ///
    /// Validates distribution dimensions and prepares deterministic random variables
    /// using the provided [`DeterministRandomVariableFactory`].
    pub fn from_config(
        config: &ClutterFaultConfig<SV>,
        va_factory: &DeterministRandomVariableFactory,
        _initial_time: f32,
    ) -> Self {
        let distributions = Arc::new(Mutex::new(
            config
                .distributions
                .iter()
                .map(|conf| va_factory.make_variable(conf.clone()))
                .collect::<Vec<DeterministRandomVariable>>(),
        ));
        let variable_order = if !config.variable_order.is_empty() {
            assert!(
                config.variable_order.len()
                    == distributions
                        .lock()
                        .unwrap()
                        .iter()
                        .map(|d| d.dim())
                        .sum::<usize>(),
                "If variable order is given, its length must match the distribution dimension."
            );
            config.variable_order.clone()
        } else {
            SV::to_vec()
        };
        let apparition_distrib = Arc::new(Mutex::new(
            va_factory.make_variable(config.apparition.clone()),
        ));
        assert!(
            apparition_distrib.lock().unwrap().dim() == 1,
            "The dimension of the apparition distribution should be 1."
        );
        Self {
            apparition: apparition_distrib,
            distributions,
            variable_order,
            observation_id: config.observation_id.clone(),
            config: config.clone(),
        }
    }

    /// Generates clutter observations for one sampling step.
    ///
    /// Returns a list of generated observations, each represented by its observation id
    /// and a variable-value map.
    pub fn add_faults(
        &mut self,
        seed: f32,
        seed_increment: f32,
    ) -> Vec<(String, HashMap<SV, f32>)> {
        let mut seed = seed;

        let n_obs = self.apparition.lock().unwrap().generate(seed)[0]
            .abs()
            .floor() as usize;
        let mut new_obs = Vec::new();
        for _ in 0..n_obs {
            let mut o = HashMap::new();
            seed += seed_increment;
            let mut random_sample = Vec::new();
            for d in self.distributions.lock().unwrap().iter() {
                random_sample.extend_from_slice(&d.generate(seed));
            }
            if !self.variable_order.is_empty() {
                for (i, variable) in self.variable_order.iter().enumerate() {
                    o.insert(variable.clone(), random_sample[i]);
                }
            } else {
                for (i, variable) in SV::to_vec().into_iter().enumerate() {
                    if i >= random_sample.len() {
                        break;
                    }
                    o.insert(variable, random_sample[i]);
                }
            }
            new_obs.push((self.observation_id.clone(), o));
        }
        new_obs
    }

    /// Returns the configuration used to build this clutter fault model.
    pub fn config(&self) -> &ClutterFaultConfig<SV> {
        &self.config
    }
}
