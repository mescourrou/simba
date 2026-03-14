//! Misdetection fault model.
//!
//! This module defines a simple fault model that randomly drops observations.
//! The drop decision is sampled from a Bernoulli distribution configured through
//! [`MisdetectionFaultConfig`] and executed by [`MisdetectionFault`].

use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::utils::{
    determinist_random_variable::DeterministRandomVariableFactory,
    distributions::bernouilli::{
        BernouilliRandomVariableConfig, DeterministBernouilliRandomVariable,
    },
};

/// Configuration of the misdetection fault model.
///
/// This configuration controls how often an observation is considered missed.
/// The `apparition` random variable is a Bernoulli distribution where `1` means
/// the observation is detected and `0` means it is dropped.
///
/// # Example
/// ```yaml
/// faults:
///  - type: Misdetection
///    apparition:
///      probability: [ 0.2 ]
/// ```
#[config_derives]
pub struct MisdetectionFaultConfig {
    /// Bernoulli distribution used to decide whether an observation is detected.
    ///
    /// This must be one-dimensional for this model.
    pub apparition: BernouilliRandomVariableConfig,
}

impl Check for MisdetectionFaultConfig {
    fn do_check(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.apparition.probability.len() != 1 {
            errors.push(format!(
                "Apparition probability should be of length 1, got {}",
                self.apparition.probability.len()
            ));
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

impl Default for MisdetectionFaultConfig {
    fn default() -> Self {
        Self {
            apparition: BernouilliRandomVariableConfig {
                probability: vec![0.1],
            },
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for MisdetectionFaultConfig {
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
        });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.label("Apparition probability: ");
                self.apparition.show(ui, ctx, unique_id);
            });
        });
    }
}

/// Runtime misdetection fault model.
///
/// This type samples a deterministic Bernoulli random variable to decide whether
/// an observation should be kept or removed.
#[derive(Debug)]
pub struct MisdetectionFault {
    apparition: DeterministBernouilliRandomVariable,
}

impl MisdetectionFault {
    /// Builds a misdetection fault model from [`MisdetectionFaultConfig`].
    pub fn from_config(
        config: &MisdetectionFaultConfig,
        va_factory: &DeterministRandomVariableFactory,
        _initial_time: f32,
    ) -> Self {
        Self {
            apparition: DeterministBernouilliRandomVariable::from_config(
                va_factory.global_seed(),
                config.apparition.clone(),
            ),
        }
    }

    /// Returns whether the observation is detected for the provided sampling seed.
    ///
    /// Returns `true` when the Bernoulli sample is `1`, and `false` otherwise.
    pub fn detected(&mut self, seed: f32) -> bool {
        self.apparition.generate(seed)[0] > 0. // = 1
    }
}

/// Record type for misdetection faults.
pub struct MisdetectionRecord {}
