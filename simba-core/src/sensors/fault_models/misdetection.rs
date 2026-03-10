//! Misdetection faults
//!
//! Remark: the order of the application of the random value is alphabetical on the name of the observation variables if no order is specified.

use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::utils::{
    determinist_random_variable::DeterministRandomVariableFactory,
    distributions::bernouilli::{
        BernouilliRandomVariableConfig, DeterministBernouilliRandomVariable,
    },
};

#[config_derives]
pub struct MisdetectionFaultConfig {
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

#[derive(Debug)]
pub struct MisdetectionFault {
    apparition: DeterministBernouilliRandomVariable,
}

impl MisdetectionFault {
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

    pub fn detected(&mut self, seed: f32) -> bool {
        self.apparition.generate(seed)[0] > 0. // = 1
    }
}

pub struct MisdetectionRecord {}
