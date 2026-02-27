//! Misdetection faults
//!
//! Remark: the order of the application of the random value is alphabetical on the name of the observation variables if no order is specified.

use std::sync::Arc;

use log::debug;
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::{
    environment::Environment,
    logger::is_enabled,
    sensors::SensorObservation,
    utils::{
        determinist_random_variable::DeterministRandomVariableFactory,
        distributions::bernouilli::{
            BernouilliRandomVariableConfig, DeterministBernouilliRandomVariable,
        },
    },
};

use super::fault_model::FaultModel;

#[config_derives]
pub struct MisdetectionFaultConfig {
    #[check(eq(self.apparition.probability.len(), 1))]
    pub apparition: BernouilliRandomVariableConfig,
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
}

impl FaultModel for MisdetectionFault {
    fn add_faults(
        &mut self,
        _time: f32,
        seed: f32,
        obs_list: &mut Vec<SensorObservation>,
        _obs_type: SensorObservation,
        _environment: &Arc<Environment>,
    ) {
        let obs_seed_increment = 1. / (100. * obs_list.len() as f32);
        let mut seed = seed;
        for i in (0..obs_list.len()).rev() {
            seed += obs_seed_increment;
            if self.apparition.generate(seed)[0] > 0. {
                if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
                    debug!("Remove observation {i}");
                }
                obs_list.remove(i);
            }
        }
    }
}

pub struct MisdetectionRecord {}
