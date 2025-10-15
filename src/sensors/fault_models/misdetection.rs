//! Misdetection faults
//!
//! Remark: the order of the application of the random value is alphabetical on the name of the observation variables if no order is specified.

use config_checker::macros::Check;
use log::debug;
use serde::{Deserialize, Serialize};

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::{
    logger::is_enabled,
    sensors::sensor::SensorObservation,
    utils::{
        determinist_random_variable::{
            DeterministRandomVariable, DeterministRandomVariableFactory,
        },
        distributions::bernouilli::{
            BernouilliRandomVariableConfig, DeterministBernouilliRandomVariable,
        },
    },
};

use super::fault_model::FaultModel;

#[derive(Debug, Serialize, Deserialize, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct MisdetectionFaultConfig {
    #[check(eq(self.apparition.probability.len(), 1))]
    pub apparition: BernouilliRandomVariableConfig,
}

impl Default for MisdetectionFaultConfig {
    fn default() -> Self {
        Self {
            apparition: BernouilliRandomVariableConfig {
                probability: vec![0.1],
                ..Default::default()
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
        unique_id: &String,
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

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &String) {
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
    ) -> Self {
        Self {
            apparition: DeterministBernouilliRandomVariable::from_config(
                va_factory.global_seed,
                config.apparition.clone(),
            ),
        }
    }
}

impl FaultModel for MisdetectionFault {
    fn add_faults(
        &self,
        time: f32,
        period: f32,
        obs_list: &mut Vec<SensorObservation>,
        _obs_type: SensorObservation,
    ) {
        let obs_seed_increment = 1. / (100. * period);
        let mut seed = time;
        for i in (0..obs_list.len()).rev() {
            seed += obs_seed_increment;
            if self.apparition.gen(seed)[0] > 0. {
                if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
                    debug!("Remove observation {i}");
                }
                obs_list.remove(i);
            }
        }
    }
}

pub struct MisdetectionRecord {}
