//! TODO: Lots of code duplication between fault models, need to find a way to factorize

use std::{fmt::Debug, sync::Arc};

use config_checker::macros::Check;
use serde::{Deserialize, Serialize};
use simba_macros::{EnumToString, ToVec};

#[cfg(feature = "gui")]
use crate::{gui::UIComponent, simulator::SimulatorConfig};
use crate::{
    physics::fault_models::additive_robot_centered::{
        AdditiveRobotCenteredPhysicsFault, AdditiveRobotCenteredPhysicsFaultConfig,
    },
    state_estimators::state_estimator::State,
    utils::determinist_random_variable::DeterministRandomVariableFactory,
};

#[derive(Debug, Serialize, Deserialize, Clone, Check, ToVec, EnumToString)]
#[serde(deny_unknown_fields)]
pub enum PhysicsFaultModelConfig {
    AdditiveRobotCentered(AdditiveRobotCenteredPhysicsFaultConfig),
}

#[cfg(feature = "gui")]
impl UIComponent for PhysicsFaultModelConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        let self_str = self.to_string();
        egui::CollapsingHeader::new(self_str)
            .id_salt(format!("physics-fault-model-{}", unique_id))
            .show(ui, |ui| {
                match self {
                    Self::AdditiveRobotCentered(cfg) => cfg.show_mut(
                        ui,
                        ctx,
                        buffer_stack,
                        global_config,
                        current_node_name,
                        unique_id,
                    ),
                };
            });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        let self_str = self.to_string();
        egui::CollapsingHeader::new(self_str)
            .id_salt(format!("physics-fault-model-{}", unique_id))
            .show(ui, |ui| {
                match self {
                    Self::AdditiveRobotCentered(cfg) => cfg.show(ui, ctx, unique_id),
                };
            });
    }
}

#[cfg(feature = "gui")]
impl PhysicsFaultModelConfig {
    pub fn show_faults_mut(
        faults: &mut Vec<PhysicsFaultModelConfig>,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        use crate::{gui::utils::string_combobox, utils::enum_tools::ToVec};

        ui.label("Faults:");
        let mut fault_to_remove = None;
        for (i, fault) in faults.iter_mut().enumerate() {
            ui.horizontal_top(|ui| {
                let unique_fault_id = format!("physics-fault-{i}-{unique_id}");
                fault.show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    &unique_fault_id,
                );

                if ui.button("X").clicked() {
                    fault_to_remove = Some(i);
                }
            });
        }
        if let Some(i) = fault_to_remove {
            faults.remove(i);
        }

        ui.horizontal(|ui| {
            let buffer_key = format!("selected-new-physics-fault-{unique_id}");
            if !buffer_stack.contains_key(&buffer_key) {
                buffer_stack.insert(buffer_key.clone(), "AdditiveRobotCentered".to_string());
            }
            string_combobox(
                ui,
                &PhysicsFaultModelConfig::to_vec()
                    .iter()
                    .map(|x| String::from(*x))
                    .collect(),
                buffer_stack.get_mut(&buffer_key).unwrap(),
                format!("physics-fault-choice-{}", unique_id),
            );
            if ui.button("Add").clicked() {
                let selected_fault = buffer_stack.get(&buffer_key).unwrap();
                match selected_fault.as_str() {
                    "AdditiveRobotCentered" => {
                        use crate::physics::fault_models::additive_robot_centered::AdditiveRobotCenteredPhysicsFaultConfig;

                        faults.push(PhysicsFaultModelConfig::AdditiveRobotCentered(
                            AdditiveRobotCenteredPhysicsFaultConfig::default(),
                        ))
                    },
                    _ => panic!("Where did you find this fault?"),
                };
            }
        });
    }

    pub fn show_faults(
        faults: &[PhysicsFaultModelConfig],
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        unique_id: &str,
    ) {
        ui.label("Faults:");
        for (i, fault) in faults.iter().enumerate() {
            ui.horizontal_top(|ui| {
                let unique_fault_id = format!("physics-fault-{i}-{unique_id}");
                fault.show(ui, ctx, &unique_fault_id);
            });
        }
    }
}

pub fn make_physics_fault_model_from_config(
    config: &PhysicsFaultModelConfig,
    _robot_name: &String,
    va_factory: &Arc<DeterministRandomVariableFactory>,
) -> Box<dyn PhysicsFaultModel> {
    match &config {
        PhysicsFaultModelConfig::AdditiveRobotCentered(cfg) => Box::new(
            AdditiveRobotCenteredPhysicsFault::from_config(cfg, va_factory),
        )
            as Box<dyn PhysicsFaultModel>,
    }
}

pub trait PhysicsFaultModel: Debug + Sync + Send {
    fn add_faults(&self, time: f32, state: &mut State);
}
