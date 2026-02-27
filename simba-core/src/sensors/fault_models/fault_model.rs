//! TODO: Lots of code duplication between fault models, need to find a way to factorize

use std::{fmt::Debug, sync::Arc};

use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::{
    environment::Environment,
    errors::SimbaResult,
    sensors::{
        SensorObservation,
        fault_models::python_fault_model::{PythonFaultModel, PythonFaultModelConfig},
    },
    simulator::SimulatorConfig,
    utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use super::{
    additive_observation_centered_polar::{
        AdditiveObservationCenteredPolarFault, AdditiveObservationCenteredPolarFaultConfig,
    },
    additive_robot_centered::{AdditiveRobotCenteredFault, AdditiveRobotCenteredFaultConfig},
    additive_robot_centered_polar::{
        AdditiveRobotCenteredPolarFault, AdditiveRobotCenteredPolarFaultConfig,
    },
    clutter::{ClutterFault, ClutterFaultConfig},
    misassociation::{MisassociationFault, MisassociationFaultConfig},
    misdetection::{MisdetectionFault, MisdetectionFaultConfig},
};

#[config_derives]
pub enum FaultModelConfig {
    AdditiveRobotCentered(AdditiveRobotCenteredFaultConfig),
    AdditiveRobotCenteredPolar(AdditiveRobotCenteredPolarFaultConfig),
    AdditiveObservationCenteredPolar(AdditiveObservationCenteredPolarFaultConfig),
    Clutter(ClutterFaultConfig),
    Misdetection(MisdetectionFaultConfig),
    Misassociation(MisassociationFaultConfig),
    Python(PythonFaultModelConfig),
}

#[cfg(feature = "gui")]
impl UIComponent for FaultModelConfig {
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
            .id_salt(format!("fault-model-{}", unique_id))
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
                    Self::AdditiveRobotCenteredPolar(cfg) => cfg.show_mut(
                        ui,
                        ctx,
                        buffer_stack,
                        global_config,
                        current_node_name,
                        unique_id,
                    ),
                    Self::AdditiveObservationCenteredPolar(cfg) => cfg.show_mut(
                        ui,
                        ctx,
                        buffer_stack,
                        global_config,
                        current_node_name,
                        unique_id,
                    ),
                    Self::Clutter(cfg) => cfg.show_mut(
                        ui,
                        ctx,
                        buffer_stack,
                        global_config,
                        current_node_name,
                        unique_id,
                    ),
                    Self::Misdetection(cfg) => cfg.show_mut(
                        ui,
                        ctx,
                        buffer_stack,
                        global_config,
                        current_node_name,
                        unique_id,
                    ),
                    Self::Misassociation(cfg) => cfg.show_mut(
                        ui,
                        ctx,
                        buffer_stack,
                        global_config,
                        current_node_name,
                        unique_id,
                    ),
                    Self::Python(cfg) => cfg.show_mut(
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
            .id_salt(format!("fault-model-{}", unique_id))
            .show(ui, |ui| {
                match self {
                    Self::AdditiveRobotCentered(cfg) => cfg.show(ui, ctx, unique_id),
                    Self::AdditiveRobotCenteredPolar(cfg) => cfg.show(ui, ctx, unique_id),
                    Self::AdditiveObservationCenteredPolar(cfg) => cfg.show(ui, ctx, unique_id),
                    Self::Clutter(cfg) => cfg.show(ui, ctx, unique_id),
                    Self::Misdetection(cfg) => cfg.show(ui, ctx, unique_id),
                    Self::Misassociation(cfg) => cfg.show(ui, ctx, unique_id),
                    Self::Python(cfg) => cfg.show(ui, ctx, unique_id),
                };
            });
    }
}

#[cfg(feature = "gui")]
impl FaultModelConfig {
    pub fn show_faults_mut(
        faults: &mut Vec<FaultModelConfig>,
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
                let unique_fault_id = format!("fault-{i}-{unique_id}");
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
            let buffer_key = format!("selected-new-fault-{unique_id}");
            if !buffer_stack.contains_key(&buffer_key) {
                buffer_stack.insert(buffer_key.clone(), "AdditiveRobotCentered".to_string());
            }
            string_combobox(
                ui,
                &FaultModelConfig::to_vec()
                    .iter()
                    .map(|x: &&str| String::from(*x))
                    .collect(),
                buffer_stack.get_mut(&buffer_key).unwrap(),
                format!("fault-choice-{}", unique_id),
            );
            if ui.button("Add").clicked() {
                let selected_fault = buffer_stack.get(&buffer_key).unwrap();
                match selected_fault.as_str() {
                    "AdditiveRobotCentered" => {
                        faults.push(FaultModelConfig::AdditiveRobotCentered(
                            AdditiveRobotCenteredFaultConfig::default(),
                        ))
                    }
                    "AdditiveRobotCenteredPolar" => {
                        faults.push(FaultModelConfig::AdditiveRobotCenteredPolar(
                            AdditiveRobotCenteredPolarFaultConfig::default(),
                        ))
                    }
                    "AdditiveObservationCenteredPolar" => {
                        faults.push(FaultModelConfig::AdditiveObservationCenteredPolar(
                            AdditiveObservationCenteredPolarFaultConfig::default(),
                        ))
                    }
                    "Clutter" => {
                        faults.push(FaultModelConfig::Clutter(ClutterFaultConfig::default()))
                    }
                    "Misdetection" => faults.push(FaultModelConfig::Misdetection(
                        MisdetectionFaultConfig::default(),
                    )),
                    "Misassociation" => faults.push(FaultModelConfig::Misassociation(
                        MisassociationFaultConfig::default(),
                    )),
                    _ => panic!("Where did you find this fault?"),
                };
            }
        });
    }

    pub fn show_faults(
        faults: &[FaultModelConfig],
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        unique_id: &str,
    ) {
        ui.label("Faults:");
        for (i, fault) in faults.iter().enumerate() {
            ui.horizontal_top(|ui| {
                let unique_fault_id = format!("fault-{i}-{unique_id}");
                fault.show(ui, ctx, &unique_fault_id);
            });
        }
    }
}

pub fn make_fault_model_from_config(
    config: &FaultModelConfig,
    global_config: &SimulatorConfig,
    robot_name: &String,
    va_factory: &DeterministRandomVariableFactory,
    initial_time: f32,
) -> Box<dyn FaultModel> {
    match &config {
        FaultModelConfig::AdditiveRobotCentered(cfg) => Box::new(
            AdditiveRobotCenteredFault::from_config(cfg, va_factory, initial_time),
        ) as Box<dyn FaultModel>,
        FaultModelConfig::AdditiveRobotCenteredPolar(cfg) => Box::new(
            AdditiveRobotCenteredPolarFault::from_config(cfg, va_factory, initial_time),
        ) as Box<dyn FaultModel>,
        FaultModelConfig::AdditiveObservationCenteredPolar(cfg) => Box::new(
            AdditiveObservationCenteredPolarFault::from_config(cfg, va_factory, initial_time),
        ) as Box<dyn FaultModel>,
        FaultModelConfig::Clutter(cfg) => {
            Box::new(ClutterFault::from_config(cfg, va_factory, initial_time))
                as Box<dyn FaultModel>
        }
        FaultModelConfig::Misdetection(cfg) => Box::new(MisdetectionFault::from_config(
            cfg,
            va_factory,
            initial_time,
        )) as Box<dyn FaultModel>,
        FaultModelConfig::Misassociation(cfg) => Box::new(MisassociationFault::from_config(
            cfg,
            global_config,
            robot_name,
            va_factory,
            initial_time,
        )) as Box<dyn FaultModel>,
        FaultModelConfig::Python(cfg) => Box::new(
            PythonFaultModel::from_config(cfg, global_config, initial_time)
                .expect("Failed to create Python Fault Model"),
        ) as Box<dyn FaultModel>,
    }
}

pub trait FaultModel: Debug + Sync + Send {
    fn post_init(&mut self, _node: &mut crate::node::Node, _initial_time: f32) -> SimbaResult<()> {
        Ok(())
    }

    fn add_faults(
        &mut self,
        time: f32,
        seed: f32,
        obs_list: &mut Vec<SensorObservation>,
        obs_type: SensorObservation,
        environment: &Arc<Environment>,
    );
}
