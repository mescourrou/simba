//! TODO: Lots of code duplication between fault models, need to find a way to factorize

use std::{fmt::Debug, sync::Arc};

use simba_macros::{EnumToString, config_derives};

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::{
    environment::Environment,
    errors::SimbaResult,
    plugin_api,
    sensors::{
        SensorObservation,
        fault_models::{
            external_fault::{ExternalFault, ExternalFaultConfig},
            python_fault_model::{PythonFaultModel, PythonFaultModelConfig},
        },
    },
    simulator::SimulatorConfig,
    utils::{
        determinist_random_variable::DeterministRandomVariableFactory, enum_tools::EnumVariables,
    },
};

use super::{
    additive::{AdditiveFault, AdditiveFaultConfig},
    clutter::{ClutterFault, ClutterFaultConfig},
    misassociation::{MisassociationFault, MisassociationFaultConfig},
    misdetection::{MisdetectionFault, MisdetectionFaultConfig},
};

// /// Config for the sensor fault model (generic).
// ///
// /// # Generic parameters
// /// * `SVO`: Sensor Variable for the `variable_order` field: variables of the observation that will be affected by the fault.
// /// * `SVProp`: Sensor Variable for the Proportionnality option, i.e. the variable of the observation that will be used to make the fault proportionnal to.
// #[config_derives]
// pub enum FaultModelConfig<SVO: EnumVariables, SVProp: EnumVariables> {
//     #[check]
//     Additive(AdditiveFaultConfig<SVO, SVProp>),
//     #[check]
//     Clutter(ClutterFaultConfig<SVO>),
//     #[check]
//     Misdetection(MisdetectionFaultConfig),
//     #[check]
//     Misassociation(MisassociationFaultConfig),
//     #[check]
//     Python(PythonFaultModelConfig),
//     #[check]
//     External(ExternalFaultConfig),
// }

// #[cfg(feature = "gui")]
// impl<SVO: EnumVariables, SVProp: EnumVariables> UIComponent for FaultModelConfig<SVO, SVProp> {
//     fn show_mut(
//         &mut self,
//         ui: &mut egui::Ui,
//         ctx: &egui::Context,
//         buffer_stack: &mut std::collections::BTreeMap<String, String>,
//         global_config: &SimulatorConfig,
//         current_node_name: Option<&String>,
//         unique_id: &str,
//     ) {
//         let self_str = self.to_string();
//         egui::CollapsingHeader::new(self_str)
//             .id_salt(format!("fault-model-{}", unique_id))
//             .show(ui, |ui| {
//                 match self {
//                     Self::Additive(cfg) => cfg.show_mut(
//                         ui,
//                         ctx,
//                         buffer_stack,
//                         global_config,
//                         current_node_name,
//                         unique_id,
//                     ),
//                     Self::Clutter(cfg) => cfg.show_mut(
//                         ui,
//                         ctx,
//                         buffer_stack,
//                         global_config,
//                         current_node_name,
//                         unique_id,
//                     ),
//                     Self::Misdetection(cfg) => cfg.show_mut(
//                         ui,
//                         ctx,
//                         buffer_stack,
//                         global_config,
//                         current_node_name,
//                         unique_id,
//                     ),
//                     Self::Misassociation(cfg) => cfg.show_mut(
//                         ui,
//                         ctx,
//                         buffer_stack,
//                         global_config,
//                         current_node_name,
//                         unique_id,
//                     ),
//                     Self::Python(cfg) => cfg.show_mut(
//                         ui,
//                         ctx,
//                         buffer_stack,
//                         global_config,
//                         current_node_name,
//                         unique_id,
//                     ),
//                     Self::External(cfg) => cfg.show_mut(
//                         ui,
//                         ctx,
//                         buffer_stack,
//                         global_config,
//                         current_node_name,
//                         unique_id,
//                     ),
//                 };
//             });
//     }

//     fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
//         let self_str = self.to_string();
//         egui::CollapsingHeader::new(self_str)
//             .id_salt(format!("fault-model-{}", unique_id))
//             .show(ui, |ui| {
//                 match self {
//                     Self::Additive(cfg) => cfg.show(ui, ctx, unique_id),
//                     Self::Clutter(cfg) => cfg.show(ui, ctx, unique_id),
//                     Self::Misdetection(cfg) => cfg.show(ui, ctx, unique_id),
//                     Self::Misassociation(cfg) => cfg.show(ui, ctx, unique_id),
//                     Self::Python(cfg) => cfg.show(ui, ctx, unique_id),
//                     Self::External(cfg) => cfg.show(ui, ctx, unique_id),
//                 };
//             });
//     }
// }

// #[cfg(feature = "gui")]
// impl<SVO: EnumVariables, SVProp: EnumVariables> FaultModelConfig<SVO, SVProp> {
//     pub fn show_faults_mut(
//         faults: &mut Vec<FaultModelConfig<SVO, SVProp>>,
//         ui: &mut egui::Ui,
//         ctx: &egui::Context,
//         buffer_stack: &mut std::collections::BTreeMap<String, String>,
//         global_config: &SimulatorConfig,
//         current_node_name: Option<&String>,
//         unique_id: &str,
//     ) {
//         use crate::{gui::utils::string_combobox, utils::enum_tools::ToVec};

//         ui.label("Faults:");
//         let mut fault_to_remove = None;
//         for (i, fault) in faults.iter_mut().enumerate() {
//             ui.horizontal_top(|ui| {
//                 let unique_fault_id = format!("fault-{i}-{unique_id}");
//                 fault.show_mut(
//                     ui,
//                     ctx,
//                     buffer_stack,
//                     global_config,
//                     current_node_name,
//                     &unique_fault_id,
//                 );

//                 if ui.button("X").clicked() {
//                     fault_to_remove = Some(i);
//                 }
//             });
//         }
//         if let Some(i) = fault_to_remove {
//             faults.remove(i);
//         }

//         ui.horizontal(|ui| {
//             let buffer_key = format!("selected-new-fault-{unique_id}");
//             if !buffer_stack.contains_key(&buffer_key) {
//                 buffer_stack.insert(buffer_key.clone(), "AdditiveRobotCentered".to_string());
//             }
//             string_combobox(
//                 ui,
//                 &FaultModelConfig::<SVO, SVProp>::to_vec(),
//                 buffer_stack.get_mut(&buffer_key).unwrap(),
//                 format!("fault-choice-{}", unique_id),
//             );
//             if ui.button("Add").clicked() {
//                 let selected_fault = buffer_stack.get(&buffer_key).unwrap();
//                 match selected_fault.as_str() {
//                     "Additive" => {
//                         faults.push(FaultModelConfig::Additive(
//                             AdditiveFaultConfig::default(),
//                         ))
//                     }
//                     "Clutter" => {
//                         faults.push(FaultModelConfig::Clutter(ClutterFaultConfig::default()))
//                     }
//                     "Misdetection" => faults.push(FaultModelConfig::Misdetection(
//                         MisdetectionFaultConfig::default(),
//                     )),
//                     "Misassociation" => faults.push(FaultModelConfig::Misassociation(
//                         MisassociationFaultConfig::default(),
//                     )),
//                     "External" => faults.push(FaultModelConfig::External(
//                         ExternalFaultConfig::default(),
//                     )),
//                     _ => panic!("Where did you find this fault?"),
//                 };
//             }
//         });
//     }

//     pub fn show_faults(
//         faults: &[FaultModelConfig<SVO, SVProp>],
//         ui: &mut egui::Ui,
//         ctx: &egui::Context,
//         unique_id: &str,
//     ) {
//         ui.label("Faults:");
//         for (i, fault) in faults.iter().enumerate() {
//             ui.horizontal_top(|ui| {
//                 let unique_fault_id = format!("fault-{i}-{unique_id}");
//                 fault.show(ui, ctx, &unique_fault_id);
//             });
//         }
//     }
// }

// #[derive(Debug, EnumToString)]
// pub enum FaultModelType<SVO: EnumVariables, SVProp: EnumVariables> {
//     Additive(AdditiveFault<SVO, SVProp>),
//     Clutter(ClutterFault<SVO>),
//     Misdetection(MisdetectionFault),
//     Misassociation(MisassociationFault),
//     Python(PythonFaultModel),
//     External(ExternalFault),
// }

// impl<SVO: EnumVariables, SVProp: EnumVariables> FaultModelType<SVO, SVProp> {
//     pub fn post_init(&mut self, node: &mut crate::node::Node, initial_time: f32) -> SimbaResult<()> {
//         match self {
//             Self::Python(f) => f.post_init(node, initial_time),
//             Self::External(f) => f.post_init(node, initial_time),
//             Self::Additive(_) | Self::Clutter(_) | Self::Misassociation(_) | Self::Misdetection(_) => Ok(()),
//         }
//     }
// }

// pub fn make_fault_model_from_config<SVO: EnumVariables, SVProp: EnumVariables>(
//     config: &FaultModelConfig<SVO, SVProp>,
//     plugin_api: &Option<Arc<dyn plugin_api::PluginAPI>>,
//     global_config: &SimulatorConfig,
//     node_name: &str,
//     va_factory: &Arc<DeterministRandomVariableFactory>,
//     initial_time: f32,
// ) -> SimbaResult<FaultModelType<SVO, SVProp>> {
//     Ok(match &config {
//         FaultModelConfig::Additive(cfg) => FaultModelType::Additive(
//             AdditiveFault::from_config(cfg, va_factory, initial_time),
//         ),
//         FaultModelConfig::Clutter(cfg) => FaultModelType::Clutter(
//             ClutterFault::from_config(cfg, va_factory, initial_time),
//         ),
//         FaultModelConfig::Misdetection(cfg) => FaultModelType::Misdetection(
//             MisdetectionFault::from_config(
//             cfg,
//             va_factory,
//             initial_time,
//         )),
//         FaultModelConfig::Misassociation(cfg) => FaultModelType::Misassociation(
//             MisassociationFault::from_config(
//             cfg,
//             global_config,
//             node_name,
//             va_factory,
//             initial_time,
//         )),
//         FaultModelConfig::Python(cfg) => FaultModelType::Python(
//             PythonFaultModel::from_config(cfg, global_config, initial_time)
//                 .expect("Failed to create Python Fault Model"),
//         ),
//         FaultModelConfig::External(cfg) => FaultModelType::External(
//             ExternalFault::from_config(
//                 cfg,
//                 plugin_api,
//                 global_config,
//                 va_factory,
//                 initial_time,
//             )
//             .expect("Failed to create External Fault"),
//         ),
//     })
// }

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
