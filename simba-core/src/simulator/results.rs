#[cfg(feature = "gui")]
use egui::{CollapsingHeader, DragValue};
#[cfg(feature = "gui")]
use std::collections::BTreeMap;

use config_checker::macros::Check;
use serde::{Deserialize, Serialize};
use serde_json::Value;
#[cfg(feature = "gui")]
use simba_macros::EnumToString;

#[cfg(feature = "gui")]
use crate::{
    constants::TIME_ROUND,
    gui::{
        utils::{json_config, path_finder, string_combobox},
        UIComponent,
    },
};

use crate::simulator::{Record, SimulatorConfig};

use crate::utils::enum_tools::ToVec;

#[derive(Serialize, Deserialize, Debug, Clone, Default)]
#[cfg_attr(feature = "gui", derive(EnumToString))]
#[serde(deny_unknown_fields)]
pub enum ResultSaveMode {
    #[default]
    AtTheEnd,
    Continuous,
    Batch(usize),
    Periodic(f32),
}

impl ToVec<&'static str> for ResultSaveMode {
    fn to_vec() -> Vec<&'static str> {
        vec!["AtTheEnd", "Continuous", "Batch", "Periodic"]
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct ResultConfig {
    /// Filename to save the results, in JSON format. The directory of this
    /// file is used to save the figures if results are computed.
    pub result_path: Option<String>,
    /// Show the matplotlib figures
    pub show_figures: bool,
    /// Path to the python analyse scrit.
    /// This script should have the following entry point:
    /// ```def analyse(result_data: Record, figure_path: str, figure_type: str)```
    /// If the option is none, the script is not run
    pub analyse_script: Option<String>,
    pub figures_path: Option<String>,
    pub python_params: Value,
    pub save_mode: ResultSaveMode,
}

impl Default for ResultConfig {
    /// Default scenario configuration: no nodes.
    fn default() -> Self {
        Self {
            result_path: None,
            show_figures: false,
            analyse_script: None,
            figures_path: None,
            python_params: Value::Null,
            save_mode: ResultSaveMode::default(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ResultConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        buffer_stack: &mut BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        let python_param_key = format!("result-config-python-params-{}", unique_id);
        let python_param_error_key = format!("result-config-python-params-error-{}", unique_id);
        CollapsingHeader::new("Results").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Result Path:");
                if let Some(path) = &mut self.result_path {
                    path_finder(ui, path, &global_config.base_path);
                    if ui.button("X").clicked() {
                        self.result_path = None;
                    }
                } else if ui.button("+").clicked() {
                    self.result_path = Some(String::from(""));
                }
            });

            let mut current_str = self.save_mode.to_string();
            ui.horizontal(|ui| {
                ui.label("Save mode:");
                string_combobox(
                    ui,
                    &ResultSaveMode::to_vec()
                        .iter()
                        .map(|x| String::from(*x))
                        .collect(),
                    &mut current_str,
                    format!("result-save-mode-choice-{}", unique_id),
                );
                if let ResultSaveMode::Batch(size) = &mut self.save_mode {
                    ui.add(DragValue::new(size).max_decimals(0));
                    if *size < 1 {
                        *size = 1;
                    }
                } else if let ResultSaveMode::Periodic(period) = &mut self.save_mode {
                    ui.add(DragValue::new(period));
                    if *period <= TIME_ROUND {
                        *period = TIME_ROUND;
                    }
                }
            });
            if current_str != self.save_mode.to_string() {
                match current_str.as_str() {
                    "AtTheEnd" => self.save_mode = ResultSaveMode::AtTheEnd,
                    "Continuous" => self.save_mode = ResultSaveMode::Continuous,
                    "Batch" => self.save_mode = ResultSaveMode::Batch(10),
                    "Periodic" => self.save_mode = ResultSaveMode::Periodic(5.),
                    _ => panic!("Where did you find this value?"),
                };
            }

            ui.horizontal(|ui| {
                ui.label("Show figures:");
                ui.checkbox(&mut self.show_figures, "");
            });

            ui.horizontal(|ui| {
                ui.label("Analyse script path:");
                if let Some(script) = &mut self.analyse_script {
                    path_finder(ui, script, &global_config.base_path);
                    if ui.button("X").clicked() {
                        self.analyse_script = None;
                    }
                } else if ui.button("+").clicked() {
                    self.analyse_script = Some(String::from(""));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Figure Path:");
                if let Some(fig) = &mut self.figures_path {
                    path_finder(ui, fig, &global_config.base_path);
                    if ui.button("X").clicked() {
                        self.figures_path = None;
                    }
                } else if ui.button("+").clicked() {
                    self.figures_path = Some(String::from(""));
                }
            });
            ui.vertical(|ui| {
                ui.label("Python params (JSON format, null for nothing):");
                json_config(
                    ui,
                    &python_param_key,
                    &python_param_error_key,
                    buffer_stack,
                    &mut self.python_params,
                );
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        CollapsingHeader::new("Results").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Result Path: ");
                if let Some(path) = &self.figures_path {
                    ui.label(path);
                } else {
                    ui.label("None");
                }
            });

            ui.horizontal(|ui| {
                let mut as_str = self.save_mode.to_string();
                if let ResultSaveMode::Batch(s) = &self.save_mode {
                    as_str = format!("{} ({})", as_str, s);
                } else if let ResultSaveMode::Periodic(p) = &self.save_mode {
                    as_str = format!("{} ({} s)", as_str, p);
                }
                ui.label(format!("Save mode: {}", as_str));
            });

            ui.horizontal(|ui| {
                ui.label("Show figures: ");
                if self.show_figures {
                    ui.label("Yes");
                } else {
                    ui.label("No");
                }
            });

            ui.horizontal(|ui| {
                ui.label("Result Path: ");
                if let Some(script) = &self.analyse_script {
                    ui.label(script);
                } else {
                    ui.label("None");
                }
            });
            ui.horizontal(|ui| {
                ui.label("Figure Path: ");
                if let Some(path) = &self.figures_path {
                    ui.label(path);
                } else {
                    ui.label("None");
                }
            });
            ui.vertical(|ui| {
                ui.label("Python params: ");
                ui.label(self.python_params.to_string());
            });
        });
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Results {
    pub config: SimulatorConfig,
    pub records: Vec<Record>,
}

#[derive(Clone)]
pub(super) struct ResultSavingData {
    pub save_mode: ResultSaveMode,
    pub first_row: bool,
}

impl Default for ResultSavingData {
    fn default() -> Self {
        Self {
            save_mode: ResultSaveMode::default(),
            first_row: true,
        }
    }
}
