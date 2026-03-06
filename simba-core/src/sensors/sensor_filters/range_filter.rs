use std::{collections::HashMap, usize};

use log::debug;
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::{
    UIComponent,
    utils::{enum_combobox, string_combobox},
};
use crate::utils::enum_tools::EnumVariables;

#[config_derives]
pub struct RangeFilterConfig<SV: EnumVariables> {
    pub variables: Vec<SV>,
    pub min_range: Vec<f32>,
    pub max_range: Vec<f32>,
    pub inside: bool,
}

impl<SV: EnumVariables> Check for RangeFilterConfig<SV> {
    fn do_check(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.variables.len() != self.min_range.len()
            || self.variables.len() != self.max_range.len()
        {
            errors.push(format!(
                "variables, min_range and max_range should have the same length, got {} variables, {} min_range and {} max_range",
                self.variables.len(),
                self.min_range.len(),
                self.max_range.len()
            ));
        }
        let possible_variables = SV::to_vec();
        for var in &self.variables {
            if !possible_variables.contains(&var.to_string().as_str()) {
                errors.push(format!(
                    "Unknown variable name: '{}'. Available variable names: [{}]",
                    var,
                    possible_variables.join(", ")
                ));
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

impl<SV: EnumVariables> Default for RangeFilterConfig<SV> {
    fn default() -> Self {
        Self {
            variables: Vec::new(),
            min_range: Vec::new(),
            max_range: Vec::new(),
            inside: true,
        }
    }
}

#[cfg(feature = "gui")]
impl<SV: EnumVariables> UIComponent for RangeFilterConfig<SV> {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &crate::simulator::SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        ui.vertical(|ui| {
            let possible_variables: Vec<SV> = SV::to_vec();
            ui.horizontal(|ui| {
                ui.label("Variable order:");
                for (i, var) in self.variables.iter_mut().enumerate() {
                    let unique_var_id = format!("variable-{i}-{unique_id}");
                    enum_combobox(ui, var, unique_var_id);
                }
                if !self.variables.is_empty() && ui.button("-").clicked() {
                    self.variables.pop();
                    self.max_range.pop();
                    self.min_range.pop();
                }
                if ui.button("+").clicked() {
                    self.variables.push(
                        possible_variables
                            .get(self.variables.len().min(possible_variables.len()))
                            .unwrap()
                            .clone(),
                    );
                    self.min_range.push(0.);
                    self.max_range.push(1.);
                }
            });

            for (i, variable) in self.variables.iter().enumerate() {
                ui.horizontal(|ui| {
                    ui.label(format!("Range for {}:", variable));
                    ui.add(egui::DragValue::new(&mut self.min_range[i]));
                    ui.label("-");
                    ui.add(egui::DragValue::new(&mut self.max_range[i]));
                });
            }

            ui.horizontal(|ui| {
                ui.label("Inside range:");
                ui.checkbox(&mut self.inside, "");
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label("Variable order:");
            for (i, var) in self.variables.iter().enumerate() {
                ui.label(format!(
                    "{}: [{} - {}]",
                    var, self.min_range[i], self.max_range[i]
                ));
            }
            ui.label(format!("Inside range: {}", self.inside));
        });
    }
}

#[derive(Debug, Clone)]
pub struct RangeFilter<SV: EnumVariables> {
    config: RangeFilterConfig<SV>,
}

impl<SV: EnumVariables> RangeFilter<SV> {
    pub fn from_config(config: &RangeFilterConfig<SV>, _initial_time: f32) -> Self {
        Self {
            config: config.clone(),
        }
    }

    pub fn match_exclusion(&self, variable_map: &HashMap<SV, f32>) -> bool {
        let mut keep = true;
        for (i, var) in self.config.variables.iter().enumerate() {
            if let Some(value) = variable_map.get(var) {
                let in_range =
                    *value >= self.config.min_range[i] && *value <= self.config.max_range[i];
                if self.config.inside {
                    if !in_range {
                        keep = false;
                        break;
                    }
                } else if in_range {
                    keep = false;
                    break;
                }
            } else {
                panic!(
                    "Variable '{}' not found in observation variables ([{}])! Variables that should be availables: [{}]",
                    var,
                    variable_map
                        .keys()
                        .into_iter()
                        .map(|v| v.to_string())
                        .collect::<Vec<_>>()
                        .join(", "),
                    (SV::to_vec() as Vec<&str>).join(", ")
                );
            }
        }
        keep
    }
}
