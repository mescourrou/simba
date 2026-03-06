#[cfg(feature = "gui")]
use std::collections::BTreeMap;

use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::{
    UIComponent,
    utils::{enum_radio, path_finder, string_combobox},
};

use crate::time_analysis::ProfileExporterConfig;

#[config_derives]
pub struct TimeAnalysisConfig {
    #[check]
    pub exporter: ProfileExporterConfig,
    pub keep_last: bool,
    pub output_path: String,
    pub analysis_unit: String,
}

impl Check for TimeAnalysisConfig {
    fn do_check(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if !["s", "ms", "us", "µs", "ns"].contains(&self.analysis_unit.as_str()) {
            errors.push(format!(
                "Invalid analysis unit: {}. Must be one of s, ms, us, µs, ns.",
                self.analysis_unit
            ));
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

impl Default for TimeAnalysisConfig {
    fn default() -> Self {
        TimeAnalysisConfig {
            exporter: ProfileExporterConfig::TraceEventExporter,
            keep_last: true,
            output_path: "time_performance".to_string(),
            analysis_unit: "s".to_string(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for TimeAnalysisConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut BTreeMap<String, String>,
        global_config: &crate::simulator::SimulatorConfig,
        _current_node_name: Option<&String>,
        _unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Time Analysis").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Exporter:");
                enum_radio(ui, &mut self.exporter);
            });

            ui.horizontal(|ui| {
                ui.label("Output path: ");
                path_finder(ui, &mut self.output_path, &global_config.base_path);
            });

            ui.horizontal(|ui| {
                ui.label("Analysis unit:");
                string_combobox(
                    ui,
                    &vec!["s", "ms", "us", "ns"],
                    &mut self.analysis_unit,
                    "time-analysis-unit",
                );
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        egui::CollapsingHeader::new("Time Analysis").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!("Exporter: {}", self.exporter));
            });

            ui.horizontal(|ui| {
                ui.label(format!("Output path: {}", self.output_path));
            });

            ui.horizontal(|ui| {
                ui.label(format!("Analysis unit: {}", self.analysis_unit));
            });
        });
    }
}
