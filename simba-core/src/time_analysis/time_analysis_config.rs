#[cfg(feature = "gui")]
use std::collections::BTreeMap;

use config_checker::macros::Check;
use serde::{Deserialize, Serialize};
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::{
    utils::{enum_radio, path_finder, string_combobox},
    UIComponent,
};

use crate::time_analysis::ProfileExporterConfig;

#[config_derives]
pub struct TimeAnalysisConfig {
    pub exporter: ProfileExporterConfig,
    pub keep_last: bool,
    pub output_path: String,
    #[convert(as_str)]
    #[check(inside("s", "ms", "us", "µs", "ns"))]
    pub analysis_unit: String,
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
                    &vec![
                        "s".to_string(),
                        "ms".to_string(),
                        "us".to_string(),
                        "ns".to_string(),
                    ],
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
