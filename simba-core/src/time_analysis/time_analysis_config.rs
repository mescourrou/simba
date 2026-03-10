#[cfg(feature = "gui")]
use std::collections::BTreeMap;

use simba_macros::{config_derives, enum_variables};

#[cfg(feature = "gui")]
use crate::gui::{
    UIComponent,
    utils::{enum_combobox, enum_radio, path_finder},
};

use crate::time_analysis::ProfileExporterConfig;

enum_variables!(
    AnalysisUnit;
    Seconds, "s", "seconds";
    Milliseconds, "ms", "milliseconds";
    Microseconds, "us", "µs", "microseconds";
    Nanoseconds, "ns", "nanoseconds";
);

#[config_derives]
pub struct TimeAnalysisConfig {
    #[check]
    pub exporter: ProfileExporterConfig,
    pub keep_last: bool,
    pub output_path: String,
    pub analysis_unit: AnalysisUnit,
}

impl Default for TimeAnalysisConfig {
    fn default() -> Self {
        TimeAnalysisConfig {
            exporter: ProfileExporterConfig::TraceEventExporter,
            keep_last: true,
            output_path: "time_performance".to_string(),
            analysis_unit: AnalysisUnit::Milliseconds,
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
                enum_combobox(ui, &mut self.analysis_unit, "time-analysis-unit");
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
