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
    "Unit for the time analysis results, to make a more readable report."
    AnalysisUnit;
    "Documentation"
    Seconds, "s", "seconds";
    Milliseconds, "ms", "milliseconds";
    Microseconds, "us", "µs", "microseconds";
    Nanoseconds, "ns", "nanoseconds";
);

/// Configuration for the time analysis. It allows to select the exporter to use, the output path, and the analysis unit.
#[config_derives]
pub struct TimeAnalysisConfig {
    /// Exporter to use for the time analysis results export.
    #[check]
    pub exporter: ProfileExporterConfig,
    /// Output path for the time analysis results. Do not add extensions, as they are added by the exporter: `.json` (for [`ProfileExporterConfig::TraceEventExporter`]) and `.report.csv`.
    /// The path is relative to the config path
    pub output_path: String,
    /// Unit for the time analysis results, to make a more readable report.
    pub analysis_unit: AnalysisUnit,
}

impl Default for TimeAnalysisConfig {
    fn default() -> Self {
        TimeAnalysisConfig {
            exporter: ProfileExporterConfig::TraceEventExporter,
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
