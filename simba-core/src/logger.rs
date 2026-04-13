//! Logging configuration and internal debug-channel filtering.
//!
//! Simulator should not generate `debug` logs if [`InternalLog`] is not enabled.
//!
//! This module provides:
//! - [`LogLevel`] for global logging level selection,
//! - [`InternalLog`] for fine-grained internal debug categories,
//! - [`LoggerConfig`] to configure logging from simulator configuration,
//! - helper functions to initialize and query internal log flags.
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::{
    gui::{
        UIComponent,
        utils::{enum_checkbox, enum_radio, string_checkbox},
    },
    simulator::SimulatorConfig,
};

/// Global logging level configuration.
#[config_derives(tag_content)]
pub enum LogLevel {
    /// Disable all logs.
    Off,
    /// Enable only error logs.
    Error,
    /// Enable warning and error logs.
    Warn,
    /// Enable info, warning, and error logs.
    Info,
    /// Enable debug, info, warning, and error logs.
    Debug,
    /// Enable debug logs with additional internal category filtering.
    Internal(Vec<InternalLog>),
}

impl From<LogLevel> for String {
    fn from(level: LogLevel) -> Self {
        match level {
            LogLevel::Off => "Off".to_string(),
            LogLevel::Error => "Error".to_string(),
            LogLevel::Warn => "Warn".to_string(),
            LogLevel::Info => "Info".to_string(),
            LogLevel::Debug => "Debug".to_string(),
            LogLevel::Internal(internals) => {
                let internals_string = internals
                    .iter()
                    .map(|il| il.to_string())
                    .collect::<Vec<String>>()
                    .join(", ");
                format!("Internal [{}]", internals_string)
            }
        }
    }
}

impl PartialOrd for LogLevel {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        let self_value = match self {
            LogLevel::Off => 0,
            LogLevel::Error => 1,
            LogLevel::Warn => 2,
            LogLevel::Info => 3,
            LogLevel::Debug => 4,
            LogLevel::Internal(_) => 5,
        };
        let other_value = match other {
            LogLevel::Off => 0,
            LogLevel::Error => 1,
            LogLevel::Warn => 2,
            LogLevel::Info => 3,
            LogLevel::Debug => 4,
            LogLevel::Internal(_) => 5,
        };
        self_value.partial_cmp(&other_value)
    }
}

/// Internal debug categories used when [`LogLevel::Internal`] is selected.
#[config_derives]
pub enum InternalLog {
    /// Enable all internal categories.
    All,
    /// Network message lifecycle logs.
    NetworkMessages,
    /// Detailed network message logs.
    NetworkMessagesDetailed,
    /// Setup phase summary logs.
    SetupSteps,
    /// Detailed setup phase logs.
    SetupStepsDetailed,
    /// Sensor manager summary logs.
    SensorManager,
    /// Detailed sensor manager logs.
    SensorManagerDetailed,
    /// Node execution summary logs.
    NodeRunning,
    /// Detailed node execution logs.
    NodeRunningDetailed,
    /// Detailed node synchronization logs.
    NodeSyncDetailed,
    /// API bridge logs (Python and PluginAPI).
    API,
    /// Detailed navigator computation logs.
    NavigatorDetailed,
    /// Scenario loading and update logs.
    Scenario,
    /// Environment summary logs.
    Environment,
    /// Detailed environment logs.
    EnvironmentDetailed,
}

/// Logger configuration applied at simulator startup.
#[config_derives]
pub struct LoggerConfig {
    /// List of node names to include exclusively in logs.
    ///
    /// Empty by default, meaning no inclusion filter.
    pub included_nodes: Vec<String>,
    /// List of node names to exclude from logs.
    ///
    /// Empty by default, meaning no exclusion filter.
    pub excluded_nodes: Vec<String>,
    /// Selected global log level.
    ///
    /// Default: [`LogLevel::Info`].
    pub log_level: LogLevel,
}

impl Default for LoggerConfig {
    fn default() -> Self {
        Self {
            included_nodes: Vec::new(),
            excluded_nodes: Vec::new(),
            log_level: LogLevel::Info,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for LoggerConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        _unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Logger").show(ui, |ui| {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    ui.label("Log level:");
                    enum_radio(ui, &mut self.log_level);
                    let mut internal = false;
                    if let LogLevel::Internal(_) = &self.log_level {
                        internal = true;
                    }
                    if ui.radio(internal, "Internal").clicked() && !internal {
                        self.log_level = LogLevel::Internal(Vec::new());
                    }
                });

                ui.horizontal_wrapped(|ui| {
                    if let LogLevel::Internal(l) = &mut self.log_level {
                        enum_checkbox(ui, l);
                    }
                });
            });

            let mut node_list = Vec::from_iter(
                global_config.robots.iter().map(|x| x.name.clone()).chain(
                    global_config
                        .computation_units
                        .iter()
                        .map(|x| x.name.clone()),
                ),
            );
            node_list.push("simulator".to_string());
            ui.horizontal_wrapped(|ui| {
                ui.label("Include only:");
                string_checkbox(ui, &node_list, &mut self.included_nodes);
            });

            ui.horizontal_wrapped(|ui| {
                ui.label("Exclude:");
                string_checkbox(ui, &node_list, &mut self.excluded_nodes);
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        egui::CollapsingHeader::new("Logger").show(ui, |ui| {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    ui.label(format!("Log level: {}", self.log_level));
                    if let LogLevel::Internal(v) = &self.log_level {
                        ui.label("(");
                        for iv in v {
                            ui.label(format!("{}, ", iv));
                        }
                        ui.label(")");
                    }
                });
            });

            ui.horizontal_wrapped(|ui| {
                ui.label("Include only:");
                for n in &self.included_nodes {
                    ui.label(format!("{n}, "));
                }
            });

            ui.horizontal_wrapped(|ui| {
                ui.label("Exclude:");
                for n in &self.excluded_nodes {
                    ui.label(format!("{n}, "));
                }
            });
        });
    }
}
