use std::sync::RwLock;

use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::{
    gui::{
        UIComponent,
        utils::{enum_checkbox, enum_radio, string_checkbox},
    },
    simulator::SimulatorConfig,
};

static INTERNAL_LOG_LEVEL: RwLock<Vec<InternalLog>> = RwLock::new(Vec::new());

#[config_derives(tag_content)]
pub enum LogLevel {
    Off,
    Error,
    Warn,
    Info,
    Debug,
    Internal(Vec<InternalLog>),
}

impl From<log::LevelFilter> for LogLevel {
    fn from(level: log::LevelFilter) -> Self {
        match level {
            log::LevelFilter::Off => LogLevel::Off,
            log::LevelFilter::Error => LogLevel::Error,
            log::LevelFilter::Warn => LogLevel::Warn,
            log::LevelFilter::Info => LogLevel::Info,
            log::LevelFilter::Debug => LogLevel::Debug,
            log::LevelFilter::Trace => LogLevel::Debug,
        }
    }
}

impl From<LogLevel> for log::LevelFilter {
    fn from(level: LogLevel) -> Self {
        match level {
            LogLevel::Off => log::LevelFilter::Off,
            LogLevel::Error => log::LevelFilter::Error,
            LogLevel::Warn => log::LevelFilter::Warn,
            LogLevel::Info => log::LevelFilter::Info,
            LogLevel::Debug => log::LevelFilter::Debug,
            LogLevel::Internal(_) => log::LevelFilter::Debug,
        }
    }
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

#[config_derives]
pub enum InternalLog {
    All,
    NetworkMessages,
    ServiceHandling,
    SetupSteps,
    SetupStepsDetailed,
    SensorManager,
    SensorManagerDetailed,
    NodeRunning,
    NodeRunningDetailed,
    NodeSyncDetailed,
    API,
    NavigatorDetailed,
    Scenario,
}

#[config_derives]
pub struct LoggerConfig {
    pub included_nodes: Vec<String>,
    pub excluded_nodes: Vec<String>,
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

pub fn init_log(config: &LoggerConfig) {
    if let LogLevel::Internal(v) = &config.log_level {
        *INTERNAL_LOG_LEVEL.write().unwrap() = v.clone();
    }
}

pub fn is_enabled(internal_level: InternalLog) -> bool {
    if let InternalLog::All = internal_level {
        return true;
    }
    INTERNAL_LOG_LEVEL
        .read()
        .unwrap()
        .contains(&InternalLog::All)
        || INTERNAL_LOG_LEVEL.read().unwrap().contains(&internal_level)
}
