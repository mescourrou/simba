use std::{
    fmt::Display,
    ops::DerefMut,
    sync::{Arc, Mutex, RwLock},
};

use config_checker::macros::Check;
use libm::tanhf;
use serde::{Deserialize, Serialize};
use simba_macros::ToVec;

#[cfg(feature = "gui")]
use crate::gui::{
    utils::{enum_checkbox, enum_radio, string_checkbox},
    UIComponent,
};
use crate::{simulator::SimulatorConfig, utils::enum_tools::ToVec};

static INTERNAL_LOG_LEVEL: RwLock<Vec<InternalLog>> = RwLock::new(Vec::new());

#[derive(Debug, Serialize, Deserialize, Check, Clone, PartialEq)]
#[cfg_attr(feature = "gui", derive(ToVec))]
pub enum LogLevel {
    Off,
    Error,
    Warn,
    Info,
    Debug,
    Internal(Vec<InternalLog>),
}

impl Into<log::LevelFilter> for LogLevel {
    fn into(self) -> log::LevelFilter {
        match self {
            LogLevel::Off => log::LevelFilter::Off,
            LogLevel::Error => log::LevelFilter::Error,
            LogLevel::Warn => log::LevelFilter::Warn,
            LogLevel::Info => log::LevelFilter::Info,
            LogLevel::Debug | LogLevel::Internal(_) => log::LevelFilter::Debug,
        }
    }
}

impl Into<&str> for LogLevel {
    fn into(self) -> &'static str {
        match self {
            LogLevel::Off => "Off",
            LogLevel::Error => "Error",
            LogLevel::Warn => "Warn",
            LogLevel::Info => "Info",
            LogLevel::Debug => "Debug",
            LogLevel::Internal(_) => "Internal",
        }
    }
}

impl Display for LogLevel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", Into::<&str>::into(self.clone()))
    }
}

#[derive(Debug, Serialize, Deserialize, Check, Clone, PartialEq)]
#[cfg_attr(feature = "gui", derive(ToVec))]
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
}

#[derive(Debug, Serialize, Deserialize, Check, Clone)]
#[serde(default)]
#[serde(deny_unknown_fields)]
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
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::HashMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
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
                    if ui.radio(internal, "Internal").clicked() {
                        if !internal {
                            self.log_level = LogLevel::Internal(Vec::new());
                        }
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
