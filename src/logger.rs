use std::{
    fmt::Display,
    sync::{Arc, Mutex, RwLock},
};

use config_checker::macros::Check;
use serde::{Deserialize, Serialize};

static INTERNAL_LOG_LEVEL: RwLock<Vec<InternalLog>> = RwLock::new(Vec::new());

#[derive(Debug, Serialize, Deserialize, Check, Clone)]
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
