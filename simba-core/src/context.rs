//! Context module, which defines the [`Context`] struct that is passed to every function of the simulator, providing tools for logging and other contextual information.

use std::{collections::HashSet, sync::{Arc, RwLock}};

use colored::Colorize;

use crate::{logger::{InternalLog, LogLevel, LoggerConfig}, time_analysis, utils::SharedRwLock};

/// Context struct, which is passed to every function of the simulator.
/// 
/// Contains tools for logging.
#[derive(Clone, Debug)]
pub struct Context {
    /// The name of the node this context belongs to.
    node_name: String,
    current_sim_time: Option<SharedRwLock<f32>>,

    /* Logging part */
    /// Optionnal callstack of the current execution, used for logging and debugging purposes.
    callstack: Vec<String>,
    enabled_loglevel: SharedRwLock<LogLevel>,
    /// List of node names to include exclusively in logs.
    ///
    /// Empty by default, meaning no inclusion filter.
    pub included_nodes: SharedRwLock<HashSet<String>>,
    /// List of node names to exclude from logs.
    ///
    /// Empty by default, meaning no exclusion filter.
    pub excluded_nodes: SharedRwLock<HashSet<String>>,
}

impl Default for Context {
    fn default() -> Self {
        Self {
            node_name: "simulator".to_string(),
            callstack: Vec::new(),
            enabled_loglevel: Arc::new(RwLock::new(LogLevel::Info)),
            current_sim_time: None,
            included_nodes: Arc::new(RwLock::new(HashSet::new())),
            excluded_nodes: Arc::new(RwLock::new(HashSet::new())),
        }
    }
}

impl Context {
    /// Creates a new context for the given node name and log level.
    pub fn new(log_config: &LoggerConfig, time: Option<f32>) -> Self {
        Self {
            node_name: "simulator".to_string(),
            callstack: Vec::new(),
            enabled_loglevel: Arc::new(RwLock::new(log_config.log_level.clone())),
            current_sim_time: time.map(|x| Arc::new(RwLock::new(x))),
            included_nodes: Arc::new(RwLock::new(log_config.included_nodes.iter().map(|s| s.clone()).collect())),
            excluded_nodes: Arc::new(RwLock::new(log_config.excluded_nodes.iter().map(|s| s.clone()).collect())),
        }
    }

    /// Update the context's logging configuration based on the given `LoggerConfig` without loosing the current time or callstack information.
    pub fn update_config(&self, log_config: &LoggerConfig) {
        {
            let mut included_nodes = self.included_nodes.write().unwrap();
            let mut excluded_nodes = self.excluded_nodes.write().unwrap();
            *self.enabled_loglevel.write().unwrap() = log_config.log_level.clone();
            included_nodes.clear();
            included_nodes.extend(log_config.included_nodes.iter().map(|s| s.clone()));
            excluded_nodes.clear();
            excluded_nodes.extend(log_config.excluded_nodes.iter().map(|s| s.clone()));
        }
        info!(self, "Log level updated to {:?}", log_config.log_level);
    }

    /// Update the current simulation time in the context, which can be used for logging or other purposes.
    pub fn update_time(&self, time: f32) {
        if let Some(current_sim_time) = &self.current_sim_time {
            *current_sim_time.write().unwrap() = time;
        }
    }

    /// Get the current simulation time from the context.
    pub fn get_time(&self) -> Option<f32> {
        self.current_sim_time.as_ref().map(|x| *x.read().unwrap())
    }

    /// Creates a new context with the same properties as the current one, but with an additional call added to the callstack.
    /// This can be used to track the execution flow across different function calls and modules, providing more detailed logging information.
    pub fn new_callstack_level<S: ToString + ?Sized>(&self, call: &S) -> Self {
        let mut new_context = self.clone();
        new_context.callstack.push(call.to_string());
        new_context
    }

    /// Creates a new context with the same properties as the current one, but with the node name updated to the given one.
    pub fn new_node_context(&self, node_name: String) -> Self {
        let mut new_context = self.clone();
        new_context.node_name = node_name;
        new_context
    }
}

// Logging part
impl Context {
    #[inline]
    fn format_log_line(&self, message: &str, callstack: bool) -> String {
        let callstack_str = if callstack && !self.callstack.is_empty() {
            format!("[{}]", self.callstack.join("/").magenta())
        } else {
            String::new()
        };
        let time_str = if let Some(time) = self.get_time() {
            format!("[{:.3}]", time)
        } else {
            String::new()
        };
        format!("{}[{}]{} {}", time_str, self.node_name.cyan(), callstack_str, message)
    }

    /// Logs a message with the given log level, including the node name and callstack for context.
    /// The message will only be logged if the log level is enabled in the context's configuration
    pub fn log(&self, level: LogLevel, message: &str) {
        if self.excluded_nodes.read().unwrap().contains(&self.node_name) {
            return;
        }
        {
            let included_nodes = self.included_nodes.read().unwrap();
            if !included_nodes.is_empty() && !included_nodes.contains(&self.node_name) {
                return;
            }
        }
        let enabled_loglevel = self.enabled_loglevel.read().unwrap();
        if level > *enabled_loglevel {
            return;
        }
        match level {
            LogLevel::Off => (),
            LogLevel::Error => eprintln!("[{}]{}", "ERROR".red(), self.format_log_line(message, false)),
            LogLevel::Warn => eprintln!("[{}]{}", "WARN ".yellow(), self.format_log_line(message, false)),
            LogLevel::Info => println!("[{}]{}", "INFO ".green(), self.format_log_line(message, false)),
            LogLevel::Debug => println!("[{}]{}", "DEBUG".blue(), self.format_log_line(message, false)),
            LogLevel::Internal(l) => {
                if let LogLevel::Internal(enabled_list) = &*enabled_loglevel {
                    let matched_category = if enabled_list.contains(&InternalLog::All) {
                        Some(l.first().unwrap_or(&InternalLog::All).clone())
                    } else {
                        let mut matched_category = None;
                        for category in l {
                            if enabled_list.contains(&category) {
                                matched_category = Some(category);
                                break;
                            }
                        }
                        matched_category
                    };
                    if let Some(category) = matched_category {
                        println!("[{}][{}]{}", "INTNL".magenta(), category.to_string().magenta(), self.format_log_line(message, true));
                    }
                } else {
                    unreachable!()
                }
            }
        }
    }

    pub fn is_internal_log_level_enabled(&self, level: InternalLog) -> bool {
        let enabled_loglevel = self.enabled_loglevel.read().unwrap();
        if let LogLevel::Internal(enabled_list) = &*enabled_loglevel && (enabled_list.contains(&InternalLog::All) || enabled_list.contains(&level)) {
            true
        } else {
            false
        }
    }

    pub fn are_internal_log_level_enabled(&self, levels: &[InternalLog]) -> bool {
        let enabled_loglevel = self.enabled_loglevel.read().unwrap();
        if let LogLevel::Internal(enabled_list) = &*enabled_loglevel {
            enabled_list.contains(&InternalLog::All) || levels.iter().any(|level| enabled_list.contains(level))
        } else {
            false
        }
    }

    /// Log an error message with the "Error" log level.
    pub fn log_error(&self, message: &str) {
        self.log(LogLevel::Error, message);
    }

    /// Log a warning message with the "Warn" log level.
    pub fn log_warn(&self, message: &str) {
        self.log(LogLevel::Warn, message);
    }

    /// Log an informational message with the "Info" log level.
    pub fn log_info(&self, message: &str) {
        self.log(LogLevel::Info, message);
    }

    /// Log a debug message with the "Debug" log level (only for external logs, not internal category logs).
    pub fn log_debug(&self, message: &str) {
        self.log(LogLevel::Debug, message);
    }

    /// Log an internal message with the "Internal" log level and given categories (at least one should be enabled so that the message is logged).
    pub(crate) fn log_internal(&self, message: &str, category: Vec<InternalLog>) {
        self.log(LogLevel::Internal(category), message);
    }
}

#[macro_export]
macro_rules! error {
    ($context: expr, $($arg:tt)+) => ({
        $context.log_error(format!($($arg)+).as_str());
    });
}
pub use error;

#[macro_export]
macro_rules! warning {
    ($context: expr, $($arg:tt)+) => ({
        $context.log_warn(format!($($arg)+).as_str());
    });
}
pub use warning;

#[macro_export]
macro_rules! info {
    ($context: expr, $($arg:tt)+) => ({
        $context.log_info(format!($($arg)+).as_str());
    });
}
pub use info;

#[macro_export]
macro_rules! debug {
    ($context: expr, $($arg:tt)+) => ({
        $context.log_debug(format!($($arg)+).as_str());
    });
}
pub use debug;

#[macro_export]
macro_rules! internal {
    ($context: expr, $($category: expr)+, $($arg:tt)+) => ({
        $context.log_internal(format!($($arg)+).as_str(), vec![$($category),+]);
    });
}
pub use internal;