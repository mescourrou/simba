//! Logging configuration and internal debug-channel filtering.
//!
//! Simulator should not generate `debug` logs if [`InternalLog`] is not enabled.
//!
//! This module provides:
//! - [`LogLevel`] for global logging level selection,
//! - [`InternalLog`] for fine-grained internal debug categories,
//! - [`LoggerConfig`] to configure logging from simulator configuration,
//! - helper functions to initialize and query internal log flags.
use std::{fmt::Debug, sync::{Arc, RwLock}};

use simba_macros::{EnumToString, config_derives};

use crate::{networking::channels::internal::log, utils::SharedRwLock};
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

impl LogLevel {
    pub fn to_log_string(&self, colored: bool) -> String {
        if colored {
            match self {
                LogLevel::Off => String::new(),
                LogLevel::Error => "ERROR".red().to_string(),
                LogLevel::Warn => "WARN ".yellow().to_string(),
                LogLevel::Info => "INFO ".green().to_string(),
                LogLevel::Debug => "DEBUG".blue().to_string(),
                LogLevel::Internal(_) => "INTNL".magenta().to_string(),
            }
        } else {
            match self {
                LogLevel::Off => String::new(),
                LogLevel::Error => "ERROR".to_string(),
                LogLevel::Warn => "WARN ".to_string(),
                LogLevel::Info => "INFO ".to_string(),
                LogLevel::Debug => "DEBUG".to_string(),
                LogLevel::Internal(_) => "INTNL".to_string(),
            }
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

    pub outputs: Vec<LoggerTypeConfig>,
}

impl Default for LoggerConfig {
    fn default() -> Self {
        Self {
            included_nodes: Vec::new(),
            excluded_nodes: Vec::new(),
            log_level: LogLevel::Info,
            outputs: vec![LoggerTypeConfig::Console],
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


/// Trait for custom log targets, allowing users to implement their own logging mechanisms (e.g., file logging, remote logging, etc.).
pub trait Logger : Debug + Send + Sync {
    /// Log a message with the given log level. The implementation of this method will determine how the log message is handled (e.g., printed to console, written to a file, sent to a remote server, etc.).
    /// 
    /// # Parameters
    /// * `loglevel` - The severity of the log message, which can be used to dispatch messages based on the configured log level.
    /// * `time` - An optional timestamp for the log message.
    /// * `node_name` - The name of the node that generated the log message.
    /// * `callstack` - An optional list of function names representing the call stack at the time the log message was generated (typically used for internal messages).
    /// * `internal_category` - An optional category for internal log messages.
    /// * `message` - The content of the log message to be logged
    fn log(&mut self, loglevel: &LogLevel, time: Option<f32>, node_name: &String, callstack: &Vec<String>, internal_category: Option<&InternalLog>, message: &str);
}

/// Configuration for different types of log targets, allowing users to specify where log messages should be sent (e.g., console, file, etc.) when configuring the logger.
#[config_derives(tag_content)]
pub enum LoggerTypeConfig {
    /// Output log messages to the console (standard output and standard error).
    Console,
    /// Write log messages to a file at a specified path. The `value` field should contain the file path where log messages will be written.
    File(String),
}

/// Enum to represent different types of log targets, allowing for flexible logging configurations.
#[derive(Debug, EnumToString)]
pub enum LoggerType {
    /// Output log messages to the console (standard output and standard error).
    Console(ConsoleLogger),
    /// Accumulate log messages in a string, which can be retrieved later. Useful for testing or displaying logs in a GUI.
    String(StringLogger),
    /// Write log messages to a file at a specified path. 
    File(FileLogger),
    /// Custom log target defined by the user, which can implement any logging mechanism (e.g., file logging, remote logging, etc.) by implementing the [`Logger`] trait.
    Custom(Box<dyn Logger>),
}

impl LoggerType {
    /// Log a message using the selected log target. The log message will be dispatched to the appropriate logging mechanism based on the variant of `LoggerType`.
    pub fn log(&mut self, loglevel: &LogLevel, time: Option<f32>, node_name: &String, callstack: &Vec<String>, internal_category: Option<&InternalLog>, message: &str) {
        match self {
            LoggerType::Console(logger) => logger.log(loglevel, time, node_name, callstack, internal_category, message),
            LoggerType::String(logger) => logger.log(loglevel, time, node_name, callstack, internal_category, message),
            LoggerType::File(logger) => logger.log(loglevel, time, node_name, callstack, internal_category, message),
            LoggerType::Custom(logger) => logger.log(loglevel, time, node_name, callstack, internal_category, message),
        }
    }
}

/// Format the log message with the appropriate log level, timestamp, node name, call stack, internal category, and message content. The formatting can be customized to include colors for better readability in the console.
/// 
/// The output is structured as follows:
/// ```text
/// [LOG_LEVEL][TIMESTAMP?][NODE_NAME][CALLSTACK?][INTERNAL_CATEGORY?] MESSAGE
/// ```
/// - `LOG_LEVEL`: The severity of the log message (e.g., ERROR, WARN, INFO, DEBUG, INTNL).
/// - `TIMESTAMP`: An optional timestamp for when the log message was generated, formatted to three decimal places.
/// - `NODE_NAME`: The name of the node that generated the log message, optionally colored for better visibility.
/// - `CALLSTACK`: An optional list of function names representing the call stack at the time the log message was generated, formatted as a single string and optionally colored. This is enabled only for internal messages.
/// - `INTERNAL_CATEGORY`: An optional category for internal log messages, formatted as a string and optionally colored.
/// - `MESSAGE`: The content of the log message to be logged.
pub fn format_log_message(loglevel: &LogLevel, time: Option<f32>, node_name: &String, callstack: &Vec<String>, internal_category: Option<&InternalLog>, message: &str, colored: bool) -> String {
    let callstack_str = if let LogLevel::Internal(_) = loglevel && !callstack.is_empty() {
        format!("[{}]", if colored {
            callstack.join("/").magenta().to_string()
        } else {
            callstack.join("/")
        })
    } else {
        String::new()
    };
    let internal_str = if let Some(internal) = internal_category {
        format!("[{}]", if colored {
            internal.to_string().magenta().to_string()
        } else {
            internal.to_string()
        })
    } else {
        String::new()
    };
    let time_str = if let Some(time) = time {
        format!("[{:.3}]", time)
    } else {
        String::new()
    };
    format!(
        "[{}]{}[{}]{}{} {}",
        loglevel.to_log_string(colored),
        time_str,
        if colored {
            &node_name.cyan().to_string()
        } else {
            node_name
        },
        callstack_str,
        internal_str,
        message
    )
}

use colored::Colorize;
/// Logger implementation that writes log messages to the console (standard output and standard error). Log messages with a log level of `Warn` or higher are written to standard error, while messages with a log level of `Info` or lower are written to standard output.
#[derive(Debug, Clone)]
pub struct ConsoleLogger {
    /// Whether to use colored output for log messages. When enabled, log levels and other components of the log message will be colored for better readability in the console.
    /// 
    /// Default: `true`.
    pub colored: bool,
}

impl Default for ConsoleLogger {
    fn default() -> Self {
        ConsoleLogger { colored: true }
    }
    
}


impl ConsoleLogger {
    /// Create a new `ConsoleLogger` instance. This logger does not require any configuration and will write log messages to the console based on their log level.
    pub fn new() -> Self {
        ConsoleLogger::default()
    }
}

impl Logger for ConsoleLogger {
    #[inline]
    fn log(&mut self, loglevel: &LogLevel, time: Option<f32>, node_name: &String, callstack: &Vec<String>, internal_category: Option<&InternalLog>, message: &str) {
        let message = format_log_message(loglevel, time, node_name, callstack, internal_category, message, self.colored);
        if *loglevel >= LogLevel::Warn {
            eprintln!("{}", message);
        } else {
            println!("{}", message);
        }
    }
}

/// Logger implementation that accumulates log messages in a string. This logger is useful for testing purposes, allowing retrieval of all logged messages as a single string.
#[derive(Debug, Clone)]
pub struct StringLogger {
    logs: SharedRwLock<String>,
    /// Whether to use colored output for log messages. When enabled, log levels and other components of the log message will be colored for better readability when retrieved as a string.
    /// 
    /// Default: `false`.
    pub colored: bool,
}

impl StringLogger {
    /// Create a new `StringLogger` instance with an empty log string. This logger will accumulate log messages in memory and allow retrieval of the entire log history as a single string.
    pub fn new() -> Self {
        StringLogger {
            logs: Arc::new(RwLock::new(String::new())),
            colored: false,
        }
    }

    /// Create a new `StringLogger` instance that shares the same log string with other owners. This allows `StringLogger` to write in a String displayed in the GUI, for example.
    pub fn from_string(logs: SharedRwLock<String>) -> Self {
        StringLogger {
            logs,
            colored: false,
        }
    }

    /// Retrieve the accumulated log messages as a single string.
    pub fn get_logs(&self) -> String {
        self.logs.read().unwrap().clone()
    }
}

impl Logger for StringLogger {
    fn log(&mut self, loglevel: &LogLevel, time: Option<f32>, node_name: &String, callstack: &Vec<String>, internal_category: Option<&InternalLog>, message: &str) {
        let mut logs = self.logs.write().unwrap();
        let message = format_log_message(loglevel, time, node_name, callstack, internal_category, message, self.colored);
        logs.push_str(&format!("{}\n", message));
    }
}

/// Logger implementation that writes log messages to a file. The file is created at the specified path when the logger is initialized, and log messages are appended to the file as they are logged.
#[derive(Debug)]
pub struct FileLogger {
    file: std::fs::File,
    /// Whether to use colored output for log messages. When enabled, log levels and other components of the log message will be colored for better readability when viewed in the file (note that this may not display correctly in all text editors).
    /// 
    /// Default: `false`.
    pub colored: bool,
}

impl FileLogger {
    /// Create a new `FileLogger` that writes log messages to the specified file path. If the file does not exist, it will be created. If the file already exists, it will be overwritten.
    pub fn new(filepath: String) -> Self {
        let file = std::fs::File::create(&filepath).expect("Unable to create log file");
        FileLogger { file, colored: false }
    }
}

impl Logger for FileLogger {
    fn log(&mut self, loglevel: &LogLevel, time: Option<f32>, node_name: &String, callstack: &Vec<String>, internal_category: Option<&InternalLog>, message: &str) {
        use std::io::Write;
        let message = format_log_message(loglevel, time, node_name, callstack, internal_category, message, self.colored);
        writeln!(self.file, "{}", message).expect("Unable to write to log file");
    }
}
