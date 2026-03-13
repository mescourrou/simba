//! Error types and result aliases used across the crate.
//! 
//! This module defines the [`SimbaError`] struct, which contains a detailed error message and an error type 
//! from the [`SimbaErrorTypes`] enum. It also defines the [`SimbaResult`] type alias for results returned by 
//! Simba functions.
//! 
//! The error types are designed to be easily chained and to provide detailed information about the error chain 
//! of errors. They cover a wide range of error categories, from mathematical errors to network errors, and can 
//! be extended as needed.
use std::{
    error::Error,
    fmt::{Debug, Display},
};

use simba_macros::EnumToString;

use crate::networking::{NetworkError, service_manager::ServiceError};

/// Errors used in Simba.
/// 
/// They are designed to be easily chained and to provide detailed information about the error chain of errors.
#[derive(Debug, Clone, PartialEq, PartialOrd, EnumToString)]
pub enum SimbaErrorTypes {
    /// An error that does not fit in any other category.
    /// It should be used as a last resort, and the error message should be as detailed as possible to allow debugging.
    UnknownError,
    /// An error that occurs during a mathematical operation, such as a division by zero or a matrix inversion failure.
    MathError,
    /// An error that occurs following a wrong implementation: cases that should not happen.
    ImplementationError,
    /// An error that occurs during the loading of a configuration file or during the validation of a configuration.
    ConfigError,
    /// An error that occurs during the initialization of the simulator, such as a failure to initialize a module or to load a plugin.
    InitializationError,
    /// An error that occurs during the call to Python scripts, through the python API or when calling directly Python scripts.
    PythonError,
    /// An error that occurs during message passing between nodes, modules, etc.
    NetworkError(NetworkError),
    /// An error that occurs during the call to a service (request or response).
    ServiceError(ServiceError),
    /// An error that occurs during the call to the [`PluginAPI`](crate::plugin_api::PluginAPI).
    ExternalAPIError,
}

/// Error struct used in Simba. It contains the type of the error and a detailed message.
#[derive(Clone)]
pub struct SimbaError {
    error_type: SimbaErrorTypes,
    what: String,
}

impl SimbaError {
    /// Create a new SimbaError with the given type and message.
    pub fn new(error_type: SimbaErrorTypes, what: String) -> Self {
        Self { error_type, what }
    }

    /// Get a detailed error message, including the type of the error and the message.
    pub fn detailed_error(&self) -> String {
        format!("Simba Error of type {}: {}", self.error_type, self.what)
    }

    /// Get the type of the error.
    pub fn error_type(&self) -> SimbaErrorTypes {
        self.error_type.clone()
    }

    /// Chain a new error message to the current error, to provide more context about the error.
    pub fn chain(self, what: String) -> Self {
        Self {
            error_type: self.error_type,
            what: format!("{}\n↪ {}", self.what, what),
        }
    }
}

impl Display for SimbaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Simba Error: {}", self.error_type)
    }
}

impl Debug for SimbaError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "Simba Error of type {}: {}", self.error_type, self.what)
    }
}

impl Error for SimbaError {}

/// Type alias for results returned by Simba functions.
pub type SimbaResult<T> = Result<T, SimbaError>;
