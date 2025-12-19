use std::{
    error::Error,
    fmt::{Debug, Display},
};

use simba_macros::EnumToString;

use crate::networking::{NetworkError, service_manager::ServiceError};

#[derive(Debug, Clone, PartialEq, PartialOrd, EnumToString)]
pub enum SimbaErrorTypes {
    UnknownError,
    MathError,
    ImplementationError,
    ConfigError,
    InitializationError,
    PythonError,
    NetworkError(NetworkError),
    ServiceError(ServiceError),
}

#[derive(Clone)]
pub struct SimbaError {
    error_type: SimbaErrorTypes,
    what: String,
}

impl SimbaError {
    pub fn new(error_type: SimbaErrorTypes, what: String) -> Self {
        Self { error_type, what }
    }

    pub fn detailed_error(&self) -> String {
        format!("Simba Error of type {}: {}", self.error_type, self.what)
    }

    pub fn error_type(&self) -> SimbaErrorTypes {
        self.error_type.clone()
    }

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

pub type SimbaResult<T> = Result<T, SimbaError>;
