use std::{
    error::Error,
    fmt::{Debug, Display},
};

use simba_macros::EnumToString;

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, EnumToString)]
pub enum SimbaErrorTypes {
    UnknwonError,
    MathError,
    ImplementationError,
    ConfigError,
    InitializationError,
    PythonError,
    NetworkError,
}

#[derive(Clone)]
pub struct SimbaError {
    error_type: SimbaErrorTypes,
    what: String,
}

impl SimbaError {
    pub fn new(error_type: SimbaErrorTypes, what: String) -> Self {
        log::error!("{}: {what}", error_type.to_string());
        Self { error_type, what }
    }

    pub fn detailed_error(&self) -> String {
        format!(
            "Simba Error of type {}: {}",
            self.error_type.to_string(),
            self.what
        )
    }

    pub fn error_type(&self) -> SimbaErrorTypes {
        self.error_type
    }
}

impl Display for SimbaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Simba Error: {}", self.error_type.to_string())
    }
}

impl Debug for SimbaError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "Simba Error of type {}: {}",
            self.error_type.to_string(),
            self.what
        )
    }
}

impl Error for SimbaError {}

pub type SimbaResult<T> = Result<T, SimbaError>;
