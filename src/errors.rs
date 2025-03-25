use std::{
    error::Error,
    fmt::{Debug, Display},
};

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum SimbaErrorTypes {
    UnknwonError,
    MathError,
}

pub struct SimbaError {
    error_type: SimbaErrorTypes,
    what: String,
}

impl SimbaError {
    pub fn new(error_type: SimbaErrorTypes, what: &str) -> Self {
        Self {
            error_type,
            what: what.to_string(),
        }
    }
}

impl Display for SimbaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let error_message = match self.error_type {
            SimbaErrorTypes::UnknwonError | _ => "Unknown error",
        };
        write!(f, "Simba Error: {}", error_message)
    }
}

impl Debug for SimbaError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "Simba Error of type {:?}: {}",
            self.error_type, self.what
        )
    }
}

impl Error for SimbaError {}
