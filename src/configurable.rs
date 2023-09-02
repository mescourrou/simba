use std::path::Path;

use std::fmt;

pub struct ConfigurationLoadingError {
    pub what: String
}

// Implement std::fmt::Display for AppError
impl fmt::Display for ConfigurationLoadingError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.what) // user-facing output
    }
}

impl fmt::Debug for ConfigurationLoadingError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{{ file: {}, line: {} }}", file!(), line!()) // programmer-facing output
    }
}
