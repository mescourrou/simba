use serde::{Deserialize, Serialize};
use simba_macros::config_derives;

use crate::utils::determinist_random_variable::RandomVariableTypeConfig;

#[config_derives(tag_content)]
pub enum NumberConfig {
    Num(f32),
    Rand(RandomVariableTypeConfig),
}

impl Default for NumberConfig {
    fn default() -> Self {
        Self::Num(0.0)
    }
}
