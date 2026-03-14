//! Shared low-level configuration primitives used across simulator modules.
//!
//! This module currently defines [`NumberConfig`], which represents either a fixed scalar value
//! or a sampled random-variable configuration.

use simba_macros::config_derives;

use crate::utils::determinist_random_variable::RandomVariableTypeConfig;

/// Numeric configuration value.
///
/// This enum is typically used in higher-level `*Config` structures when a field can be either a
/// fixed number or a random variable.
///
/// Default: [`NumberConfig::Num`] with `0.0`.
#[config_derives(tag_content)]
pub enum NumberConfig {
    /// Deterministic numeric value.
    Num(f32),
    /// Random variable description to sample numeric values from.
    #[check]
    Rand(RandomVariableTypeConfig),
}

impl Default for NumberConfig {
    fn default() -> Self {
        Self::Num(0.0)
    }
}
