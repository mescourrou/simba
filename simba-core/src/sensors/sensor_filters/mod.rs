//! Sensor observation filtering framework.
//!
//! This module provides a flexible filtering system for sensor observations based on multiple criteria.
//! Filters can be configured to accept or reject observations based on numeric ranges (for enumerated variables), string patterns (for sensor IDs or labels), or custom logic via Python or external plugins.
//! Multiple filters can be chained together (the order is important); all custom filters must accept an observation for it to pass through.
//!
//! The module provides [`SensorFilter`]: Base trait for all custom filter implementations

use crate::{errors::SimbaResult, sensors::SensorObservation, state_estimators::State};

pub mod external_filter;
pub mod python_filter;
pub mod range_filter;
pub mod string_filter;

/// Trait defining the sensor observations filter interface for custom implementation.
///
/// All custom filter implementations must satisfy this trait to integrate with the observation filtering system.
/// A filter's decision is binary: it returns `Some(observation)` to keep the observation (can be modified) or `None` to exclude it.
pub trait SensorFilter: Send + Sync + std::fmt::Debug {
    /// Initializes the filter with node context and current simulation time.
    ///
    /// Called once at simulation before starting the simulation loop to allow filters to set up internal state, validate configurations,
    /// or perform any needed interactions with the node. This is optional; default implementation does nothing.
    #[allow(unused_variables)]
    fn post_init(&mut self, node: &mut crate::node::Node, initial_time: f32) -> SimbaResult<()> {
        Ok(())
    }
    /// Applies the filter to an observation and decides whether to keep or exclude it. The observation can be modified.
    ///
    /// Returns `Some(observation)` to keep, or `None` to exclude from further processing.
    /// The filter has access to observer and observee states for context-aware decisions.
    fn filter(
        &self,
        time: f32,
        observation: SensorObservation,
        observer_state: &State,
        observee_state: Option<&State>,
    ) -> Option<SensorObservation>;
}
