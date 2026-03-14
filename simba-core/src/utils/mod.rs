//! Shared utility modules and helper types used across Simba.
//!
//! This module provides serialization helpers plus common shared-pointer aliases.

pub mod barrier;
pub mod confy;
pub mod determinist_random_variable;
pub mod distributions;
pub mod enum_tools;
pub mod geometry;
pub mod macros;
pub mod maths;
pub mod numbers;
pub mod occupancy_grid;
pub mod periodicity;
pub mod python;
pub mod read_only_lock;

use serde::Serializer;

/// Serialize an `f32` using a stable high-precision scientific representation.
pub fn format_f32<S>(val: &f32, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    let s = format!("{:.40e}", val);
    let number: f64 = s.parse().unwrap();
    serializer.serialize_f64(number)
}

/// Serialize an optional `f32` using [`format_f32`] when present.
pub fn format_option_f32<S>(val: &Option<f32>, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    match val {
        Some(inner) => format_f32(inner, serializer),
        None => serializer.serialize_none(),
    }
}

/// Shared trait-object read-only lock (Shorthand for `Arc<dyn read_only_lock::RoLock<T>>`).
pub type SharedRoLock<T> = std::sync::Arc<dyn read_only_lock::RoLock<T>>;
/// Shared read-write lock (Shorthand for `Arc<RwLock<T>>`).
pub type SharedRwLock<T> = std::sync::Arc<std::sync::RwLock<T>>;
/// Shared mutex lock (Shorthand for `Arc<Mutex<T>>`).
pub type SharedMutex<T> = std::sync::Arc<std::sync::Mutex<T>>;
