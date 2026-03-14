#![warn(missing_docs)]
#![deny(rustdoc::broken_intra_doc_links)]
//! Multi-robot simulator with asynchronous, event-driven time progression.
//!
//! This crate provides a multi-robot simulator where each robot (node) executes in a dedicated thread.
//! Simulation time is non-linear: the engine jumps directly to the next relevant event, allowing
//! execution faster than real time when possible.
//!
//! Configuration drives the full simulation flow. The root configuration type is
//! [`SimulatorConfig`](crate::simulator::SimulatorConfig).
//!
//! Main subsystems include:
//! - [`physics`] for robot ground-truth dynamics,
//! - [`navigators`] and [`controllers`] for guidance and control,
//! - [`state_estimators`] and [`sensors`] for perception and estimation.
//!
//! The main runtime entry point is [`simulator::Simulator`].
//!
//! # Example
//! ```no_run
//! use std::path::Path;
//! use simba::simulator::Simulator;
//!
//! Simulator::init_environment();
//! let mut simulator = Simulator::from_config_path(
//!     Path::new("config_example/config.yaml"),
//!     None,
//! ).unwrap();
//! simulator.show();
//! simulator.run().unwrap();
//! simulator.compute_results().unwrap();
//! ```

#![doc = include_str!("../../doc/user_manual/docs/config_documentation.md")]

use pyo3::prelude::*;

pub use simba_com;

pub mod config;
pub mod controllers;
pub mod environment;
pub mod logger;
pub mod navigators;
pub mod networking;
pub mod node;
pub mod physics;
pub mod recordable;
pub mod scenario;
pub mod sensors;
pub mod simulator;
pub mod state_estimators;
pub mod utils;

pub mod plugin_api;
pub mod pybinds;
pub mod pywrappers;

pub mod api;
pub mod time_analysis;

#[cfg(test)]
mod integration_tests;

#[cfg(feature = "gui")]
pub mod gui;

#[pymodule]
/// Python module initializer for the `simba` package.
pub fn simba(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pybinds::make_python_bindings(m)?;
    Ok(())
}

pub mod constants;
pub mod errors;

/// Crate version extracted from Cargo package metadata.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
/// Crate authors extracted from Cargo package metadata.
pub const AUTHORS: &str = env!("CARGO_PKG_AUTHORS");
