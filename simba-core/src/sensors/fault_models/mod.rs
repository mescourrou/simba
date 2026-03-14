//! Fault models used by sensor implementations.
//!
//! Fault models are used to inject faults in the sensor observations, to simulate realistic scenarios where sensors can be affected by various types of faults. Fault models are applied in the order they are defined in the sensor config, and can be used to simulate different types of faults, such as additive faults, misdetection, misassociation, clutter, etc.
pub mod additive;
pub mod clutter;
pub mod external_fault;
pub mod misassociation;
pub mod misdetection;
pub mod python_fault_model;

pub mod fault_model;
