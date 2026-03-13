//! Python-backed sensor fault model integration.
//!
//! This module provides a fault model implementation that delegates fault injection logic
//! to a Python class loaded at runtime from configuration.
//! The Python side is expected to implement the [`FaultModel`] contract,
//! especially the [`FaultModel::add_faults`] method applied to sensor observations.

use std::sync::Arc;

use pyo3::prelude::*;

use crate::{
    environment::Environment,
    errors::SimbaResult,
    node::Node,
    pywrappers::{NodeWrapper, SensorObservationWrapper},
    sensors::{SensorObservation, fault_models::fault_model::FaultModel},
    simulator::SimulatorConfig,
    utils::{
        macros::python_class_config,
        python::{call_py_method, call_py_method_void, load_class_from_python_script},
    },
};

python_class_config!(
    /// Configuration for the Python-based sensor fault model.
    /// 
    /// The implementation of the Python class should follow the interface defined in the documentation of the [`FaultModel`] trait, especially for the [`FaultModel::add_faults()`] method, which is the core of the fault model behavior.
    PythonFaultModelConfig,
    "Python Fault Model",
    "python-fault-model"
);

#[derive(Debug)]
/// Runtime wrapper around a Python fault model instance.
///
/// This type forwards [`FaultModel`] lifecycle and fault-injection calls
/// to the loaded Python object.
pub struct PythonFaultModel {
    instance: Py<PyAny>,
}

impl PythonFaultModel {
    /// Builds a Python fault model from configuration.
    ///
    /// Loads the configured Python class and returns a ready-to-use runtime wrapper.
    pub fn from_config(
        config: &PythonFaultModelConfig,
        global_config: &SimulatorConfig,
        _initial_time: f32,
    ) -> SimbaResult<Self> {
        let instance =
            load_class_from_python_script(config, global_config, _initial_time, "Fault Model")?;
        Ok(Self { instance })
    }
}

impl FaultModel for PythonFaultModel {
    fn post_init(&mut self, node: &mut Node, initial_time: f32) -> SimbaResult<()> {
        let py_node = NodeWrapper::from_rust(node);
        call_py_method_void!(self.instance, "post_init", py_node, initial_time);
        Ok(())
    }

    fn add_faults(
        &mut self,
        time: f32,
        seed: f32,
        obs_list: &mut Vec<SensorObservation>,
        _obs_type: SensorObservation,
        _environment: &Arc<Environment>,
    ) {
        let py_obs_list: Vec<SensorObservationWrapper> = obs_list
            .iter()
            .map(SensorObservationWrapper::from_rust)
            .collect();
        let py_obs_list = call_py_method!(
            self.instance,
            "add_faults",
            Vec<SensorObservationWrapper>,
            time,
            seed,
            py_obs_list
        );
        obs_list.clear();
        for py_obs in py_obs_list {
            obs_list.push(py_obs.to_rust());
        }
    }
}
