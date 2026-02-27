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
    PythonFaultModelConfig,
    "Python Fault Model",
    "python-fault-model"
);

#[derive(Debug)]
pub struct PythonFaultModel {
    instance: Py<PyAny>,
}

impl PythonFaultModel {
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
        environment: &Arc<Environment>,
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
