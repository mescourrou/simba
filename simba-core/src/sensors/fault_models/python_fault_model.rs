use pyo3::prelude::*;

use crate::{
    errors::SimbaResult,
    pywrappers::SensorObservationWrapper,
    sensors::{SensorObservation, fault_models::fault_model::FaultModel},
    simulator::SimulatorConfig,
    utils::{
        macros::python_class_config,
        python::{call_py_method, load_class_from_python_script},
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
    fn add_faults(
        &self,
        time: f32,
        period: f32,
        obs_list: &mut Vec<SensorObservation>,
        _obs_type: SensorObservation,
    ) {
        let py_obs_list: Vec<SensorObservationWrapper> = obs_list
            .iter()
            .map(|o| SensorObservationWrapper::from_rust(o))
            .collect();
        let py_obs_list = call_py_method!(
            self.instance,
            "add_faults",
            Vec<SensorObservationWrapper>,
            time,
            period,
            py_obs_list
        );
        obs_list.clear();
        for py_obs in py_obs_list {
            obs_list.push(py_obs.to_rust());
        }
    }
}
