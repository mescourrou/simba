use pyo3::prelude::*;

use crate::{
    errors::SimbaResult,
    pywrappers::{SensorObservationWrapper, StateWrapper},
    sensors::{sensor_filters::SensorFilter, SensorObservation},
    simulator::SimulatorConfig,
    state_estimators::State,
    utils::{
        macros::python_class_config,
        python::{call_py_method, load_class_from_python_script},
    },
};

python_class_config!(PythonFilterConfig, "Python Filter", "python-filter");

#[derive(Debug)]
pub struct PythonFilter {
    instance: Py<PyAny>,
}

impl PythonFilter {
    pub fn from_config(
        config: &PythonFilterConfig,
        global_config: &SimulatorConfig,
    ) -> SimbaResult<Self> {
        let instance = load_class_from_python_script(config, global_config, "Filter")?;
        Ok(Self { instance })
    }
}

impl SensorFilter for PythonFilter {
    fn filter(
        &self,
        time: f32,
        observation: SensorObservation,
        observer_state: &State,
        observee_state: Option<&State>,
    ) -> Option<SensorObservation> {
        let py_observation = SensorObservationWrapper::from_rust(&observation);
        let py_observer_state = StateWrapper::from_rust(observer_state);
        let py_observee_state = observee_state.map(|s| StateWrapper::from_rust(s));
        let ret = call_py_method!(
            self.instance,
            "filter",
            Option<SensorObservationWrapper>,
            time,
            py_observation,
            py_observer_state,
            py_observee_state
        );
        ret.map(|o| o.to_rust())
    }
}
