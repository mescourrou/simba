//! Python-based sensor observation filtering.
//!
//! This module provides a `SensorFilter` implementation that delegates filtering logic to a Python class.
//! Users can define custom filter behavior by implementing a Python class with [`SensorFilter`] interface
//! that will be invoked during simulation to modify or drop sensor observations.

use pyo3::prelude::*;

use crate::{
    errors::SimbaResult,
    node::Node,
    pywrappers::{NodeWrapper, SensorObservationWrapper, StateWrapper},
    sensors::{SensorObservation, sensor_filters::SensorFilter},
    simulator::SimulatorConfig,
    state_estimators::State,
    utils::{
        macros::python_class_config,
        python::{call_py_method, call_py_method_void, load_class_from_python_script},
    },
};

python_class_config!(
    /// Configuration for the Python-based sensor filter.
    PythonFilterConfig, "Python Filter", "python-filter");

#[derive(Debug)]
/// Python-based sensor observation filter implementation.
///
/// This struct wraps a Python filter instance loaded from a user-provided script.
/// It dynamically invokes Python methods to decide whether to filter observations.
pub struct PythonFilter {
    instance: Py<PyAny>,
}

impl PythonFilter {
    /// Creates a new Python filter by loading the user-defined Python class.
    ///
    /// Constructs the runtime filter instance from configuration by loading the Python class
    /// specified in the config via `load_class_from_python_script()`.
    pub fn from_config(
        config: &PythonFilterConfig,
        global_config: &SimulatorConfig,
        initial_time: f32,
    ) -> SimbaResult<Self> {
        let instance =
            load_class_from_python_script(config, global_config, initial_time, "Filter")?;
        Ok(Self { instance })
    }
}

impl SensorFilter for PythonFilter {
    fn post_init(&mut self, node: &mut Node, initial_time: f32) -> SimbaResult<()> {
        let py_node = NodeWrapper::from_rust(node);
        call_py_method_void!(self.instance, "post_init", py_node, initial_time);
        Ok(())
    }

    fn filter(
        &self,
        time: f32,
        observation: SensorObservation,
        observer_state: &State,
        observee_state: Option<&State>,
    ) -> Option<SensorObservation> {
        let py_observation = SensorObservationWrapper::from_rust(&observation);
        let py_observer_state = StateWrapper::from_rust(observer_state);
        let py_observee_state = observee_state.map(StateWrapper::from_rust);
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
