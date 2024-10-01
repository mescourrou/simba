use core::panic;
use std::str::FromStr;

use log::debug;
use pyo3::{prelude::*, types::{PyDict, PyFloat, PyTuple}};
use serde_json::Value;

use crate::stateful::Stateful;

use super::{external_estimator::ExternalEstimatorRecord, state_estimator::{State, StateEstimator, StateEstimatorRecord}};

pub fn make_state_estimator_module(m: &Bound<'_, PyModule>) -> PyResult<()> {
    let module = PyModule::new_bound(m.py(), "state_estimators")?;

    module.add_class::<PythonStateEstimator>()?;
    m.add_submodule(&module)
}

#[derive(Debug)]
#[pyclass]
pub struct PythonStateEstimator {
    model: Py<PyAny>,
}

#[pymethods]
impl PythonStateEstimator {
    #[new]
    pub fn new(py_model: Py<PyAny>) -> PythonStateEstimator {
        Python::with_gil(|py| {
            debug!("Model got: {}", py_model.bind(py).dir().unwrap());
        });
        PythonStateEstimator { 
            model: py_model
        }
    }
}

impl StateEstimator for PythonStateEstimator {
    fn prediction_step(&mut self, turtle: &mut crate::turtlebot::Turtlebot, time: f32) {
        debug!("Calling the pybind implementation of prediction_step");
        debug!("Calling python implementation of prediction_step");
        Python::with_gil(|py| {
            self.model
                .bind(py)
                .call_method("prediction_step", (time,), None)
                .expect("Python implementation of PythonStateEstimator does not have a correct 'prediction_step' method");
        });
    }

    fn correction_step(
        &mut self,
        turtle: &mut crate::turtlebot::Turtlebot,
        observations: &Vec<Box<dyn crate::sensors::sensor::GenericObservation>>,
        time: f32,
    ) {
        debug!("Calling the pybind implementation of correction_step");
        debug!("Calling python implementation of correction_step");
        Python::with_gil(|py| {
            self.model
                .bind(py)
                .call_method("correction_step", (time,), None)
                .expect("Python implementation of PythonStateEstimator does not have a correct 'correction_step' method");
        });
    }

    fn state(&self) -> State {
        debug!("Calling the pybind implementation of state");
        debug!("Calling python implementation of state");
        State::from_vector(Python::with_gil(|py| {
            self.model
                .bind(py)
                .call_method("state", (), None)
                .expect("Python implementation of PythonStateEstimator does not have a correct 'state' method")
                .extract()
                .expect("The 'state' method of PythonStateEstimator does not return a correct state vector")
        }))
    }

    fn next_time_step(&self) -> f32 {
        debug!("Calling the pybind implementation next_time_step");
        // PythonStateEstimator::next_time_step(self)
        debug!("Calling python implementation of next_time_step");
        let time = Python::with_gil(|py| {
            debug!("Bouh");
            self.model
                .bind(py)
                .call_method("next_time_step", (), None)
                .expect("Python implementation of PythonStateEstimator does not have a correct 'next_time_step' method")
                .extract()
                .expect("The 'next_time_step' method of PythonStateEstimator does not return a correct time for next step")
        });
        debug!("Out");
        time
    }
}

impl Stateful<StateEstimatorRecord> for PythonStateEstimator {
    fn record(&self) -> StateEstimatorRecord {
        debug!("Calling the pybind implementation of record");
        debug!("Calling python implementation of record");
        let record_str: String = Python::with_gil(|py| {
            self.model
                .bind(py)
                .call_method("record", (), None)
                .expect("Python implementation of PythonStateEstimator does not have a correct 'record' method")
                .extract()
                .expect("The 'record' method of PythonStateEstimator does not return a valid EstimatorRecord type")
        });
        debug!("Out of python record.");
        let record = ExternalEstimatorRecord {
            record: Value::from_str(record_str.as_str()).expect("Impossible to get serde_json::Value from the input serialized python structure"),
        };
        // record.clone()
        // StateEstimatorRecord::External(PythonStateEstimator::record(&self))
        StateEstimatorRecord::External(record)
    }

    fn from_record(&mut self, record: StateEstimatorRecord) {
        if let StateEstimatorRecord::External(record) = record {
            debug!("Calling python implementation of from_record");
            Python::with_gil(|py| {
                self.model
                    .bind(py)
                    .call_method("from_record", (serde_json::to_string(&record).unwrap(),), None)
                    .expect("Python implementation of PythonStateEstimator does not have a correct 'from_record' method");
            });
        }
    }
}