use log::debug;
use pyo3::prelude::*;
use serde::Serialize;
use serde_json::Value;

use crate::{
    plugin_api::PluginAPI,
    simulator::{Record, Simulator, SimulatorConfig},
    state_estimators::{
        pybinds::{make_state_estimator_module, PythonStateEstimator},
        state_estimator::StateEstimator,
    },
};
use std::path::Path;

pub fn make_python_bindings(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<SimulatorWrapper>()?;
    m.add_class::<PythonAPI>()?;
    m.add_class::<Record>()?;
    make_state_estimator_module(m)?;
    Ok(())
}

#[pyclass]
#[pyo3(name = "Simulator")]
struct SimulatorWrapper {
    simulator: Simulator,
}

#[pymethods]
impl SimulatorWrapper {
    #[staticmethod]
    #[pyo3(signature = (config_path, api=None, loglevel="off"))]
    pub fn from_config(
        config_path: String,
        api: Option<&PythonAPI>,
        loglevel: &str,
    ) -> SimulatorWrapper {
        Simulator::init_environment(match loglevel.to_lowercase().as_str() {
            "debug" => log::LevelFilter::Debug,
            "info" => log::LevelFilter::Info,
            "warn" => log::LevelFilter::Warn,
            "error" => log::LevelFilter::Error,
            "off" => log::LevelFilter::Off,
            &_ => log::LevelFilter::Off,
        }, Vec::new());
        SimulatorWrapper {
            simulator: Simulator::from_config_path(
                Path::new(&config_path),
                match api {
                    Some(py_api) => Some(Box::new(py_api)),
                    None => None,
                },
            ),
        }
    }

    pub fn show(&self) {
        self.simulator.show();
    }

    pub fn run(&mut self, max_time: f32) {
        self.simulator.run(max_time);
    }
}

#[derive(Debug)]
#[pyclass]
pub struct PythonAPI {
    api: Py<PyAny>,
}

#[pymethods]
impl PythonAPI {
    #[new]
    pub fn new(m: Py<PyAny>) -> PythonAPI {
        PythonAPI { api: m }
    }
}

impl PluginAPI for PythonAPI {
    fn get_state_estimator(
        &self,
        config: &Value,
        global_config: &SimulatorConfig,
    ) -> Box<dyn StateEstimator> {
        let st = Box::new(PythonStateEstimator::new(Python::with_gil(|py| {
            self.api
                .bind(py)
                .call_method(
                    "get_state_estimator",
                    (
                        config.to_string(),
                        serde_json::to_string(global_config).expect("Failed to serialize global_config"),
                    ),
                    None,
                )
                .expect("Error during execution of python method 'get_state_estimator'")
                .extract()
                .expect("Expecting function return of PythonStateEstimator but failed")
        })));
        debug!("Got api {:?}", st);
        st
    }
}
