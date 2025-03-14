use log::debug;
use pyo3::prelude::*;
use serde_json::Value;

use crate::{
    api::async_api::{AsyncApi, AsyncApiRunner, PluginAsyncAPI}, controllers::{controller::Controller, pybinds::{make_controllers_module, PythonController}}, plugin_api::PluginAPI, simulator::{Record, Simulator, SimulatorConfig}, state_estimators::{
        pybinds::{make_state_estimators_module, PythonStateEstimator},
        state_estimator::StateEstimator,
    }
};
use std::sync::{Arc, Mutex};

pub fn make_python_bindings(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<SimulatorWrapper>()?;
    m.add_class::<PythonAPI>()?;
    m.add_class::<Record>()?;
    make_state_estimators_module(m)?;
    make_controllers_module(m)?;
    Ok(())
}

#[pyclass]
#[pyo3(name = "Simulator")]
struct SimulatorWrapper {
    server: Arc<Mutex<AsyncApiRunner>>,
    api: AsyncApi,
    async_plugin_api: Option<PluginAsyncAPI>,
}

#[pymethods]
impl SimulatorWrapper {
    #[staticmethod]
    #[pyo3(signature = (config_path, plugin_api=None, loglevel="off"))]
    pub fn from_config(
        config_path: String,
        mut plugin_api: Option<&mut PythonAPI>,
        loglevel: &str,
    ) -> SimulatorWrapper {
        Simulator::init_environment(
            match loglevel.to_lowercase().as_str() {
                "debug" => log::LevelFilter::Debug,
                "info" => log::LevelFilter::Info,
                "warn" => log::LevelFilter::Warn,
                "error" => log::LevelFilter::Error,
                "off" => log::LevelFilter::Off,
                &_ => log::LevelFilter::Off,
            },
            Vec::new(),
            Vec::new(),
        );

        let server = Arc::new(Mutex::new(AsyncApiRunner::new()));
        let api = server.lock().unwrap().get_api();
        let wrapper = SimulatorWrapper {
            server,
            api,
            async_plugin_api: match &plugin_api {
                Some(_) => Some(PluginAsyncAPI::new()),
                None => None,
            },
        };

        // Unsafe use because wrapper is a python object, which should be used until the end. But PyO3 does not support lifetimes to force the behaviour
        wrapper
            .server
            .lock()
            .unwrap()
            .run(match &wrapper.async_plugin_api {
                Some(api) => Some(Box::<&dyn PluginAPI>::new(unsafe {
                    std::mem::transmute::<&dyn PluginAPI, &'static dyn PluginAPI>(api)
                })),
                None => None,
            });
        wrapper.api.load_config.send(config_path).unwrap();

        if let Some(unwrapped_async_api) = &wrapper.async_plugin_api {
            let api_client = &unwrapped_async_api.client;
            let python_api = plugin_api.as_mut().unwrap();
            while wrapper.api.ended.lock().unwrap().try_recv().is_err() {
                if let Ok((config, simulator_config)) = api_client
                    .get_state_estimator_request
                    .lock()
                    .unwrap()
                    .try_recv()
                {
                    let state_estimator =
                        python_api.get_state_estimator(&config, &simulator_config);
                    api_client
                        .get_state_estimator_response
                        .send(state_estimator).unwrap();
                }
                if let Ok((config, simulator_config)) = api_client
                    .get_controller_request
                    .lock()
                    .unwrap()
                    .try_recv()
                {
                    let controller =
                        python_api.get_controller(&config, &simulator_config);
                    api_client
                        .get_controller_response
                        .send(controller).unwrap();
                }
                python_api.check_requests();
            }
        } else {
            wrapper.api.ended.lock().unwrap().recv().unwrap();
        }

        wrapper
    }

    #[pyo3(signature = (plugin_api=None))]
    pub fn run(&mut self, plugin_api: Option<&mut PythonAPI>) {
        self.api
            .run
            .send(None)
            .expect("Error while sending 'run' request");
        if plugin_api.is_none() && self.async_plugin_api.is_some() {
            panic!("Please provide the plugin api for running too");
        }
        if let Some(python_api) = plugin_api {
            while self.api.ended.lock().unwrap().try_recv().is_err() {
                python_api.check_requests();
                if Python::with_gil(|py| py.check_signals()).is_err() {
                    break;
                }
            }
        } else {
            while self.api.ended.lock().unwrap().try_recv().is_err() {
                if Python::with_gil(|py| py.check_signals()).is_err() {
                    break;
                }
            }
        }
        self.server.lock().unwrap().stop();
    }
}

#[derive(Debug)]
#[pyclass]
pub struct PythonAPI {
    api: Py<PyAny>,
    state_estimators: Vec<PythonStateEstimator>,
    controllers: Vec<PythonController>,
}

#[pymethods]
impl PythonAPI {
    #[new]
    pub fn new(m: Py<PyAny>) -> PythonAPI {
        PythonAPI {
            api: m,
            state_estimators: Vec::new(),
            controllers: Vec::new(),
        }
    }
}

impl PythonAPI {
    pub fn check_requests(&mut self) {
        for state_estimator in &mut self.state_estimators {
            state_estimator.check_requests();
        }
        for controller in &mut self.controllers {
            controller.check_requests();
        }
    }

    pub fn get_state_estimator(
        &mut self,
        config: &Value,
        global_config: &SimulatorConfig,
    ) -> Box<dyn StateEstimator> {
        println!("Calling Python API");
        self.state_estimators
            .push(PythonStateEstimator::new(Python::with_gil(|py| {
                self.api
                    .bind(py)
                    .call_method(
                        "get_state_estimator",
                        (
                            config.to_string(),
                            serde_json::to_string(global_config)
                                .expect("Failed to serialize global_config"),
                        ),
                        None,
                    )
                    .expect("Error during execution of python method 'get_state_estimator'")
                    .extract()
                    .expect("Expecting function return of PythonStateEstimator but failed")
            })));
        let st = Box::new(self.state_estimators.last().unwrap().get_client());
        debug!("Got api {:?}", st);
        st
    }

    pub fn get_controller(
        &mut self,
        config: &Value,
        global_config: &SimulatorConfig,
    ) -> Box<dyn Controller> {
        println!("Calling Python API");
        self.controllers
            .push(PythonController::new(Python::with_gil(|py| {
                self.api
                    .bind(py)
                    .call_method(
                        "get_controller",
                        (
                            config.to_string(),
                            serde_json::to_string(global_config)
                                .expect("Failed to serialize global_config"),
                        ),
                        None,
                    )
                    .expect("Error during execution of python method 'get_state_estimator'")
                    .extract()
                    .expect("Expecting function return of PythonStateEstimator but failed")
            })));
        let st = Box::new(self.controllers.last().unwrap().get_client());
        debug!("Got api {:?}", st);
        st
    }
}
