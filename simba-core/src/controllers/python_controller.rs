/*!
Module providing the interface to use external Python [`Controller`].
*/

use std::ffi::CString;
use std::fs;
use std::str::FromStr;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};

use config_checker::macros::Check;
use log::{debug, info};
use pyo3::ffi::c_str;
use pyo3::prelude::*;
use pyo3::types::PyModule;
use pyo3::{pyclass, pymethods, PyResult, Python};
use serde_json::Value;

#[cfg(feature = "gui")]
use crate::gui::{utils::json_config, UIComponent};

use crate::networking::message_handler::MessageHandler;
use crate::networking::network::Envelope;
use crate::physics::robot_models::Command;
use crate::pywrappers::NodeWrapper;
use crate::utils::python::{CONVERT_TO_DICT, ensure_venv_pyo3, python_class_config};
use crate::{
    controllers::{Controller, ControllerError, ControllerRecord},
    errors::{SimbaError, SimbaErrorTypes, SimbaResult},
    logger::is_enabled,
    pywrappers::{CommandWrapper, ControllerErrorWrapper},
    recordable::Recordable,
    simulator::SimulatorConfig,
};

use serde_derive::{Deserialize, Serialize};

python_class_config!(
/// Config for the external controller (generic).
///
/// The config for [`PythonController`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
///
/// You need to provide the path of the script containing the controller.
///
/// In the yaml file, the config could be:
/// ```YAML
/// controller:
///     Python:
///         file: ""../my_python_script.py"
///         class_name: MyController
///         parameter_of_my_own_controller: true
/// ```
    PythonControllerConfig,
    "External Python Controller",
    "external-python-controller"
);

/// Record for the external controller (generic).
///
/// Like [`PythonControllerConfig`], [`PythonController`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::record`] implementations.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass]
pub struct PythonControllerRecord {
    /// Record serialized.
    #[serde(flatten)]
    pub record: Value,
}

impl Default for PythonControllerRecord {
    fn default() -> Self {
        Self {
            record: Value::Null,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PythonControllerRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.label(self.record.to_string());
    }
}

#[pymethods]
impl PythonControllerRecord {
    #[getter]
    fn record(&self) -> String {
        self.record.to_string()
    }
}

use crate::node::Node;

/// External controller strategy, which does the bridge with your own strategy.
pub struct PythonController {
    /// External controller.
    controller: Py<PyAny>,
    letter_box_receiver: Arc<Mutex<Receiver<Envelope>>>,
    letter_box_sender: Sender<Envelope>,
}

impl PythonController {
    /// Creates a new [`PythonController`]
    pub fn new() -> SimbaResult<Self> {
        Self::from_config(
            &PythonControllerConfig::default(),
            &SimulatorConfig::default(),
        )
    }

    /// Creates a new [`PythonController`] from the given config.
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External estimator.
    /// * `plugin_api` -- [`PluginAPI`] implementation (not used).
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables.
    pub fn from_config(
        config: &PythonControllerConfig,
        global_config: &SimulatorConfig,
    ) -> SimbaResult<Self> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }

        let json_config = serde_json::to_string(&config)
            .expect("Error during converting Python Controller config to json");


        let script_path = global_config.base_path.as_ref().join(&config.file);
        let python_script = match fs::read_to_string(script_path.clone()) {
            Err(e) => {
                return Err(SimbaError::new(
                    SimbaErrorTypes::ConfigError,
                    format!(
                        "Python controller script not found ({}): {}",
                        script_path.to_str().unwrap(),
                        e
                    ),
                ))
            }
            Ok(s) => CString::new(s).unwrap(),
        };
        let res = Python::attach(|py| -> PyResult<Py<PyAny>> {
            ensure_venv_pyo3(py)?;
            let script = PyModule::from_code(py, CONVERT_TO_DICT, c_str!(""), c_str!(""))?;
            let convert_fn: Py<PyAny> = script.getattr("convert")?.into();
            let config_dict = convert_fn.call(py, (json_config,), None)?;

            let script = PyModule::from_code(py, &python_script, c_str!(""), c_str!(""))?;
            let controller_class: Py<PyAny> = script.getattr(config.class_name.as_str())?.into();
            info!("Load Controller class {} ...", config.class_name);

            let res = controller_class.call(py, (config_dict,), None);
            let controller_instance = match res {
                Err(err) => {
                    err.display(py);
                    return Err(err);
                }
                Ok(instance) => instance,
            };
            Ok(controller_instance)
        });
        let controller_instance = match res {
            Err(err) => {
                return Err(SimbaError::new(
                    SimbaErrorTypes::PythonError,
                    err.to_string(),
                ))
            }
            Ok(instance) => instance,
        };
        let (tx, rx) = mpsc::channel();
        Ok(Self {
            controller: controller_instance,
            letter_box_receiver: Arc::new(Mutex::new(rx)),
            letter_box_sender: tx,
        })
    }
}

impl std::fmt::Debug for PythonController {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "PythonController {{}}")
    }
}

impl Controller for PythonController {
    fn make_command(&mut self, node: &mut Node, error: &ControllerError, time: f32) -> Command {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of make_command");
        }
        let node_py = NodeWrapper::from_rust(node, self.letter_box_receiver.clone());
        let result = Python::attach(|py| -> CommandWrapper {
            match self.controller.bind(py).call_method(
                "make_command",
                (node_py, ControllerErrorWrapper::from_rust(error), time),
                None,
            ) {
                Err(e) => {
                    e.display(py);
                    panic!("Error while calling 'make_command' method of PythonController.");
                }
                Ok(r) => r
                    .extract()
                    .expect("Error during the call of Python implementation of 'make_command'"),
            }
        });
        result.to_rust()
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of pre_loop_hook");
        }
        let node_py = NodeWrapper::from_rust(node, self.letter_box_receiver.clone());
        Python::attach(|py| {
            if let Err(e) =
                self.controller
                    .bind(py)
                    .call_method("pre_loop_hook", (node_py, time), None)
            {
                e.display(py);
                panic!("Error while calling 'pre_loop_hook' method of PythonController.");
            }
        });
    }
}

impl Recordable<ControllerRecord> for PythonController {
    fn record(&self) -> ControllerRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = Python::attach(|py| {
            match self.controller
                .bind(py)
                .call_method("record", (), None) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error while calling 'record' method of PythonController.");
                    }
                    Ok(r) => {
                        r.extract()
                        .expect("The 'record' method of PythonController does not return a valid PythonControllerRecord type")
                    }
                }
        });
        let record = PythonControllerRecord {
            record: Value::from_str(record_str.as_str()).expect(
                "Impossible to get serde_json::Value from the input serialized python structure",
            ),
        };
        // record.clone()
        // StateEstimatorRecord::External(PythonController::record(&self))
        ControllerRecord::Python(record)
    }
}

impl MessageHandler for PythonController {
    fn get_letter_box(&self) -> Option<Sender<Envelope>> {
        Some(self.letter_box_sender.clone())
    }
}
