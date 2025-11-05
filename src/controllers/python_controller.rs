/*!
Module providing the interface to use external Python [`Controller`].
*/

use std::fs;
use std::str::FromStr;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};

use config_checker::macros::Check;
use log::{debug, info};
use pyo3::prelude::*;
use pyo3::types::PyModule;
use pyo3::{pyclass, pymethods, PyResult, Python};
use serde_json::Value;

#[cfg(feature = "gui")]
use crate::gui::{utils::json_config, UIComponent};

use crate::networking::message_handler::MessageHandler;
use crate::networking::network::Envelope;
use crate::pywrappers::NodeWrapper;
use crate::{
    controllers::controller::{Controller, ControllerError, ControllerRecord},
    errors::{SimbaError, SimbaErrorTypes, SimbaResult},
    logger::is_enabled,
    physics::physics::Command,
    pywrappers::{CommandWrapper, ControllerErrorWrapper},
    recordable::Recordable,
    simulator::SimulatorConfig,
};

use serde_derive::{Deserialize, Serialize};

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
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
pub struct PythonControllerConfig {
    file: String,
    class_name: String,
    /// Config serialized.
    #[serde(flatten)]
    pub config: Value,
}

impl Default for PythonControllerConfig {
    fn default() -> Self {
        Self {
            file: String::new(),
            class_name: String::new(),
            config: Value::Null,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PythonControllerConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new("External Python Controller").show(ui, |ui| {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    ui.label("Script path: ");
                    ui.text_edit_singleline(&mut self.file);
                });

                ui.horizontal(|ui| {
                    ui.label("Class name: ");
                    ui.text_edit_singleline(&mut self.class_name);
                });

                ui.label("Config (JSON):");
                json_config(
                    ui,
                    &format!("external-python-controller-key-{}", &unique_id),
                    &format!("external-python-controller-error-key-{}", &unique_id),
                    buffer_stack,
                    &mut self.config,
                );
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &String) {
        egui::CollapsingHeader::new("External Python Controller").show(ui, |ui| {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    ui.label("Script path: ");
                    ui.label(&self.file);
                });
                ui.horizontal(|ui| {
                    ui.label("Class name: ");
                    ui.label(&self.class_name);
                });
                ui.label("Config (JSON):");
                ui.label(self.config.to_string());
            });
        });
    }
}

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
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &String) {
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

        // prepare_freethreaded_python();

        let json_config = serde_json::to_string(&config)
            .expect("Error during converting Python Controller config to json");

        let convert_to_dict = r#"
import json
class NoneDict(dict):
    """ dict subclass that returns a value of None for missing keys instead
        of raising a KeyError. Note: doesn't add item to dictionary.
    """
    def __missing__(self, key):
        return None


def converter(decoded_dict):
    """ Convert any None values in decoded dict into empty NoneDict's. """
    return {k: NoneDict() if v is None else v for k,v in decoded_dict.items()}

def convert(records):
    return json.loads(records, object_hook=converter)
"#;

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
            Ok(s) => s,
        };
        let res = Python::with_gil(|py| -> PyResult<Py<PyAny>> {
            let script = PyModule::from_code_bound(py, &convert_to_dict, "", "")?;
            let convert_fn: Py<PyAny> = script.getattr("convert")?.into();
            let config_dict = convert_fn.call_bound(py, (json_config,), None)?;

            let script = PyModule::from_code_bound(py, &python_script, "", "")?;
            let controller_class: Py<PyAny> = script.getattr(config.class_name.as_str())?.into();
            info!("Load Controller class {} ...", config.class_name);

            let res = controller_class.call_bound(py, (config_dict,), None);
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
        let node_py = NodeWrapper::from_rust(&node, self.letter_box_receiver.clone());
        let result = Python::with_gil(|py| -> CommandWrapper {
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
        let node_py = NodeWrapper::from_rust(&node, self.letter_box_receiver.clone());
        Python::with_gil(|py| {
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
        let record_str: String = Python::with_gil(|py| {
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
