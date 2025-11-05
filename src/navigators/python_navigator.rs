/*!
Module providing the interface to use external Python [`Navigator`].
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
    controllers::controller::ControllerError,
    errors::{SimbaError, SimbaErrorTypes, SimbaResult},
    logger::is_enabled,
    navigators::navigator::{Navigator, NavigatorRecord},
    pywrappers::{ControllerErrorWrapper, WorldStateWrapper},
    recordable::Recordable,
    simulator::SimulatorConfig,
    state_estimators::state_estimator::WorldState,
};
use serde_derive::{Deserialize, Serialize};

/// Config for the external navigator (generic).
///
/// The config for [`PythonNavigator`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
///
/// You need to provide the path of the script containing the navigator.
///
/// In the yaml file, the config could be:
/// ```YAML
/// navigator:
///     Python:
///         file: ""../my_python_script.py"
///         class_name: MyNavigator
///         parameter_of_my_own_navigator: true
/// ```
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
pub struct PythonNavigatorConfig {
    file: String,
    class_name: String,
    /// Config serialized.
    #[serde(flatten)]
    pub config: Value,
}

impl Default for PythonNavigatorConfig {
    fn default() -> Self {
        Self {
            file: String::new(),
            class_name: String::new(),
            config: Value::Null,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PythonNavigatorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new("External Python Navigator").show(ui, |ui| {
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
                    &format!("external-python-navigator-key-{}", &unique_id),
                    &format!("external-python-navigator-error-key-{}", &unique_id),
                    buffer_stack,
                    &mut self.config,
                );
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &String) {
        egui::CollapsingHeader::new("External Python Navigator").show(ui, |ui| {
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

/// Record for the external navigator (generic).
///
/// Like [`PythonNavigatorConfig`], [`PythonNavigator`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::record`] implementations.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass]
pub struct PythonNavigatorRecord {
    /// Record serialized.
    #[serde(flatten)]
    pub record: Value,
}

impl Default for PythonNavigatorRecord {
    fn default() -> Self {
        Self {
            record: Value::Null,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PythonNavigatorRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &String) {
        ui.label(self.record.to_string());
    }
}

#[pymethods]
impl PythonNavigatorRecord {
    #[getter]
    fn record(&self) -> String {
        self.record.to_string()
    }
}

use crate::node::Node;

/// External navigator strategy, which does the bridge with your own strategy.
pub struct PythonNavigator {
    /// External navigator.
    navigator: Py<PyAny>,
    letter_box_receiver: Arc<Mutex<Receiver<Envelope>>>,
    letter_box_sender: Sender<Envelope>,
}

impl PythonNavigator {
    /// Creates a new [`PythonNavigator`]
    pub fn new() -> SimbaResult<Self> {
        Self::from_config(
            &PythonNavigatorConfig::default(),
            &SimulatorConfig::default(),
        )
    }

    /// Creates a new [`PythonNavigator`] from the given config.
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External estimator.
    /// * `plugin_api` -- [`PluginAPI`] implementation (not used).
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables.
    pub fn from_config(
        config: &PythonNavigatorConfig,
        global_config: &SimulatorConfig,
    ) -> SimbaResult<Self> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }

        // prepare_freethreaded_python();

        let json_config = serde_json::to_string(&config)
            .expect("Error during converting Python Navigator config to json");

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
                        "Python navigator script not found ({}): {}",
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
            let navigator_class: Py<PyAny> = script.getattr(config.class_name.as_str())?.into();
            info!("Load Navigator class {} ...", config.class_name);

            let res = navigator_class.call_bound(py, (config_dict,), None);
            let navigator_instance = match res {
                Err(err) => {
                    err.display(py);
                    return Err(err);
                }
                Ok(instance) => instance,
            };
            Ok(navigator_instance)
        });
        let navigator_instance = match res {
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
            navigator: navigator_instance,
            letter_box_receiver: Arc::new(Mutex::new(rx)),
            letter_box_sender: tx,
        })
    }
}

impl std::fmt::Debug for PythonNavigator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "PythonNavigator {{}}")
    }
}

impl Navigator for PythonNavigator {
    fn compute_error(&mut self, node: &mut Node, state: WorldState) -> ControllerError {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of compute_error");
        }
        let node_py = NodeWrapper::from_rust(&node, self.letter_box_receiver.clone());
        let result = Python::with_gil(|py| -> ControllerErrorWrapper {
            match self.navigator.bind(py).call_method(
                "compute_error",
                (node_py, WorldStateWrapper::from_rust(&state)),
                None,
            ) {
                Err(e) => {
                    e.display(py);
                    panic!("Error while calling 'compute_error' method of PythonNavigator.");
                }
                Ok(r) => r
                    .extract()
                    .expect("Error during the call of Python implementation of 'compute_error'"),
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
                self.navigator
                    .bind(py)
                    .call_method("pre_loop_hook", (node_py, time), None)
            {
                e.display(py);
                panic!("Error while calling 'pre_loop_hook' method of PythonNavigator.");
            }
        });
    }
}

impl Recordable<NavigatorRecord> for PythonNavigator {
    fn record(&self) -> NavigatorRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = Python::with_gil(|py| {
            match self.navigator
                .bind(py)
                .call_method("record", (), None) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error while calling 'record' method of PythonNavigator.");
                    }
                    Ok(r) => {
                        r.extract()
                        .expect("The 'record' method of PythonNavigator does not return a valid PythonNavigatorRecord type")
                    }
                }
        });
        let record = PythonNavigatorRecord {
            record: Value::from_str(record_str.as_str()).expect(
                "Impossible to get serde_json::Value from the input serialized python structure",
            ),
        };
        // record.clone()
        // StateEstimatorRecord::External(PythonNavigator::record(&self))
        NavigatorRecord::Python(record)
    }
}

impl MessageHandler for PythonNavigator {
    fn get_letter_box(&self) -> Option<Sender<Envelope>> {
        Some(self.letter_box_sender.clone())
    }
}
