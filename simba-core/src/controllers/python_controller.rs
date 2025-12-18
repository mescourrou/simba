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
use pyo3::{PyResult, Python, pyclass, pymethods};
use serde_json::Value;

#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::json_config};

use crate::networking::message_handler::MessageHandler;
use crate::networking::network::Envelope;
use crate::physics::robot_models::Command;
use crate::pywrappers::NodeWrapper;
use crate::utils::macros::{external_record_python_methods, python_class_config};
use crate::utils::python::{
    CONVERT_TO_DICT, call_py_method, call_py_method_void, ensure_venv_pyo3,
    load_class_from_python_script,
};
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

external_record_python_methods!(
/// Record for the external controller (generic).
///
/// Like [`PythonControllerConfig`], [`PythonController`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::record`] implementations.
PythonControllerRecord,
);

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
            0.0,
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
        initial_time: f32,
    ) -> SimbaResult<Self> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }

        let controller_instance =
            load_class_from_python_script(config, global_config, initial_time, "Controller")?;
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
        let result = call_py_method!(
            self.controller,
            "make_command",
            CommandWrapper,
            node_py,
            ControllerErrorWrapper::from_rust(error),
            time
        );
        result.to_rust()
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of pre_loop_hook");
        }
        let node_py = NodeWrapper::from_rust(node, self.letter_box_receiver.clone());
        call_py_method_void!(self.controller, "pre_loop_hook", node_py, time);
    }
}

impl Recordable<ControllerRecord> for PythonController {
    fn record(&self) -> ControllerRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = call_py_method!(self.controller, "record", String,);
        let record = PythonControllerRecord {
            record: Value::from_str(&record_str).expect(
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
