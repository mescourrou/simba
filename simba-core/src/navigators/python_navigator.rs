/*!
Module providing the interface to use external Python [`Navigator`].
*/

use std::str::FromStr;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};

use log::debug;
use pyo3::prelude::*;
use pyo3::{Python, pyclass, pymethods};
use serde_json::Value;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::networking::network::Envelope;
use crate::pywrappers::NodeWrapper;
use crate::utils::SharedMutex;
use crate::utils::macros::{external_record_python_methods, python_class_config};
use crate::utils::python::{call_py_method, call_py_method_void, load_class_from_python_script};
use crate::{
    controllers::ControllerError,
    errors::SimbaResult,
    logger::is_enabled,
    navigators::{Navigator, NavigatorRecord},
    pywrappers::{ControllerErrorWrapper, WorldStateWrapper},
    recordable::Recordable,
    simulator::SimulatorConfig,
    state_estimators::WorldState,
};
use serde_derive::{Deserialize, Serialize};

python_class_config!(
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
    PythonNavigatorConfig,
    "External Python Navigator",
    "external-python-navigator"
);

external_record_python_methods!(
/// Record for the external navigator (generic).
///
/// Like [`PythonNavigatorConfig`], [`PythonNavigator`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::record`] implementations.
    PythonNavigatorRecord,
);

use crate::node::Node;

/// External navigator strategy, which does the bridge with your own strategy.
pub struct PythonNavigator {
    /// External navigator.
    navigator: Py<PyAny>,
    letter_box_receiver: SharedMutex<Receiver<Envelope>>,
    letter_box_sender: Sender<Envelope>,
}

impl PythonNavigator {
    /// Creates a new [`PythonNavigator`]
    pub fn new() -> SimbaResult<Self> {
        Self::from_config(
            &PythonNavigatorConfig::default(),
            &SimulatorConfig::default(),
            0.0,
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
        initial_time: f32,
    ) -> SimbaResult<Self> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }

        let navigator_instance =
            load_class_from_python_script(config, global_config, initial_time, "Navigator")?;
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
        let node_py = NodeWrapper::from_rust(node);
        let result = call_py_method!(
            self.navigator,
            "compute_error",
            ControllerErrorWrapper,
            node_py,
            WorldStateWrapper::from_rust(&state)
        );
        result.to_rust()
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of pre_loop_hook");
        }
        let node_py = NodeWrapper::from_rust(node);
        call_py_method_void!(self.navigator, "pre_loop_hook", node_py, time);
    }
}

impl Recordable<NavigatorRecord> for PythonNavigator {
    fn record(&self) -> NavigatorRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str = call_py_method!(self.navigator, "record", String,);
        let record = PythonNavigatorRecord {
            record: Value::from_str(&record_str).expect(
                "Impossible to get serde_json::Value from the input serialized python structure",
            ),
        };
        // record.clone()
        // StateEstimatorRecord::External(PythonNavigator::record(&self))
        NavigatorRecord::Python(record)
    }
}
