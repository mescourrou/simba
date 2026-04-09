/*!
Module providing the interface to use external Python [`Controller`].
*/

use std::str::FromStr;

use pyo3::prelude::*;
use pyo3::{Python, pyclass, pymethods};
use serde_json::Value;

use crate::context::Context;
#[cfg(feature = "gui")]
use crate::gui::UIComponent;

use crate::internal;
use crate::physics::robot_models::Command;
use crate::pywrappers::NodeWrapper;
use crate::utils::macros::{external_record_python_methods, python_class_config};
use crate::utils::python::{call_py_method, call_py_method_void, load_class_from_python_script};
use crate::{
    controllers::{Controller, ControllerError, ControllerRecord},
    errors::SimbaResult,
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
PythonControllerRecord,
);

use crate::node::Node;

/// External controller strategy, which does the bridge with your own strategy.
pub struct PythonController {
    /// External controller.
    controller: Py<PyAny>,
}

impl PythonController {
    /// Creates a new [`PythonController`]
    pub fn new() -> SimbaResult<Self> {
        Self::from_config(
            &PythonControllerConfig::default(),
            &SimulatorConfig::default(),
            0.0,
            &Context::default(),
        )
    }

    /// Creates a new [`PythonController`] from the given config.
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External estimator.
    /// * `global_config` -- Simulator config.
    /// * `initial_time` -- Initial time of the node.
    /// * `context` -- Execution context.
    pub fn from_config(
        config: &PythonControllerConfig,
        global_config: &SimulatorConfig,
        initial_time: f32,
        context: &Context,
    ) -> SimbaResult<Self> {
        internal!(context, crate::logger::InternalLog::API, "Config given: {:?}", config);

        let controller_instance =
            load_class_from_python_script(config, global_config, initial_time, "Controller", context)?;
        Ok(Self {
            controller: controller_instance,
        })
    }
}

impl std::fmt::Debug for PythonController {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "PythonController {{}}")
    }
}

impl Controller for PythonController {
    fn post_init(&mut self, node: &mut Node, context: &Context) -> SimbaResult<()> {
        internal!(context, crate::logger::InternalLog::API, "Calling python implementation of post_init");
        let py_node = NodeWrapper::from_rust(node, context.clone());
        call_py_method_void!(self.controller, "post_init", (py_node,));
        Ok(())
    }

    fn make_command(&mut self, node: &mut Node, error: &ControllerError, time: f32, context: &Context) -> Command {
        internal!(context, crate::logger::InternalLog::API, "Calling python implementation of make_command");
        let node_py = NodeWrapper::from_rust(node, context.clone());
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

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32, context: &Context) {
        internal!(context, crate::logger::InternalLog::API, "Calling python implementation of pre_loop_hook");
        let node_py = NodeWrapper::from_rust(node, context.clone());
        call_py_method_void!(self.controller, "pre_loop_hook", node_py, time);
    }

    fn next_time_step(&self, context: &Context) -> Option<f32> {
        internal!(context, crate::logger::InternalLog::API, "Calling python implementation of next_time_step");
        call_py_method!(self.controller, "next_time_step", Option<f32>,)
    }
}

impl Recordable<ControllerRecord> for PythonController {
    fn record(&self, context: &Context) -> ControllerRecord {
        internal!(context, crate::logger::InternalLog::API, "Calling python implementation of record");
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
