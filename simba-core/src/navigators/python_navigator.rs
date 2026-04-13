/*!
Module providing the interface to use external Python [`Navigator`].
*/

use std::str::FromStr;

use pyo3::prelude::*;
use pyo3::{Python, pyclass, pymethods};
use serde_json::Value;

use crate::context::Context;
#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::internal;
use crate::pywrappers::NodeWrapper;
use crate::utils::macros::{external_record_python_methods, python_class_config};
use crate::utils::python::{call_py_method, call_py_method_void, load_class_from_python_script};
use crate::{
    controllers::ControllerError,
    errors::SimbaResult,
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
    PythonNavigatorRecord,
);

use crate::node::Node;

/// External navigator strategy, which does the bridge with your own strategy.
pub struct PythonNavigator {
    /// External navigator.
    navigator: Py<PyAny>,
}

impl PythonNavigator {
    /// Creates a new [`PythonNavigator`]
    pub fn new() -> SimbaResult<Self> {
        Self::from_config(
            &PythonNavigatorConfig::default(),
            &SimulatorConfig::default(),
            0.0,
            &Context::default(),
        )
    }

    /// Creates a new [`PythonNavigator`] from the given config.
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config ] implementation (not used).
    /// * `global_config` -- Simulator config.
    /// * `initial_time` -- Initial time of the node.
    pub fn from_config(
        config: &PythonNavigatorConfig,
        global_config: &SimulatorConfig,
        initial_time: f32,
        context: &Context,
    ) -> SimbaResult<Self> {
        internal!(
            context,
            crate::logger::InternalLog::API,
            "Config given: {:?}",
            config
        );

        let navigator_instance = load_class_from_python_script(
            config,
            global_config,
            initial_time,
            "Navigator",
            context,
        )?;
        Ok(Self {
            navigator: navigator_instance,
        })
    }
}

impl std::fmt::Debug for PythonNavigator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "PythonNavigator {{}}")
    }
}

impl Navigator for PythonNavigator {
    fn post_init(&mut self, node: &mut Node, context: &Context) -> SimbaResult<()> {
        internal!(
            context,
            crate::logger::InternalLog::API,
            "Calling python implementation of post_init"
        );
        let node_py = NodeWrapper::from_rust(node, context.clone());
        call_py_method_void!(self.navigator, "post_init", (node_py,));
        Ok(())
    }

    fn compute_error(
        &mut self,
        node: &mut Node,
        state: WorldState,
        context: &Context,
    ) -> ControllerError {
        internal!(
            context,
            crate::logger::InternalLog::API,
            "Calling python implementation of compute_error"
        );
        let node_py = NodeWrapper::from_rust(node, context.clone());
        let result = call_py_method!(
            self.navigator,
            "compute_error",
            ControllerErrorWrapper,
            node_py,
            WorldStateWrapper::from_rust(&state)
        );
        result.to_rust()
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32, context: &Context) {
        internal!(
            context,
            crate::logger::InternalLog::API,
            "Calling python implementation of pre_loop_hook"
        );
        let node_py = NodeWrapper::from_rust(node, context.clone());
        call_py_method_void!(self.navigator, "pre_loop_hook", node_py, time);
    }

    fn next_time_step(&self, context: &Context) -> Option<f32> {
        internal!(
            context,
            crate::logger::InternalLog::API,
            "Calling python implementation of next_time_step"
        );
        call_py_method!(self.navigator, "next_time_step", Option<f32>,)
    }
}

impl Recordable<NavigatorRecord> for PythonNavigator {
    fn record(&self, context: &Context) -> NavigatorRecord {
        internal!(
            context,
            crate::logger::InternalLog::API,
            "Calling python implementation of record"
        );
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
