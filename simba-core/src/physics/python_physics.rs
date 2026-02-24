/*!
Module providing the interface to use external Python [`Physics`].
*/

use std::str::FromStr;

use log::debug;
use pyo3::prelude::*;
use pyo3::{Python, pyclass, pymethods};
use serde_json::Value;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::physics::robot_models::Command;
use crate::pywrappers::NodeWrapper;
use crate::utils::macros::{external_record_python_methods, python_class_config};
use crate::utils::python::{call_py_method, call_py_method_void, load_class_from_python_script};
use crate::{
    errors::SimbaResult,
    logger::is_enabled,
    networking::service::HasService,
    physics::{GetRealStateReq, GetRealStateResp, Physics, PhysicsRecord},
    pywrappers::{CommandWrapper, StateWrapper},
    recordable::Recordable,
    simulator::SimulatorConfig,
    state_estimators::State,
};

use serde_derive::{Deserialize, Serialize};

python_class_config!(
/// Config for the external physics (generic).
///
/// The config for [`PythonPhysics`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
///
/// You need to provide the path of the script containing the physics.
///
/// In the yaml file, the config could be:
/// ```YAML
/// physics:
///     Python:
///         file: ""../my_python_script.py"
///         class_name: MyPhysics
///         parameter_of_my_own_physics: true
/// ```
    PythonPhysicsConfig,
    "External Python Physics",
    "external-python-physics"
);

external_record_python_methods!(
/// Record for the external physics (generic).
///
/// Like [`PythonPhysicsConfig`], [`PythonPhysics`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::record`] implementations.
PythonPhysicsRecord,
);

/// External physics strategy, which does the bridge with your own strategy.
pub struct PythonPhysics {
    /// External physics.
    physics: Py<PyAny>,
}

impl PythonPhysics {
    /// Creates a new [`PythonPhysics`]
    pub fn new() -> SimbaResult<Self> {
        Self::from_config(
            &PythonPhysicsConfig::default(),
            &SimulatorConfig::default(),
            0.0,
        )
    }

    /// Creates a new [`PythonPhysics`] from the given config.
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External estimator.
    /// * `plugin_api` -- [`PluginAPI`] implementation (not used).
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables.
    pub fn from_config(
        config: &PythonPhysicsConfig,
        global_config: &SimulatorConfig,
        initial_time: f32,
    ) -> SimbaResult<Self> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }

        let physics_instance =
            load_class_from_python_script(config, global_config, initial_time, "Physics")?;
        Ok(Self {
            physics: physics_instance,
        })
    }
}

impl std::fmt::Debug for PythonPhysics {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "PythonPhysics {{}}")
    }
}

impl Physics for PythonPhysics {
    fn post_init(&mut self, node: &mut crate::node::Node) -> SimbaResult<()> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of post_init");
        }
        call_py_method_void!(self.physics, "post_init", (NodeWrapper::from_rust(node),));
        Ok(())
    }

    fn apply_command(&mut self, command: &Command, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of apply_command");
        }
        // let robot_record = robot.record();
        call_py_method_void!(
            self.physics,
            "apply_command",
            CommandWrapper::from_rust(command),
            time
        );
    }

    fn update_state(&mut self, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of update_state");
        }
        call_py_method_void!(self.physics, "update_state", (time,));
    }

    fn state(&self, time: f32) -> State {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of state");
        }
        let state = call_py_method!(self.physics, "state", StateWrapper, (time,));
        state.to_rust()
    }

    fn next_time_step(&self) -> Option<f32> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of next_time_step");
        }
        call_py_method!(self.physics, "next_time_step", Option<f32>,)
    }
}

impl Recordable<PhysicsRecord> for PythonPhysics {
    fn record(&self) -> PhysicsRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = call_py_method!(self.physics, "record", String,);
        let record = PythonPhysicsRecord {
            record: Value::from_str(&record_str).expect(
                "Impossible to get serde_json::Value from the input serialized python structure",
            ),
        };
        // record.clone()
        // StateEstimatorRecord::External(PythonPhysics::record(&self))
        PhysicsRecord::Python(record)
    }
}

impl HasService<GetRealStateReq, GetRealStateResp> for PythonPhysics {
    fn handle_service_requests(
        &mut self,
        _req: GetRealStateReq,
        time: f32,
    ) -> Result<GetRealStateResp, String> {
        Ok(GetRealStateResp {
            state: self.state(time),
        })
    }
}
