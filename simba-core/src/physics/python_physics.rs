/*!
Module providing the interface to use external Python [`Physics`].
*/

use std::ffi::CString;
use std::fs;
use std::str::FromStr;

use config_checker::macros::Check;
use log::{debug, info};
use pyo3::ffi::c_str;
use pyo3::prelude::*;
use pyo3::types::PyModule;
use pyo3::{pyclass, pymethods, PyResult, Python};
use serde_json::Value;

#[cfg(feature = "gui")]
use crate::gui::{utils::json_config, UIComponent};
use crate::physics::robot_models::Command;
use crate::utils::python::ensure_venv_pyo3;
use crate::{
    errors::{SimbaError, SimbaErrorTypes, SimbaResult},
    logger::is_enabled,
    networking::service::HasService,
    physics::{GetRealStateReq, GetRealStateResp, Physics, PhysicsRecord},
    pywrappers::{CommandWrapper, StateWrapper},
    recordable::Recordable,
    simulator::SimulatorConfig,
    state_estimators::State,
};

use serde_derive::{Deserialize, Serialize};

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
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
pub struct PythonPhysicsConfig {
    file: String,
    class_name: String,
    /// Config serialized.
    #[serde(flatten)]
    pub config: Value,
}

impl Default for PythonPhysicsConfig {
    fn default() -> Self {
        Self {
            file: String::new(),
            class_name: String::new(),
            config: Value::Null,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PythonPhysicsConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("External Python Physics").show(ui, |ui| {
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
                    &format!("external-python-physics-key-{}", &unique_id),
                    &format!("external-python-physics-error-key-{}", &unique_id),
                    buffer_stack,
                    &mut self.config,
                );
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        egui::CollapsingHeader::new("External Python Physics").show(ui, |ui| {
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

/// Record for the external physics (generic).
///
/// Like [`PythonPhysicsConfig`], [`PythonPhysics`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::record`] implementations.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass]
pub struct PythonPhysicsRecord {
    /// Record serialized.
    #[serde(flatten)]
    pub record: Value,
}

impl Default for PythonPhysicsRecord {
    fn default() -> Self {
        Self {
            record: Value::Null,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PythonPhysicsRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.label(self.record.to_string());
    }
}

#[pymethods]
impl PythonPhysicsRecord {
    #[getter]
    fn record(&self) -> String {
        self.record.to_string()
    }
}

/// External physics strategy, which does the bridge with your own strategy.
pub struct PythonPhysics {
    /// External physics.
    physics: Py<PyAny>,
}

impl PythonPhysics {
    /// Creates a new [`PythonPhysics`]
    pub fn new() -> SimbaResult<Self> {
        Self::from_config(&PythonPhysicsConfig::default(), &SimulatorConfig::default())
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
    ) -> SimbaResult<Self> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }

        let json_config = serde_json::to_string(&config)
            .expect("Error during converting Python Physics config to json");

        let convert_to_dict = cr#"
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
                        "Python physics script not found ({}): {}",
                        script_path.to_str().unwrap(),
                        e
                    ),
                ))
            }
            Ok(s) => CString::new(s).unwrap(),
        };
        let res = Python::attach(|py| -> PyResult<Py<PyAny>> {
            ensure_venv_pyo3(py)?;

            let script = PyModule::from_code(py, convert_to_dict, c_str!(""), c_str!(""))?;
            let convert_fn: Py<PyAny> = script.getattr("convert")?.into();
            let config_dict = convert_fn.call(py, (json_config,), None)?;

            let script = PyModule::from_code(py, &python_script, c_str!(""), c_str!(""))?;
            let physics_class: Py<PyAny> = script.getattr(config.class_name.as_str())?.into();
            info!("Load Physics class {} ...", config.class_name);

            let res = physics_class.call(py, (config_dict,), None);
            let physics_instance = match res {
                Err(err) => {
                    err.display(py);
                    return Err(err);
                }
                Ok(instance) => instance,
            };
            Ok(physics_instance)
        });
        let physics_instance = match res {
            Err(err) => {
                return Err(SimbaError::new(
                    SimbaErrorTypes::PythonError,
                    err.to_string(),
                ))
            }
            Ok(instance) => instance,
        };
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
    fn apply_command(&mut self, command: &Command, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of apply_command");
        }
        // let robot_record = robot.record();
        Python::attach(|py| {
            if let Err(e) = self.physics.bind(py).call_method(
                "apply_command",
                (CommandWrapper::from_rust(command), time),
                None,
            ) {
                e.display(py);
                panic!("Error while calling 'apply_command' method of PythonPhysics.");
            }
        });
    }

    fn update_state(&mut self, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of update_state");
        }
        // let robot_record = robot.record();
        Python::attach(|py| {
            if let Err(e) = self
                .physics
                .bind(py)
                .call_method("update_state", (time,), None)
            {
                e.display(py);
                panic!("Error while calling 'update_state' method of PythonPhysics.");
            }
        });
    }

    fn state(&self, time: f32) -> State {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of state");
        }
        // let robot_record = robot.record();
        let state = Python::attach(|py| -> StateWrapper {
            match self.physics.bind(py).call_method("state", (time,), None) {
                Err(e) => {
                    e.display(py);
                    panic!("Error while calling 'state' method of PythonPhysics.");
                }
                Ok(s) => s.extract().expect(
                    "The 'state' method of PythonPhysics does not return a correct state vector",
                ),
            }
        });
        state.to_rust()
    }
}

impl Recordable<PhysicsRecord> for PythonPhysics {
    fn record(&self) -> PhysicsRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = Python::attach(|py| {
            match self.physics
                .bind(py)
                .call_method("record", (), None) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error while calling 'record' method of PythonPhysics.");
                    }
                    Ok(r) => {
                        r.extract()
                        .expect("The 'record' method of PythonPhysics does not return a valid PythonPhysicsRecord type")
                    }
                }
        });
        let record = PythonPhysicsRecord {
            record: Value::from_str(record_str.as_str()).expect(
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
