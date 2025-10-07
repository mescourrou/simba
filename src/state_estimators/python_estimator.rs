/*!
Module providing the interface to use external Python [`StateEstimator`].
*/

use std::fs;
use std::str::FromStr;

use config_checker::macros::Check;
use log::{debug, info};
use pyo3::types::PyModule;
use pyo3::{prepare_freethreaded_python, pyclass, pymethods, PyResult, Python};
use pyo3::prelude::*;
use serde_json::Value;

use super::state_estimator::{StateEstimator, WorldState};
use crate::constants::TIME_ROUND;
use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
#[cfg(feature = "gui")]
use crate::gui::{utils::json_config, UIComponent};
use crate::logger::is_enabled;
use crate::pywrappers::{ObservationWrapper, WorldStateWrapper};
use crate::simulator::SimulatorConfig;
use crate::stateful::Stateful;
use crate::utils::maths::round_precision;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use super::state_estimator::StateEstimatorRecord;
use crate::sensors::sensor::Observation;
use serde_derive::{Deserialize, Serialize};

/// Config for the external state estimation (generic).
///
/// The config for [`PythonEstimator`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
/// 
/// You need to provide the path of the script containing the state estimator.
///
/// In the yaml file, the config could be:
/// ```YAML
/// state_estimator:
///     Python:
///         file: ""../my_python_script.py"
///         class_name: MyStateEstimator
///         parameter_of_my_own_estimator: true
/// ```
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
pub struct PythonEstimatorConfig {
    file: String,
    class_name: String,
    /// Config serialized.
    #[serde(flatten)]
    pub config: Value,
}

impl Default for PythonEstimatorConfig {
    fn default() -> Self {
        Self {
            file: String::new(),
            class_name: String::new(),
            config: Value::Null,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PythonEstimatorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new("External Python State Estimator").show(ui, |ui| {
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
                    &format!("external-python-state-estimator-key-{}", &unique_id),
                    &format!("external-python-state-estimator-error-key-{}", &unique_id),
                    buffer_stack,
                    &mut self.config,
                );
            });
        });
    }

    fn show(
        &self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new("External Python State Estimator").show(ui, |ui| {
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

/// Record for the external state estimation (generic).
///
/// Like [`PythonEstimatorConfig`], [`PythonEstimator`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::from_record`] and [`Stateful::record`] implementations.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass]
pub struct PythonEstimatorRecord {
    /// Record serialized.
    #[serde(flatten)]
    pub record: Value,
}

impl Default for PythonEstimatorRecord {
    fn default() -> Self {
        Self {
            record: Value::Null,
        }
    }
}


#[cfg(feature = "gui")]
impl UIComponent for PythonEstimatorRecord {
    fn show(
            &self,
            ui: &mut egui::Ui,
            ctx: &egui::Context,
            unique_id: &String,
        ) {
        ui.label(self.record.to_string());
    }
}

#[pymethods]
impl PythonEstimatorRecord {
    #[getter]
    fn record(&self) -> String {
        self.record.to_string()
    }
}

use crate::node::Node;

/// External estimator strategy, which does the bridge with your own strategy.
pub struct PythonEstimator {
    /// External state estimator.
    state_estimator: Py<PyAny>,
}

impl PythonEstimator {
    /// Creates a new [`PythonEstimator`]
    pub fn new() -> SimbaResult<Self> {
        Self::from_config(
            &PythonEstimatorConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Creates a new [`PythonEstimator`] from the given config.
    ///
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External estimator.
    /// * `plugin_api` -- [`PluginAPI`] implementation (not used).
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables.
    pub fn from_config(
        config: &PythonEstimatorConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        _va_factory: &DeterministRandomVariableFactory,
    ) -> SimbaResult<Self> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }

        // prepare_freethreaded_python();

        let json_config =
            serde_json::to_string(&config).expect("Error during converting Python State Estimator config to json");

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

        let script_path = global_config
            .base_path
            .as_ref()
            .join(&config.file);
        let python_script = match fs::read_to_string(script_path.clone()) {
            Err(e) => {
                return Err(SimbaError::new(
                    SimbaErrorTypes::ConfigError,
                    format!(
                        "Python state estimator script not found ({}): {}",
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
            let state_estimator_class: Py<PyAny> = script.getattr(config.class_name.as_str())?.into();
            info!("Load State Estimator class {} ...", config.class_name);
            
            let res = state_estimator_class.call_bound(
                py,
                (config_dict,),
                None,
            );
            let state_estimator_instance = match res {
                Err(err) => {
                    err.display(py);
                    return Err(err);
                },
                Ok(instance) => instance,
            };
            Ok(state_estimator_instance)
        });
        let state_estimator_instance = match res {
            Err(err) => 
                return Err(SimbaError::new(
                    SimbaErrorTypes::PythonError,
                    err.to_string(),
                )),
            Ok(instance) => instance,
        };
        Ok(Self { state_estimator: state_estimator_instance })
    }
}

impl std::fmt::Debug for PythonEstimator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "PythonEstimator {{}}")
    }
}

impl StateEstimator for PythonEstimator {
    fn prediction_step(&mut self, _node: &mut Node, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of prediction_step");
        }
        // let node_record = node.record();
        Python::with_gil(|py| {
            if let Err(e) = self.state_estimator
                .bind(py)
                .call_method("prediction_step", (time,), None) {
                    e.display(py);
                    panic!("Error while calling 'prediction_step' method of PythonEstimator.");
                }
        });
    }

    fn correction_step(&mut self, _node: &mut Node, observations: &Vec<Observation>, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of correction_step");
        }
        let mut observation_py = Vec::new();
        for obs in observations {
            observation_py.push(ObservationWrapper::from_rust(obs));
        }
        Python::with_gil(|py| {
            if let Err(e) = self.state_estimator
                .bind(py)
                .call_method("correction_step", (observation_py, time), None) {
                    e.display(py);
                    panic!("Error while calling 'correction_step' method of PythonEstimator.");
                }
        });
    }

    fn world_state(&self) -> WorldState {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of state");
        }
        let state = Python::with_gil(|py| -> WorldStateWrapper {
            match self.state_estimator
                .bind(py)
                .call_method("state", (), None) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error while calling 'state' method of PythonEstimator.");
                    }
                    Ok(r) => {
                        r.extract()
                        .expect("The 'state' method of PythonEstimator does not return a correct state vector")
                    }
                }
        });
        state.to_rust()
    }

    fn next_time_step(&self) -> f32 {
        // PythonStateEstimator::next_time_step(self)
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of next_time_step");
        }
        let time = Python::with_gil(|py| {
            match self.state_estimator
                .bind(py)
                .call_method("next_time_step", (), None) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error while calling 'next_time_step' method of PythonEstimator.");
                    }
                    Ok(r) => {
                        r.extract()
                        .expect("The 'next_time_step' method of PythonEstimator does not return a correct time for next step")
                    }
                }
        });
        round_precision(time, TIME_ROUND).unwrap()
    }

}

impl Stateful<StateEstimatorRecord> for PythonEstimator {
    fn record(&self) -> StateEstimatorRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = Python::with_gil(|py| {
            match self.state_estimator
                .bind(py)
                .call_method("record", (), None) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error while calling 'record' method of PythonEstimator.");
                    }
                    Ok(r) => {
                        r.extract()
                        .expect("The 'record' method of PythonEstimator does not return a valid EstimatorRecord type")
                    }
                }
        });
        let record = PythonEstimatorRecord {
            record: Value::from_str(record_str.as_str()).expect(
                "Impossible to get serde_json::Value from the input serialized python structure",
            ),
        };
        StateEstimatorRecord::Python(record)
    }
    
    fn from_record(&mut self, record: StateEstimatorRecord) {
        if let StateEstimatorRecord::Python(record) = record {
            if is_enabled(crate::logger::InternalLog::API) {
                debug!("Calling python implementation of from_record");
            }
            Python::with_gil(|py| {
                if let Err(e) = self.state_estimator
                    .bind(py)
                    .call_method("from_record", (serde_json::to_string(&record).unwrap(),), None) {
                        e.display(py);
                        panic!("Error while calling 'from_record' method of PythonEstimator.");
                    }
            });
        }
    }
}
