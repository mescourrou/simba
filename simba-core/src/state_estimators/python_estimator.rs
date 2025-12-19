/*!
Module providing the interface to use external Python [`StateEstimator`].
*/

use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};

use log::debug;
use pyo3::prelude::*;
use pyo3::{Python, pyclass, pymethods};

use super::{StateEstimator, WorldState};
use crate::constants::TIME_ROUND;
use crate::errors::SimbaResult;
#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::logger::is_enabled;
use crate::networking::message_handler::MessageHandler;
use crate::networking::network::Envelope;
use crate::pywrappers::{NodeWrapper, ObservationWrapper, WorldStateWrapper};
use crate::recordable::Recordable;
use crate::simulator::SimulatorConfig;
use crate::utils::macros::{external_record_python_methods, python_class_config};
use crate::utils::maths::round_precision;
use crate::utils::python::{call_py_method, call_py_method_void, load_class_from_python_script};

use super::StateEstimatorRecord;
use crate::sensors::Observation;
use serde_derive::{Deserialize, Serialize};

python_class_config!(
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
    PythonEstimatorConfig,
    "External Python State Estimator",
    "external-python-state-estimator"
);

external_record_python_methods!(
/// Record for the external state estimation (generic).
///
/// Like [`PythonEstimatorConfig`], [`PythonEstimator`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::record`] implementations.
PythonEstimatorRecord,
);

use crate::node::Node;

/// External estimator strategy, which does the bridge with your own strategy.
pub struct PythonEstimator {
    /// External state estimator.
    state_estimator: Py<PyAny>,
    letter_box_receiver: Arc<Mutex<Receiver<Envelope>>>,
    letter_box_sender: Sender<Envelope>,
}

impl PythonEstimator {
    /// Creates a new [`PythonEstimator`]
    pub fn new() -> SimbaResult<Self> {
        Self::from_config(
            &PythonEstimatorConfig::default(),
            &SimulatorConfig::default(),
            0.0,
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
        global_config: &SimulatorConfig,
        initial_time: f32,
    ) -> SimbaResult<Self> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }

        let state_estimator_instance =
            load_class_from_python_script(config, global_config, initial_time, "State Estimator")?;
        let (tx, rx) = mpsc::channel();
        Ok(Self {
            state_estimator: state_estimator_instance,
            letter_box_receiver: Arc::new(Mutex::new(rx)),
            letter_box_sender: tx,
        })
    }
}

impl std::fmt::Debug for PythonEstimator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "PythonEstimator {{}}")
    }
}

impl StateEstimator for PythonEstimator {
    fn prediction_step(&mut self, node: &mut Node, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of prediction_step");
        }
        let node_py = NodeWrapper::from_rust(node, self.letter_box_receiver.clone());
        call_py_method_void!(self.state_estimator, "prediction_step", node_py, time);
    }

    fn correction_step(&mut self, node: &mut Node, observations: &[Observation], time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of correction_step");
        }
        let mut observation_py = Vec::new();
        for obs in observations {
            observation_py.push(ObservationWrapper::from_rust(obs));
        }
        let node_py = NodeWrapper::from_rust(node, self.letter_box_receiver.clone());
        call_py_method_void!(
            self.state_estimator,
            "correction_step",
            node_py,
            observation_py,
            time
        );
    }

    fn world_state(&self) -> WorldState {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of state");
        }
        let state = call_py_method!(self.state_estimator, "state", WorldStateWrapper,);
        state.to_rust()
    }

    fn next_time_step(&self) -> f32 {
        // PythonStateEstimator::next_time_step(self)
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of next_time_step");
        }
        let time = call_py_method!(self.state_estimator, "next_time_step", f32,);
        round_precision(time, TIME_ROUND).unwrap()
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of pre_loop_hook");
        }
        let node_py = NodeWrapper::from_rust(node, self.letter_box_receiver.clone());
        call_py_method_void!(self.state_estimator, "pre_loop_hook", node_py, time);
    }
}

impl Recordable<StateEstimatorRecord> for PythonEstimator {
    fn record(&self) -> StateEstimatorRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = Python::attach(|py| {
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
            record: serde_json::from_str(&record_str).expect(
                "Impossible to get serde_json::Value from the input serialized python structure",
            ),
        };
        StateEstimatorRecord::Python(record)
    }
}

impl MessageHandler for PythonEstimator {
    fn get_letter_box(&self) -> Option<Sender<Envelope>> {
        Some(self.letter_box_sender.clone())
    }
}
