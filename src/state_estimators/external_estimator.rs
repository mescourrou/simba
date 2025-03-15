/*!
Module providing the interface to use external [`StateEstimator`].

To make your own external state estimation strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external state estimation strategy is made using the
[`PluginAPI::get_state_estimator`] function.

For the [`Stateful`] trait, the generic type is [`StateEstimatorRecord`],
and your implementation should return a [`StateEstimatorRecord::External`]
type. The value inside is a [`serde_json::Value`]. Use [`serde_json::to_value`]
and [`serde_json::from_value`] to make the bridge to your own Record struct.
*/

use config_checker::macros::Check;
use pyo3::{pyclass, pymethods};
use serde_json::Value;

use super::state_estimator::{State, StateEstimator};
use crate::simulator::SimulatorConfig;
use crate::stateful::Stateful;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use super::state_estimator::StateEstimatorRecord;
use crate::sensors::sensor::Observation;
use serde_derive::{Deserialize, Serialize};

/// Config for the external state estimation (generic).
///
/// The config for [`ExternalEstimator`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
///
/// In the yaml file, the config could be:
/// ```YAML
/// state_estimator:
///     External:
///         parameter_of_my_own_estimator: true
/// ```
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
pub struct ExternalEstimatorConfig {
    /// Config serialized.
    #[serde(flatten)]
    pub config: Value,
}

impl Default for ExternalEstimatorConfig {
    fn default() -> Self {
        Self {
            config: Value::Null,
        }
    }
}

/// Record for the external state estimation (generic).
///
/// Like [`ExternalEstimatorConfig`], [`ExternalEstimator`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::from_record`] and [`Stateful::record`] implementations.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass]
pub struct ExternalEstimatorRecord {
    /// Record serialized.
    #[serde(flatten)]
    pub record: Value,
}

impl Default for ExternalEstimatorRecord {
    fn default() -> Self {
        Self {
            record: Value::Null,
        }
    }
}

#[pymethods]
impl ExternalEstimatorRecord {
    #[getter]
    fn record(&self) -> String {
        self.record.to_string()
    }
}

use crate::robot::Robot;

/// External estimator strategy, which does the bridge with your own strategy.
pub struct ExternalEstimator {
    /// External state estimator.
    state_estimator: Box<dyn StateEstimator>,
}

impl ExternalEstimator {
    /// Creates a new [`ExternalEstimator`]
    pub fn new() -> Self {
        Self::from_config(
            &ExternalEstimatorConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Creates a new [`ExternalEstimator`] from the given config.
    ///
    /// <div class="warning">The `plugin_api` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External estimator.
    /// * `plugin_api` -- Required [`PluginAPI`] implementation.
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables.
    pub fn from_config(
        config: &ExternalEstimatorConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        _va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        println!("Config given: {:?}", config);
        Self {
            state_estimator: plugin_api
                .as_ref()
                .expect("Plugin API not set!")
                .get_state_estimator(&config.config, global_config),
        }
    }
}

impl std::fmt::Debug for ExternalEstimator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalEstimator {{}}")
    }
}

impl StateEstimator for ExternalEstimator {
    fn prediction_step(&mut self, robot: &mut Robot, time: f32) {
        if time < self.next_time_step() {
            println!("Error trying to update estimate too soon !");
            return;
        }
        self.state_estimator.prediction_step(robot, time);
    }

    fn correction_step(&mut self, robot: &mut Robot, observations: &Vec<Observation>, time: f32) {
        self.state_estimator
            .correction_step(robot, observations, time);
    }

    fn state(&self) -> State {
        self.state_estimator.state()
    }

    fn next_time_step(&self) -> f32 {
        self.state_estimator.next_time_step()
    }
}

impl Stateful<StateEstimatorRecord> for ExternalEstimator {
    fn record(&self) -> StateEstimatorRecord {
        self.state_estimator.record()
    }

    fn from_record(&mut self, record: StateEstimatorRecord) {
        self.state_estimator.from_record(record);
    }
}
