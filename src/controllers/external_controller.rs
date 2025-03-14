/*!
Module providing the interface to use external [`Controller`].

To make your own external controller strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external controller strategy is made using the
[`PluginAPI::get_controller`] function.

For the [`Stateful`] trait, the generic type is [`ControllerRecord`],
and your implementation should return a [`ControllerRecord::External`]
type. The value inside is a [`serde_json::Value`]. Use [`serde_json::to_value`]
and [`serde_json::from_value`] to make the bridge to your own Record struct.
*/

use config_checker::macros::Check;
use pyo3::{pyclass, pymethods};
use serde_json::Value;

use crate::physics::physic::Command;
use crate::simulator::SimulatorConfig;
use crate::stateful::Stateful;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use super::controller::{Controller, ControllerError, ControllerRecord};
use serde_derive::{Deserialize, Serialize};

/// Config for the external controller (generic).
///
/// The config for [`ExternalController`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
///
/// In the yaml file, the config could be:
/// ```YAML
/// controller:
///     External:
///         parameter_of_my_own_controller: true
/// ```
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
pub struct ExternalControllerConfig {
    /// Config serialized.
    #[serde(flatten)]
    pub config: Value,
}

impl Default for ExternalControllerConfig {
    fn default() -> Self {
        Self {
            config: Value::Null,
        }
    }
}

/// Record for the external controller (generic).
///
/// Like [`ExternalControllerConfig`], [`ExternalController`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::from_record`] and [`Stateful::record`] implementations.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass]
pub struct ExternalControllerRecord {
    /// Record serialized.
    #[serde(flatten)]
    pub record: Value,
}

impl Default for ExternalControllerRecord {
    fn default() -> Self {
        Self {
            record: Value::Null,
        }
    }
}

#[pymethods]
impl ExternalControllerRecord {
    #[getter]
    fn record(&self) -> String {
        self.record.to_string()
    }
}

use crate::robot::Robot;

/// External controller strategy, which does the bridge with your own strategy.
pub struct ExternalController {
    /// External controller.
    controller: Box<dyn Controller>,
}

impl ExternalController {
    /// Creates a new [`ExternalController`]
    pub fn new() -> Self {
        Self::from_config(
            &ExternalControllerConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Creates a new [`ExternalController`] from the given config.
    ///
    /// <div class="warning">The `plugin_api` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External controller.
    /// * `plugin_api` -- Required [`PluginAPI`] implementation.
    /// * `meta_config` -- Simulator config.
    pub fn from_config(
        config: &ExternalControllerConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        _va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        println!("Config given: {:?}", config);
        Self {
            controller: plugin_api
                .as_ref()
                .expect("Plugin API not set!")
                .get_controller(&config.config, global_config),
        }
    }
}

impl std::fmt::Debug for ExternalController {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalController {{}}")
    }
}

impl Controller for ExternalController {
    fn make_command(&mut self, robot: &mut Robot, error: &ControllerError, time: f32) -> Command {
        self.controller.make_command(robot, error, time)
    }
}

impl Stateful<ControllerRecord> for ExternalController {
    fn record(&self) -> ControllerRecord {
        self.controller.record()
    }

    fn from_record(&mut self, record: ControllerRecord) {
        self.controller.from_record(record);
    }
}
