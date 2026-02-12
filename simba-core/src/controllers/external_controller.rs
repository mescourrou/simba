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

use std::sync::Arc;

use log::debug;
use pyo3::{pyclass, pymethods};
use simba_macros::config_derives;

use crate::constants::TIME_ROUND;
use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::json_config};
use crate::logger::is_enabled;
use crate::networking::network::Network;
use crate::physics::robot_models::Command;
use crate::recordable::Recordable;
use crate::simulator::SimulatorConfig;
use crate::utils::SharedRwLock;
use crate::utils::macros::{external_config, external_record_python_methods};
use crate::utils::maths::round_precision;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use super::{Controller, ControllerError, ControllerRecord};
use serde_derive::{Deserialize, Serialize};

external_config!(
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
    ExternalControllerConfig,
    "External Controller",
    "external-controller"
);

external_record_python_methods!(
/// Record for the external controller (generic).
///
/// Like [`ExternalControllerConfig`], [`ExternalController`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::record`] implementations.
    ExternalControllerRecord,
);

use crate::node::Node;

/// External controller strategy, which does the bridge with your own strategy.
pub struct ExternalController {
    /// External controller.
    controller: Box<dyn Controller>,
}

impl ExternalController {
    /// Creates a new [`ExternalController`] from the given config.
    ///
    /// <div class="warning">The `plugin_api` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External controller.
    /// * `plugin_api` -- Required [`PluginAPI`] implementation.
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables
    pub fn from_config(
        config: &ExternalControllerConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        network: &SharedRwLock<Network>,
        initial_time: f32,
    ) -> SimbaResult<Self> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }
        Ok(Self {
            controller: plugin_api
                .as_ref()
                .ok_or_else(|| {
                    SimbaError::new(
                        SimbaErrorTypes::ExternalAPIError,
                        "Plugin API not set!".to_string(),
                    )
                })?
                .get_controller(
                    &config.config,
                    global_config,
                    va_factory,
                    network,
                    initial_time,
                ),
        })
    }
}

impl std::fmt::Debug for ExternalController {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalController {{}}")
    }
}

impl Controller for ExternalController {
    fn make_command(&mut self, robot: &mut Node, error: &ControllerError, time: f32) -> Command {
        self.controller.make_command(robot, error, time)
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        self.controller.pre_loop_hook(node, time);
    }

    fn next_time_step(&self) -> Option<f32> {
        self.controller
            .next_time_step()
            .map(|t| round_precision(t, TIME_ROUND).unwrap())
    }
}

impl Recordable<ControllerRecord> for ExternalController {
    fn record(&self) -> ControllerRecord {
        self.controller.record()
    }
}
