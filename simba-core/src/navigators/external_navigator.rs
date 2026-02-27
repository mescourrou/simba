/*!
Module providing the interface to use external [`Navigator`].

To make your own external navigator strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external navigator strategy is made using the
[`PluginAPI::get_navigator`] function.

For the [`Stateful`] trait, the generic type is [`NavigatorRecord`],
and your implementation should return a [`NavigatorRecord::External`]
type. The value inside is a [`serde_json::Value`]. Use [`serde_json::to_value`]
and [`serde_json::from_value`] to make the bridge to your own Record struct.
*/

use log::debug;
use pyo3::{pyclass, pymethods};
use simba_macros::config_derives;
use std::sync::Arc;

use crate::constants::TIME_ROUND;
use crate::controllers::ControllerError;
use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::json_config};
use crate::logger::is_enabled;
use crate::networking::network::Network;
use crate::recordable::Recordable;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::WorldState;
use crate::utils::SharedRwLock;
use crate::utils::macros::{external_config, external_record_python_methods};
use crate::utils::maths::round_precision;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use super::{Navigator, NavigatorRecord};
use serde_derive::{Deserialize, Serialize};

external_config!(
/// Config for the external navigator (generic).
///
/// The config for [`ExternalNavigator`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
///
/// In the yaml file, the config could be:
/// ```YAML
/// navigator:
///     External:
///         parameter_of_my_own_navigator: true
/// ```
    ExternalNavigatorConfig,
    "External Navigator",
    "external-navigator"
);

external_record_python_methods!(
/// Record for the external navigator (generic).
///
/// Like [`ExternalNavigatorConfig`], [`ExternalNavigator`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::record`] implementations.
    ExternalNavigatorRecord,
);

use crate::node::Node;

/// External navigator strategy, which does the bridge with your own strategy.
pub struct ExternalNavigator {
    /// External navigator.
    navigator: Box<dyn Navigator>,
}

impl ExternalNavigator {
    /// Creates a new [`ExternalNavigator`] from the given config.
    ///
    /// <div class="warning">The `plugin_api` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External navigator.
    /// * `plugin_api` -- Required [`PluginAPI`] implementation.
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables
    pub fn from_config(
        config: &ExternalNavigatorConfig,
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
            navigator: plugin_api
                .as_ref()
                .ok_or_else(|| {
                    SimbaError::new(
                        SimbaErrorTypes::ExternalAPIError,
                        "Plugin API not set!".to_string(),
                    )
                })?
                .get_navigator(
                    &config.config,
                    global_config,
                    va_factory,
                    network,
                    initial_time,
                ),
        })
    }
}

impl std::fmt::Debug for ExternalNavigator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalNavigator {{}}")
    }
}

impl Navigator for ExternalNavigator {
    fn post_init(&mut self, node: &mut Node) -> SimbaResult<()> {
        self.navigator.post_init(node)
    }

    fn compute_error(&mut self, robot: &mut Node, world_state: WorldState) -> ControllerError {
        self.navigator.compute_error(robot, world_state)
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        self.navigator.pre_loop_hook(node, time);
    }

    fn next_time_step(&self) -> Option<f32> {
        self.navigator
            .next_time_step()
            .map(|t| round_precision(t, TIME_ROUND).unwrap())
    }
}

impl Recordable<NavigatorRecord> for ExternalNavigator {
    fn record(&self) -> NavigatorRecord {
        self.navigator.record()
    }
}
