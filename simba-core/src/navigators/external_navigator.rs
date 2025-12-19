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

use std::sync::Arc;
use std::sync::mpsc::Sender;

use log::debug;
use pyo3::{pyclass, pymethods};
use simba_macros::config_derives;

use crate::controllers::ControllerError;
#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::json_config};
use crate::logger::is_enabled;
use crate::networking::message_handler::MessageHandler;
use crate::networking::network::Envelope;
use crate::recordable::Recordable;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::WorldState;
use crate::utils::macros::{external_config, external_record_python_methods};
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
    /// Creates a new [`ExternalNavigator`]
    pub fn new() -> Self {
        Self::from_config(
            &ExternalNavigatorConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &Arc::new(DeterministRandomVariableFactory::default()),
            0.0,
        )
    }

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
        initial_time: f32,
    ) -> Self {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }
        Self {
            navigator: plugin_api
                .as_ref()
                .expect("Plugin API not set!")
                .get_navigator(&config.config, global_config, va_factory, initial_time),
        }
    }
}

impl Default for ExternalNavigator {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for ExternalNavigator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalNavigator {{}}")
    }
}

impl Navigator for ExternalNavigator {
    fn compute_error(&mut self, robot: &mut Node, world_state: WorldState) -> ControllerError {
        self.navigator.compute_error(robot, world_state)
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        self.navigator.pre_loop_hook(node, time);
    }
}

impl Recordable<NavigatorRecord> for ExternalNavigator {
    fn record(&self) -> NavigatorRecord {
        self.navigator.record()
    }
}

impl MessageHandler for ExternalNavigator {
    fn get_letter_box(&self) -> Option<Sender<Envelope>> {
        self.navigator.get_letter_box()
    }
}
