/*!
Module providing the interface to use external [`Navigator`].

To make your own external navigator strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external navigator strategy is made using the
[`crate::plugin_api::PluginAPI::get_navigator`] function.
*/

use pyo3::{pyclass, pymethods};
use simba_macros::config_derives;

use crate::constants::TIME_ROUND;
use crate::context::Context;
use crate::controllers::ControllerError;
use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::json_config};
use crate::internal;
use crate::node::node_factory::FromConfigArguments;
use crate::recordable::Recordable;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::WorldState;
use crate::utils::macros::{external_config, external_record_python_methods};
use crate::utils::maths::round_precision;

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
///   type: External:
///   config:
///     parameter_of_my_own_navigator: true
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
    /// <div class="warning">The `plugin_api` in `from_config_params` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External navigator.
    /// * `from_config_params` -- Parameters required to create the navigator from config, including the required `plugin_api`.
    pub fn from_config(
        config: &ExternalNavigatorConfig,
        from_config_params: &FromConfigArguments,
    ) -> SimbaResult<Self> {
        internal!(
            from_config_params.context,
            crate::logger::InternalLog::API,
            "Config given: {:?}",
            config
        );
        Ok(Self {
            navigator: from_config_params
                .plugin_api
                .as_ref()
                .ok_or_else(|| {
                    SimbaError::new(
                        SimbaErrorTypes::ExternalAPIError,
                        "Plugin API not set!".to_string(),
                    )
                })?
                .get_navigator(
                    &config.config,
                    from_config_params.global_config,
                    from_config_params.va_factory,
                    from_config_params.network,
                    from_config_params.initial_time,
                    from_config_params.context,
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
    fn post_init(&mut self, node: &mut Node, context: &Context) -> SimbaResult<()> {
        self.navigator.post_init(node, context)
    }

    fn compute_error(
        &mut self,
        robot: &mut Node,
        world_state: WorldState,
        context: &Context,
    ) -> ControllerError {
        self.navigator.compute_error(robot, world_state, context)
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32, context: &Context) {
        self.navigator.pre_loop_hook(node, time, context);
    }

    fn next_time_step(&self, context: &Context) -> Option<f32> {
        self.navigator
            .next_time_step(context)
            .map(|t| round_precision(t, TIME_ROUND).unwrap())
    }
}

impl Recordable<NavigatorRecord> for ExternalNavigator {
    fn record(&self, context: &Context) -> NavigatorRecord {
        self.navigator.record(context)
    }
}
