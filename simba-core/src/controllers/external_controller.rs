/*!
Module providing the interface to use external [`Controller`].

To make your own external controller strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external controller strategy is made using the
[`crate::plugin_api::PluginAPI::get_controller`] function.
*/

use pyo3::{pyclass, pymethods};
use simba_macros::config_derives;

use crate::constants::TIME_ROUND;
use crate::context::Context;
use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::json_config};
use crate::internal;
use crate::node::node_factory::FromConfigArguments;
use crate::physics::robot_models::Command;
use crate::recordable::Recordable;
use crate::simulator::SimulatorConfig;
use crate::utils::macros::{external_config, external_record_python_methods};
use crate::utils::maths::round_precision;

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
    /// <div class="warning">The `plugin_api` in `from_config_params` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External controller.
    /// * `from_config_params` -- Parameters required to create the controller from config, including the required `plugin_api`
    pub fn from_config(
        config: &ExternalControllerConfig,
        from_config_params: &FromConfigArguments,
    ) -> SimbaResult<Self> {
        internal!(
            from_config_params.context,
            crate::logger::InternalLog::API,
            "Config given: {:?}",
            config
        );
        Ok(Self {
            controller: from_config_params
                .plugin_api
                .as_ref()
                .ok_or_else(|| {
                    SimbaError::new(
                        SimbaErrorTypes::ExternalAPIError,
                        "Plugin API not set!".to_string(),
                    )
                })?
                .get_controller(
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

impl std::fmt::Debug for ExternalController {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalController {{}}")
    }
}

impl Controller for ExternalController {
    fn post_init(&mut self, node: &mut Node, context: &Context) -> SimbaResult<()> {
        self.controller.post_init(node, context)
    }

    fn make_command(
        &mut self,
        robot: &mut Node,
        error: &ControllerError,
        time: f32,
        context: &Context,
    ) -> Command {
        self.controller.make_command(robot, error, time, context)
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32, context: &Context) {
        self.controller.pre_loop_hook(node, time, context);
    }

    fn next_time_step(&self, context: &Context) -> Option<f32> {
        self.controller
            .next_time_step(context)
            .map(|t| round_precision(t, TIME_ROUND).unwrap())
    }
}

impl Recordable<ControllerRecord> for ExternalController {
    fn record(&self, context: &Context) -> ControllerRecord {
        self.controller.record(context)
    }
}
