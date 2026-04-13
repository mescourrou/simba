/*!
Module providing the interface to use external [`Physics`].

To make your own external physic strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external physic strategy is made using the
[`crate::plugin_api::PluginAPI::get_physics`] function.

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
use crate::state_estimators::State;
use crate::utils::macros::{external_config, external_record_python_methods};
use crate::utils::maths::round_precision;

use serde_derive::{Deserialize, Serialize};

external_config!(
/// Config for the external physics (generic).
///
/// The config for [`ExternalPhysics`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
///
/// In the yaml file, the config could be:
/// ```YAML
/// physics:
///     External:
///         parameter_of_my_own_physics: true
/// ```
    ExternalPhysicsConfig,
    "External Physics",
    "external-physics"
);

external_record_python_methods!(
/// Record for the external physics (generic).
///
/// Like [`ExternalPhysicsConfig`], [`ExternalPhysics`] uses a [`serde_json::Value`]
/// to take every record.
ExternalPhysicsRecord,
);

use super::{Physics, PhysicsRecord};

/// External physics strategy, which does the bridge with your own strategy.
pub struct ExternalPhysics {
    /// External physics.
    physics: Box<dyn Physics>,
}

impl ExternalPhysics {
    /// Creates a new [`ExternalPhysics`] from the given config.
    ///
    /// <div class="warning">The `plugin_api` in `from_config_params` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External physics.
    /// * `from_config_params` -- Parameters required to create the physics from config, including the required `plugin_api`.
    pub fn from_config(
        config: &ExternalPhysicsConfig,
        from_config_params: &FromConfigArguments,
    ) -> SimbaResult<Self> {
        internal!(
            from_config_params.context,
            crate::logger::InternalLog::API,
            "Config given: {:?}",
            config
        );
        Ok(Self {
            physics: from_config_params
                .plugin_api
                .as_ref()
                .ok_or_else(|| {
                    SimbaError::new(
                        SimbaErrorTypes::ExternalAPIError,
                        "Plugin API not set!".to_string(),
                    )
                })?
                .get_physics(
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

impl std::fmt::Debug for ExternalPhysics {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalPhysics {{}}")
    }
}

impl Physics for ExternalPhysics {
    fn post_init(&mut self, node: &mut crate::node::Node, context: &Context) -> SimbaResult<()> {
        self.physics.post_init(node, context)
    }

    fn apply_command(&mut self, command: &Command, time: f32, context: &Context) {
        self.physics.apply_command(command, time, context);
    }

    fn state(&self, time: f32, context: &Context) -> State {
        self.physics.state(time, context).clone()
    }

    fn update_state(&mut self, time: f32, context: &Context) {
        self.physics.update_state(time, context);
    }

    fn next_time_step(&self, context: &Context) -> Option<f32> {
        self.physics
            .next_time_step(context)
            .map(|t| round_precision(t, TIME_ROUND).unwrap())
    }
}

impl Recordable<PhysicsRecord> for ExternalPhysics {
    fn record(&self, context: &Context) -> PhysicsRecord {
        self.physics.record(context)
    }
}
