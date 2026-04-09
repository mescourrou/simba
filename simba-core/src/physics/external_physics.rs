/*!
Module providing the interface to use external [`Physics`].

To make your own external physic strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external physic strategy is made using the
[`PluginAPI::get_physics`] function.

*/

use std::sync::Arc;

use pyo3::{pyclass, pymethods};
use simba_macros::config_derives;

use crate::constants::TIME_ROUND;
use crate::context::{self, Context};
use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::json_config};
use crate::internal;
use crate::networking::network::Network;
use crate::physics::robot_models::Command;
use crate::recordable::Recordable;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::State;
use crate::utils::SharedRwLock;
use crate::utils::macros::{external_config, external_record_python_methods};
use crate::utils::maths::round_precision;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

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

use super::{GetRealStateReq, GetRealStateResp, Physics, PhysicsRecord};

/// External physics strategy, which does the bridge with your own strategy.
pub struct ExternalPhysics {
    /// External physics.
    physics: Box<dyn Physics>,
}

impl ExternalPhysics {
    /// Creates a new [`ExternalPhysics`] from the given config.
    ///
    /// <div class="warning">The `plugin_api` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External physics.
    /// * `plugin_api` -- Required [`PluginAPI`] implementation.
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables
    pub fn from_config(
        config: &ExternalPhysicsConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        network: &SharedRwLock<Network>,
        initial_time: f32,
        context: &Context,
    ) -> SimbaResult<Self> {
        internal!(context, crate::logger::InternalLog::API, "Config given: {:?}", config);
        Ok(Self {
            physics: plugin_api
                .as_ref()
                .ok_or_else(|| {
                    SimbaError::new(
                        SimbaErrorTypes::ExternalAPIError,
                        "Plugin API not set!".to_string(),
                    )
                })?
                .get_physics(
                    &config.config,
                    global_config,
                    va_factory,
                    network,
                    initial_time,
                    context,
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