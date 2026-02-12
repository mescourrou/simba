/*!
Module providing the interface to use external [`Physics`].

To make your own external physic strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external physic strategy is made using the
[`PluginAPI::get_physics`] function.

For the [`Stateful`] trait, the generic type is [`PhysicsRecord`],
and your implementation should return a [`PhysicsRecord::External`]
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
use crate::networking::service::HasService;
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
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::record`] implementations.
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
    ) -> SimbaResult<Self> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }
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
    fn apply_command(&mut self, command: &Command, time: f32) {
        self.physics.apply_command(command, time);
    }

    fn state(&self, time: f32) -> State {
        self.physics.state(time).clone()
    }

    fn update_state(&mut self, time: f32) {
        self.physics.update_state(time);
    }

    fn next_time_step(&self) -> Option<f32> {
        self.physics
            .next_time_step()
            .map(|t| round_precision(t, TIME_ROUND).unwrap())
    }
}

impl Recordable<PhysicsRecord> for ExternalPhysics {
    fn record(&self) -> PhysicsRecord {
        self.physics.record()
    }
}

impl HasService<GetRealStateReq, GetRealStateResp> for ExternalPhysics {
    fn handle_service_requests(
        &mut self,
        _req: GetRealStateReq,
        time: f32,
    ) -> Result<GetRealStateResp, String> {
        Ok(GetRealStateResp {
            state: self.state(time).clone(),
        })
    }
}
