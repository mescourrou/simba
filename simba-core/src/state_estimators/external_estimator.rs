/*!
Module providing the interface to use external [`StateEstimator`].

To make your own external state estimation strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external state estimation strategy is made using the
[`PluginAPI::get_state_estimator`] function.
*/

use std::sync::Arc;

use pyo3::{pyclass, pymethods};
use simba_macros::config_derives;

use super::{StateEstimator, WorldState};
use crate::constants::TIME_ROUND;
use crate::context::Context;
use crate::{context, error, internal};
use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::json_config};
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

use super::StateEstimatorRecord;
use crate::sensors::Observation;
use serde_derive::{Deserialize, Serialize};

external_config!(
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
    ExternalEstimatorConfig,
    "External State Estimator",
    "external-state-estimator"
);

external_record_python_methods!(
/// Record for the external state estimation (generic).
///
/// Like [`ExternalEstimatorConfig`], [`ExternalEstimator`] uses a [`serde_json::Value`]
/// to take every record.
ExternalEstimatorRecord,
);

use crate::node::Node;

/// External estimator strategy, which does the bridge with your own strategy.
pub struct ExternalEstimator {
    /// External state estimator.
    state_estimator: Box<dyn StateEstimator>,
}

impl ExternalEstimator {
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
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        network: &SharedRwLock<Network>,
        initial_time: f32,
        context: &Context,
    ) -> SimbaResult<Self> {
        internal!(context, crate::logger::InternalLog::API, "Config given: {:?}", config);
        Ok(Self {
            state_estimator: plugin_api
                .as_ref()
                .ok_or_else(|| {
                    SimbaError::new(
                        SimbaErrorTypes::ExternalAPIError,
                        "Plugin API not set!".to_string(),
                    )
                })?
                .get_state_estimator(
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

impl std::fmt::Debug for ExternalEstimator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalEstimator {{}}")
    }
}

impl StateEstimator for ExternalEstimator {
    fn post_init(&mut self, node: &mut Node, context: &Context) -> SimbaResult<()> {
        self.state_estimator.post_init(node, context)
    }

    fn prediction_step(&mut self, node: &mut Node, command: Option<Command>, time: f32, context: &Context) {
        if (time - self.next_time_step(context)).abs() > TIME_ROUND / 2. {
            error!(context, "Error trying to update estimate too soon !");
            return;
        }
        self.state_estimator.prediction_step(node, command, time, context);
    }

    fn correction_step(&mut self, node: &mut Node, observations: &[Observation], time: f32, context: &Context) {
        self.state_estimator
            .correction_step(node, observations, time, context);
    }

    fn world_state(&self, context: &Context) -> WorldState {
        self.state_estimator.world_state(context)
    }

    fn next_time_step(&self, context: &Context) -> f32 {
        round_precision(self.state_estimator.next_time_step(context), TIME_ROUND).unwrap()
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32, context: &Context) {
        self.state_estimator.pre_loop_hook(node, time, context);
    }
}

impl Recordable<StateEstimatorRecord> for ExternalEstimator {
    fn record(&self, context: &Context) -> StateEstimatorRecord {
        self.state_estimator.record(context)
    }
}
