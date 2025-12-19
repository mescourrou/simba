/*!
Module providing the interface to use external [`StateEstimator`].

To make your own external state estimation strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external state estimation strategy is made using the
[`PluginAPI::get_state_estimator`] function.

For the [`Stateful`] trait, the generic type is [`StateEstimatorRecord`],
and your implementation should return a [`StateEstimatorRecord::External`]
type. The value inside is a [`serde_json::Value`]. Use [`serde_json::to_value`]
and [`serde_json::from_value`] to make the bridge to your own Record struct.
*/

use std::sync::Arc;

use log::debug;
use pyo3::{pyclass, pymethods};
use simba_macros::config_derives;

use super::{StateEstimator, WorldState};
use crate::constants::TIME_ROUND;
#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::json_config};
use crate::logger::is_enabled;
use crate::networking::message_handler::MessageHandler;
use crate::networking::network::Envelope;
use crate::recordable::Recordable;
use crate::simulator::SimulatorConfig;
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
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::record`] implementations.
ExternalEstimatorRecord,
);

use crate::node::Node;

/// External estimator strategy, which does the bridge with your own strategy.
pub struct ExternalEstimator {
    /// External state estimator.
    state_estimator: Box<dyn StateEstimator>,
}

impl ExternalEstimator {
    /// Creates a new [`ExternalEstimator`]
    pub fn new() -> Self {
        Self::from_config(
            &ExternalEstimatorConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &Arc::new(DeterministRandomVariableFactory::default()),
            0.0,
        )
    }

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
        initial_time: f32,
    ) -> Self {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }
        Self {
            state_estimator: plugin_api
                .as_ref()
                .expect("Plugin API not set!")
                .get_state_estimator(&config.config, global_config, va_factory, initial_time),
        }
    }
}

impl Default for ExternalEstimator {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for ExternalEstimator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalEstimator {{}}")
    }
}

impl StateEstimator for ExternalEstimator {
    fn prediction_step(&mut self, node: &mut Node, time: f32) {
        if (time - self.next_time_step()).abs() > TIME_ROUND / 2. {
            println!("Error trying to update estimate too soon !");
            return;
        }
        self.state_estimator.prediction_step(node, time);
    }

    fn correction_step(&mut self, node: &mut Node, observations: &[Observation], time: f32) {
        self.state_estimator
            .correction_step(node, observations, time);
    }

    fn world_state(&self) -> WorldState {
        self.state_estimator.world_state()
    }

    fn next_time_step(&self) -> f32 {
        round_precision(self.state_estimator.next_time_step(), TIME_ROUND).unwrap()
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        self.state_estimator.pre_loop_hook(node, time);
    }
}

impl Recordable<StateEstimatorRecord> for ExternalEstimator {
    fn record(&self) -> StateEstimatorRecord {
        self.state_estimator.record()
    }
}

impl MessageHandler for ExternalEstimator {
    fn get_letter_box(&self) -> Option<std::sync::mpsc::Sender<Envelope>> {
        self.state_estimator.get_letter_box()
    }
}
