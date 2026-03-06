/*!
Module providing the interface to use external [`SensorFilter`].

To make your own external sensor filter strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external sensor filter strategy is made using the
[`PluginAPI::get_sensor_filter`] function.
*/

use std::sync::Arc;

use log::debug;
use simba_macros::config_derives;

use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::json_config};
use crate::logger::is_enabled;
use crate::networking::network::Network;
use crate::sensors::sensor_filters::SensorFilter;
use crate::simulator::SimulatorConfig;
use crate::utils::SharedRwLock;
use crate::utils::macros::external_config;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use crate::sensors::SensorObservation;

external_config!(
/// Config for the external sensor filter (generic).
///
/// The config for [`ExternalFilter`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
///
/// In the yaml file, the config could be:
/// ```YAML
/// filters:
///     - type: External
///       parameter_of_my_own_estimator: true
/// ```
    ExternalFilterConfig,
    "External Filter",
    "external-filter"
);

/// External sensor strategy, which does the bridge with your own strategy.
pub struct ExternalFilter {
    /// External sensor.
    filter: Box<dyn SensorFilter>,
}

impl ExternalFilter {
    /// Creates a new [`ExternalFilter`] from the given config.
    ///
    /// <div class="warning">The `plugin_api` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External filter.
    /// * `plugin_api` -- Required [`PluginAPI`] implementation.
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables.
    pub fn from_config(
        config: &ExternalFilterConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> SimbaResult<Self> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }
        Ok(Self {
            filter: plugin_api
                .as_ref()
                .ok_or_else(|| {
                    SimbaError::new(
                        SimbaErrorTypes::ExternalAPIError,
                        "Plugin API not set!".to_string(),
                    )
                })?
                .get_sensor_filter(&config.config, global_config, va_factory, initial_time),
        })
    }
}

impl std::fmt::Debug for ExternalFilter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalFilter {{}}")
    }
}

impl SensorFilter for ExternalFilter {
    fn post_init(&mut self, node: &mut crate::node::Node, initial_time: f32) -> SimbaResult<()> {
        self.filter.post_init(node, initial_time)
    }

    fn filter(
        &self,
        time: f32,
        observation: SensorObservation,
        observer_state: &crate::state_estimators::State,
        observee_state: Option<&crate::state_estimators::State>,
    ) -> Option<SensorObservation> {
        self.filter
            .filter(time, observation, observer_state, observee_state)
    }
}
