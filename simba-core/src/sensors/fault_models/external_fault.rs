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
use crate::sensors::fault_models::fault_model::FaultModel;
use crate::simulator::SimulatorConfig;
use crate::utils::macros::external_config;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use crate::sensors::SensorObservation;

external_config!(
/// Config for the external sensor fault (generic).
///
/// The config for [`ExternalFault`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
///
/// In the yaml file, the config could be:
/// ```YAML
/// faults:
///     - type: External
///       parameter_of_my_own_estimator: true
/// ```
    ExternalFaultConfig,
    "External Fault",
    "external-fault"
);

pub struct ExternalFault {
    fault: Box<dyn FaultModel>,
}

impl ExternalFault {
    /// Creates a new [`ExternalFault`] from the given config.
    ///
    /// <div class="warning">The `plugin_api` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External fault.
    /// * `plugin_api` -- Required [`PluginAPI`] implementation.
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables.
    pub fn from_config(
        config: &ExternalFaultConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> SimbaResult<Self> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }
        Ok(Self {
            fault: plugin_api
                .as_ref()
                .ok_or_else(|| {
                    SimbaError::new(
                        SimbaErrorTypes::ExternalAPIError,
                        "Plugin API not set!".to_string(),
                    )
                })?
                .get_sensor_fault(&config.config, global_config, va_factory, initial_time),
        })
    }
}

impl std::fmt::Debug for ExternalFault {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalFault {{}}")
    }
}

impl FaultModel for ExternalFault {
    fn add_faults(
        &mut self,
        time: f32,
        seed: f32,
        obs_list: &mut Vec<SensorObservation>,
        obs_type: SensorObservation,
        environment: &Arc<crate::environment::Environment>,
    ) {
        self.fault
            .add_faults(time, seed, obs_list, obs_type, environment);
    }

    fn post_init(&mut self, node: &mut crate::node::Node, initial_time: f32) -> SimbaResult<()> {
        self.fault.post_init(node, initial_time)
    }
}
