//! External sensor fault integration.
//!
//! This module provides the adapter used to load a fault model from an external
//! plugin implementing [`PluginAPI`].
//! It wraps the plugin-provided implementation behind [`FaultModel`]
//! so it can be used transparently by the simulator.

use std::sync::Arc;

use simba_macros::config_derives;

use crate::context::Context;
use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::json_config};
use crate::internal;
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
///   - type: External
///     config:
///       parameter_of_my_own_estimator: true
/// ```
    ExternalFaultConfig,
    "External Fault",
    "external-fault"
);

/// Runtime wrapper around a plugin-provided [`FaultModel`].
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
    /// * `va_factory` -- Factory for Determinists random variables.
    /// * `initial_time` -- Time at which the node using this fault is initialized (can be different from 0 if the node is initialized later in the scenario).
    pub fn from_config(
        config: &ExternalFaultConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
        context: &Context,
    ) -> SimbaResult<Self> {
        internal!(
            context,
            crate::logger::InternalLog::API,
            "Config given: {:?}",
            config
        );
        Ok(Self {
            fault: plugin_api
                .as_ref()
                .ok_or_else(|| {
                    SimbaError::new(
                        SimbaErrorTypes::ExternalAPIError,
                        "Plugin API not set!".to_string(),
                    )
                })?
                .get_sensor_fault(
                    &config.config,
                    global_config,
                    va_factory,
                    initial_time,
                    context,
                ),
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
        context: &Context,
    ) {
        self.fault
            .add_faults(time, seed, obs_list, obs_type, environment, context);
    }

    fn post_init(
        &mut self,
        node: &mut crate::node::Node,
        initial_time: f32,
        context: &Context,
    ) -> SimbaResult<()> {
        self.fault.post_init(node, initial_time, context)
    }
}
