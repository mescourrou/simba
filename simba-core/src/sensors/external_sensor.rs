//! External sensor integration.
//!
//! This module provides the bridge allowing simulator users to plug custom sensor
//! implementations through [`PluginAPI`].
//! External sensor creation is delegated to [`PluginAPI::get_sensor`], and observation/record
//! payloads are exchanged through [`serde_json::Value`].

use std::sync::Arc;

use log::debug;
use pyo3::{pyclass, pymethods};
use serde_json::Value;
use simba_macros::config_derives;

use crate::constants::TIME_ROUND;
use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::json_config};
use crate::logger::is_enabled;
use crate::networking::network::Network;
use crate::recordable::Recordable;
use crate::simulator::SimulatorConfig;
use crate::utils::SharedRwLock;
use crate::utils::macros::{external_config, external_record_python_methods};
use crate::utils::maths::round_precision;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use crate::sensors::{Sensor, SensorObservation, SensorRecord};
use serde_derive::{Deserialize, Serialize};

external_record_python_methods!(
/// Record for the external sensor (generic).
ExternalObservationRecord,
);

/// Runtime observation payload emitted by an external sensor.
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct ExternalObservation {
    /// Opaque JSON payload produced by the plugin sensor implementation.
    pub observation: Value,
}

#[cfg(feature = "gui")]
impl UIComponent for ExternalObservation {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        egui::CollapsingHeader::new("External Observation").show(ui, |ui| {
            ui.vertical(|ui| {
                ui.label("Observation (JSON):");
                ui.label(self.observation.to_string());
            });
        });
    }
}

impl Recordable<ExternalObservationRecord> for ExternalObservation {
    fn record(&self) -> ExternalObservationRecord {
        ExternalObservationRecord {
            record: self.observation.clone(),
        }
    }
}

external_config!(
/// Configuration for an external sensor (generic).
///
/// The config for [`ExternalSensor`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
///
/// Default value: `config = serde_json::Value::Null`.
///
/// In the yaml file, the config could be:
/// ```YAML
/// sensors:
///   - type: External
///     config:
///       parameter_of_my_own_sensor: true
/// ```
    ExternalSensorConfig,
    "External Sensor",
    "external-sensor"
);

external_record_python_methods!(
/// Record for the external sensor (generic).
///
/// Like [`ExternalSensorConfig`], [`ExternalSensor`] uses a [`serde_json::Value`]
/// to store records.
///
/// The record is not automatically cast to your own type; conversion should be
/// handled in your plugin-specific serialization/deserialization code.
ExternalSensorRecord,
);

use crate::node::Node;

/// External sensor strategy, which does the bridge with your own strategy.
pub struct ExternalSensor {
    /// External sensor.
    sensor: Box<dyn Sensor>,
}

impl ExternalSensor {
    /// Creates a new [`ExternalSensor`] from the given config.
    ///
    /// <div class="warning">The `plugin_api` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External sensor.
    /// * `plugin_api` -- Required [`PluginAPI`] implementation.
    /// * `global_config` -- Simulator config.
    /// * `va_factory` -- Factory for Determinists random variables.
    /// * `network` -- Reference to the network, to allow the sensor to send messages if needed.
    /// * `initial_time` -- Initial time of the simulation, to allow the sensor to initialize itself with the correct time.
    pub fn from_config(
        config: &ExternalSensorConfig,
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
            sensor: plugin_api
                .as_ref()
                .ok_or_else(|| {
                    SimbaError::new(
                        SimbaErrorTypes::ExternalAPIError,
                        "Plugin API not set!".to_string(),
                    )
                })?
                .get_sensor(&config.config, global_config, va_factory, network, initial_time),
        })
    }
}

impl std::fmt::Debug for ExternalSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalSensor {{}}")
    }
}

impl Sensor for ExternalSensor {
    fn post_init(&mut self, node: &mut Node, initial_time: f32) -> SimbaResult<()> {
        self.sensor.post_init(node, initial_time)
    }

    fn get_observations(&mut self, node: &mut Node, time: f32) -> Vec<SensorObservation> {
        self.sensor.get_observations(node, time)
    }

    fn next_time_step(&self) -> f32 {
        round_precision(self.sensor.next_time_step(), TIME_ROUND)
            .expect("Sensor next_time_step rounding returned an error:")
    }
}

impl Recordable<SensorRecord> for ExternalSensor {
    fn record(&self) -> SensorRecord {
        self.sensor.record()
    }
}
