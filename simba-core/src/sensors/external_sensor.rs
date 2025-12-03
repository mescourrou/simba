/*!
Module providing the interface to use external [`Sensor`].

To make your own external sensor strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external sensor strategy is made using the
[`PluginAPI::get_sensor`] function.

For the [`Stateful`] trait, the generic type is [`SensorRecord`],
and your implementation should return a [`SensorRecord::External`]
type. The value inside is a [`serde_json::Value`]. Use [`serde_json::to_value`]
and [`serde_json::from_value`] to make the bridge to your own Record struct.
*/

use std::sync::Arc;

use config_checker::macros::Check;
use log::debug;
use pyo3::{pyclass, pymethods};
use serde_json::Value;
use simba_macros::config_derives;

use crate::constants::TIME_ROUND;
#[cfg(feature = "gui")]
use crate::gui::{utils::json_config, UIComponent};
use crate::logger::is_enabled;
use crate::recordable::Recordable;
use crate::simulator::SimulatorConfig;
use crate::utils::macros::{external_config, external_record_python_methods};
use crate::utils::maths::round_precision;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use crate::sensors::{Observation, ObservationRecord, Sensor, SensorObservation, SensorObservationRecord, SensorRecord};
use serde_derive::{Deserialize, Serialize};

external_record_python_methods!(
/// Record for the external sensor (generic).
ExternalObservationRecord,
);

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct ExternalObservation {
    pub observation: Value,
}

#[cfg(feature = "gui")]
impl UIComponent for ExternalObservation {
    fn show(
            &self,
            ui: &mut egui::Ui,
            _ctx: &egui::Context,
            _unique_id: &str,
        ) {
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
    ExternalSensorConfig,
    "External Sensor",
    "external-sensor"
);

external_record_python_methods!(
/// Record for the external sensor (generic).
///
/// Like [`ExternalSensorConfig`], [`ExternalSensor`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::from_record`] and [`Stateful::record`] implementations.
ExternalSensorRecord,
);

use crate::node::Node;

/// External sensor strategy, which does the bridge with your own strategy.
pub struct ExternalSensor {
    /// External sensor.
    sensor: Box<dyn Sensor>,
}

impl ExternalSensor {
    /// Creates a new [`ExternalSensor`]
    pub fn new() -> Self {
        Self::from_config(
            &ExternalSensorConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Creates a new [`ExternalSensor`] from the given config.
    ///
    /// <div class="warning">The `plugin_api` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External sensor.
    /// * `plugin_api` -- Required [`PluginAPI`] implementation.
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables.
    pub fn from_config(
        config: &ExternalSensorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        _va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }
        Self {
            sensor: plugin_api
                .as_ref()
                .expect("Plugin API not set!")
                .get_sensor(&config.config, global_config),
        }
    }
}

impl std::fmt::Debug for ExternalSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalSensor {{}}")
    }
}

impl Sensor for ExternalSensor {
    fn init(&mut self, node: &mut Node) {
        self.sensor.init(node);
    }

    fn get_observations(&mut self, node: &mut Node, time: f32) -> Vec<SensorObservation> {
        self.sensor.get_observations(node, time)
    }

    fn next_time_step(&self) -> f32 {
        round_precision(self.sensor.next_time_step(), TIME_ROUND).expect("Sensor next_time_step rounding returned an error:")
    }

}

impl Recordable<SensorRecord> for ExternalSensor {
    fn record(&self) -> SensorRecord {
        self.sensor.record()
    }
}
