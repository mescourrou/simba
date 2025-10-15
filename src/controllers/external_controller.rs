/*!
Module providing the interface to use external [`Controller`].

To make your own external controller strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external controller strategy is made using the
[`PluginAPI::get_controller`] function.

For the [`Stateful`] trait, the generic type is [`ControllerRecord`],
and your implementation should return a [`ControllerRecord::External`]
type. The value inside is a [`serde_json::Value`]. Use [`serde_json::to_value`]
and [`serde_json::from_value`] to make the bridge to your own Record struct.
*/

use config_checker::macros::Check;
use log::debug;
use pyo3::{pyclass, pymethods};
use serde_json::Value;

#[cfg(feature = "gui")]
use crate::gui::{utils::json_config, UIComponent};
use crate::logger::is_enabled;
use crate::physics::physics::Command;
use crate::recordable::Recordable;
use crate::simulator::SimulatorConfig;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use super::controller::{Controller, ControllerError, ControllerRecord};
use serde_derive::{Deserialize, Serialize};

/// Config for the external controller (generic).
///
/// The config for [`ExternalController`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
///
/// In the yaml file, the config could be:
/// ```YAML
/// controller:
///     External:
///         parameter_of_my_own_controller: true
/// ```
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
pub struct ExternalControllerConfig {
    /// Config serialized.
    #[serde(flatten)]
    pub config: Value,
}

impl Default for ExternalControllerConfig {
    fn default() -> Self {
        Self {
            config: Value::Null,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ExternalControllerConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new("External Controller").show(ui, |ui| {
            ui.vertical(|ui| {
                ui.label("Config (JSON):");
                json_config(
                    ui,
                    &format!("external-controller-key-{}", &unique_id),
                    &format!("external-controller-error-key-{}", &unique_id),
                    buffer_stack,
                    &mut self.config,
                );
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &String) {
        egui::CollapsingHeader::new("External Controller").show(ui, |ui| {
            ui.vertical(|ui| {
                ui.label("Config (JSON):");
                ui.label(self.config.to_string());
            });
        });
    }
}

/// Record for the external controller (generic).
///
/// Like [`ExternalControllerConfig`], [`ExternalController`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::record`] implementations.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass]
pub struct ExternalControllerRecord {
    /// Record serialized.
    #[serde(flatten)]
    pub record: Value,
}

impl Default for ExternalControllerRecord {
    fn default() -> Self {
        Self {
            record: Value::Null,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ExternalControllerRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &String) {
        ui.label(self.record.to_string());
    }
}

#[pymethods]
impl ExternalControllerRecord {
    #[getter]
    fn record(&self) -> String {
        self.record.to_string()
    }
}

use crate::node::Node;

/// External controller strategy, which does the bridge with your own strategy.
pub struct ExternalController {
    /// External controller.
    controller: Box<dyn Controller>,
}

impl ExternalController {
    /// Creates a new [`ExternalController`]
    pub fn new() -> Self {
        Self::from_config(
            &ExternalControllerConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Creates a new [`ExternalController`] from the given config.
    ///
    /// <div class="warning">The `plugin_api` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External controller.
    /// * `plugin_api` -- Required [`PluginAPI`] implementation.
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables
    pub fn from_config(
        config: &ExternalControllerConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        _va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }
        Self {
            controller: plugin_api
                .as_ref()
                .expect("Plugin API not set!")
                .get_controller(&config.config, global_config),
        }
    }
}

impl std::fmt::Debug for ExternalController {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalController {{}}")
    }
}

impl Controller for ExternalController {
    fn make_command(&mut self, robot: &mut Node, error: &ControllerError, time: f32) -> Command {
        self.controller.make_command(robot, error, time)
    }
}

impl Recordable<ControllerRecord> for ExternalController {
    fn record(&self) -> ControllerRecord {
        self.controller.record()
    }
}
