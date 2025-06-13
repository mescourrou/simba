/*!
Module providing the interface to use external [`Navigator`].

To make your own external navigator strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external navigator strategy is made using the
[`PluginAPI::get_navigator`] function.

For the [`Stateful`] trait, the generic type is [`NavigatorRecord`],
and your implementation should return a [`NavigatorRecord::External`]
type. The value inside is a [`serde_json::Value`]. Use [`serde_json::to_value`]
and [`serde_json::from_value`] to make the bridge to your own Record struct.
*/

use config_checker::macros::Check;
use pyo3::{pyclass, pymethods};
use rand::distributions::uniform::UniformFloat;
use serde_json::Value;

use crate::controllers::controller::ControllerError;
#[cfg(feature = "gui")]
use crate::gui::{utils::json_config, UIComponent};
use crate::simulator::SimulatorConfig;
use crate::state_estimators::state_estimator::{State, WorldState};
use crate::stateful::Stateful;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use super::navigator::{Navigator, NavigatorRecord};
use serde_derive::{Deserialize, Serialize};

/// Config for the external navigator (generic).
///
/// The config for [`ExternalNavigator`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
///
/// In the yaml file, the config could be:
/// ```YAML
/// navigator:
///     External:
///         parameter_of_my_own_navigator: true
/// ```
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
pub struct ExternalNavigatorConfig {
    /// Config serialized.
    #[serde(flatten)]
    pub config: Value,
}

impl Default for ExternalNavigatorConfig {
    fn default() -> Self {
        Self {
            config: Value::Null,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ExternalNavigatorConfig {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::HashMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new("External Navigator").show(ui, |ui| {
            ui.vertical(|ui| {
                ui.label("Config (JSON):");
                json_config(
                    ui,
                    &format!("external-navigator-key-{}", &unique_id),
                    &format!("external-navigator-error-key-{}", &unique_id),
                    buffer_stack,
                    &mut self.config,
                );
            });
        });
    }
}

/// Record for the external navigator (generic).
///
/// Like [`ExternalNavigatorConfig`], [`ExternalNavigator`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::from_record`] and [`Stateful::record`] implementations.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass]
pub struct ExternalNavigatorRecord {
    /// Record serialized.
    #[serde(flatten)]
    pub record: Value,
}

impl Default for ExternalNavigatorRecord {
    fn default() -> Self {
        Self {
            record: Value::Null,
        }
    }
}

#[pymethods]
impl ExternalNavigatorRecord {
    #[getter]
    fn record(&self) -> String {
        self.record.to_string()
    }
}

use crate::node::Node;

/// External navigator strategy, which does the bridge with your own strategy.
pub struct ExternalNavigator {
    /// External navigator.
    navigator: Box<dyn Navigator>,
}

impl ExternalNavigator {
    /// Creates a new [`ExternalNavigator`]
    pub fn new() -> Self {
        Self::from_config(
            &ExternalNavigatorConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Creates a new [`ExternalNavigator`] from the given config.
    ///
    /// <div class="warning">The `plugin_api` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External navigator.
    /// * `plugin_api` -- Required [`PluginAPI`] implementation.
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables
    pub fn from_config(
        config: &ExternalNavigatorConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        _va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        println!("Config given: {:?}", config);
        Self {
            navigator: plugin_api
                .as_ref()
                .expect("Plugin API not set!")
                .get_navigator(&config.config, global_config),
        }
    }
}

impl std::fmt::Debug for ExternalNavigator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalNavigator {{}}")
    }
}

impl Navigator for ExternalNavigator {
    fn compute_error(&mut self, robot: &mut Node, world_state: WorldState) -> ControllerError {
        self.navigator.compute_error(robot, world_state)
    }
}

impl Stateful<NavigatorRecord> for ExternalNavigator {
    fn record(&self) -> NavigatorRecord {
        self.navigator.record()
    }

    fn from_record(&mut self, record: NavigatorRecord) {
        self.navigator.from_record(record);
    }
}
