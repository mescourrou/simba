/*!
Module providing the interface to use external [`Physic`].

To make your own external physic strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external physic strategy is made using the
[`PluginAPI::get_physic`] function.

For the [`Stateful`] trait, the generic type is [`PhysicRecord`],
and your implementation should return a [`PhysicRecord::External`]
type. The value inside is a [`serde_json::Value`]. Use [`serde_json::to_value`]
and [`serde_json::from_value`] to make the bridge to your own Record struct.
*/

use config_checker::macros::Check;
use log::debug;
use pyo3::{pyclass, pymethods};
use serde_json::Value;

#[cfg(feature = "gui")]
use crate::gui::{
    utils::json_config,
    UIComponent,
};
use crate::logger::is_enabled;
use crate::networking::service::HasService;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::state_estimator::State;
use crate::stateful::Stateful;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use serde_derive::{Deserialize, Serialize};

/// Config for the external physic (generic).
///
/// The config for [`ExternalPhysic`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
///
/// In the yaml file, the config could be:
/// ```YAML
/// physic:
///     External:
///         parameter_of_my_own_physic: true
/// ```
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
pub struct ExternalPhysicConfig {
    /// Config serialized.
    #[serde(flatten)]
    pub config: Value,
}

impl Default for ExternalPhysicConfig {
    fn default() -> Self {
        Self {
            config: Value::Null,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ExternalPhysicConfig {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::HashMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new("External Physics").show(ui, |ui| {
            ui.vertical(|ui| {
                ui.label("Config (JSON):");
                json_config(
                    ui,
                    &format!("external-physics-key-{}", &unique_id),
                    &format!("external-physics-error-key-{}", &unique_id),
                    buffer_stack,
                    &mut self.config,
                );
            });
        });
    }
}

/// Record for the external physic (generic).
///
/// Like [`ExternalPhysicConfig`], [`ExternalPhysic`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::from_record`] and [`Stateful::record`] implementations.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass]
pub struct ExternalPhysicRecord {
    /// Record serialized.
    #[serde(flatten)]
    pub record: Value,
}

impl Default for ExternalPhysicRecord {
    fn default() -> Self {
        Self {
            record: Value::Null,
        }
    }
}

#[pymethods]
impl ExternalPhysicRecord {
    #[getter]
    fn record(&self) -> String {
        self.record.to_string()
    }
}

use super::physic::{Command, GetRealStateReq, GetRealStateResp, Physic, PhysicRecord};

/// External physic strategy, which does the bridge with your own strategy.
pub struct ExternalPhysic {
    /// External physic.
    physic: Box<dyn Physic>,
}

impl ExternalPhysic {
    /// Creates a new [`ExternalPhysic`]
    pub fn new() -> Self {
        Self::from_config(
            &ExternalPhysicConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Creates a new [`ExternalPhysic`] from the given config.
    ///
    /// <div class="warning">The `plugin_api` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External physic.
    /// * `plugin_api` -- Required [`PluginAPI`] implementation.
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables
    pub fn from_config(
        config: &ExternalPhysicConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        _va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }
        Self {
            physic: plugin_api
                .as_ref()
                .expect("Plugin API not set!")
                .get_physic(&config.config, global_config),
        }
    }
}

impl std::fmt::Debug for ExternalPhysic {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalPhysic {{}}")
    }
}

impl Physic for ExternalPhysic {
    fn apply_command(&mut self, command: &Command, time: f32) {
        self.physic.apply_command(command, time);
    }

    fn state(&self, time: f32) -> &State {
        self.physic.state(time)
    }

    fn update_state(&mut self, time: f32) {
        self.physic.update_state(time);
    }
}

impl Stateful<PhysicRecord> for ExternalPhysic {
    fn record(&self) -> PhysicRecord {
        self.physic.record()
    }

    fn from_record(&mut self, record: PhysicRecord) {
        self.physic.from_record(record);
    }
}

impl HasService<GetRealStateReq, GetRealStateResp> for ExternalPhysic {
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
