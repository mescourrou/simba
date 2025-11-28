/*!
Module providing the interface to use external [`Physics`].

To make your own external physic strategy, the simulator should
be used as a library (see [dedicated page](crate::plugin_api)).

Your own external physic strategy is made using the
[`PluginAPI::get_physics`] function.

For the [`Stateful`] trait, the generic type is [`PhysicsRecord`],
and your implementation should return a [`PhysicsRecord::External`]
type. The value inside is a [`serde_json::Value`]. Use [`serde_json::to_value`]
and [`serde_json::from_value`] to make the bridge to your own Record struct.
*/

use std::sync::Arc;

use config_checker::macros::Check;
use log::debug;
use pyo3::{pyclass, pymethods};
use serde_json::Value;

#[cfg(feature = "gui")]
use crate::gui::{utils::json_config, UIComponent};
use crate::logger::is_enabled;
use crate::networking::service::HasService;
use crate::physics::robot_models::Command;
use crate::recordable::Recordable;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::State;
use crate::utils::macros::external_record_python_methods;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use serde_derive::{Deserialize, Serialize};

/// Config for the external physics (generic).
///
/// The config for [`ExternalPhysics`] uses a [`serde_json::Value`] to
/// integrate your own configuration inside the full simulator config.
///
/// In the yaml file, the config could be:
/// ```YAML
/// physics:
///     External:
///         parameter_of_my_own_physics: true
/// ```
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
pub struct ExternalPhysicsConfig {
    /// Config serialized.
    #[serde(flatten)]
    pub config: Value,
}

impl Default for ExternalPhysicsConfig {
    fn default() -> Self {
        Self {
            config: Value::Null,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ExternalPhysicsConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
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

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        egui::CollapsingHeader::new("External Physics").show(ui, |ui| {
            ui.vertical(|ui| {
                ui.label("Config (JSON):");
                ui.label(self.config.to_string());
            });
        });
    }
}

external_record_python_methods!(
/// Record for the external physics (generic).
///
/// Like [`ExternalPhysicsConfig`], [`ExternalPhysics`] uses a [`serde_json::Value`]
/// to take every record.
///
/// The record is not automatically cast to your own type, the cast should be done
/// in [`Stateful::record`] implementations.
ExternalPhysicsRecord,
);

use super::{GetRealStateReq, GetRealStateResp, Physics, PhysicsRecord};

/// External physics strategy, which does the bridge with your own strategy.
pub struct ExternalPhysics {
    /// External physics.
    physics: Box<dyn Physics>,
}

impl ExternalPhysics {
    /// Creates a new [`ExternalPhysics`]
    pub fn new() -> Self {
        Self::from_config(
            &ExternalPhysicsConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &Arc::new(DeterministRandomVariableFactory::default()),
        )
    }

    /// Creates a new [`ExternalPhysics`] from the given config.
    ///
    /// <div class="warning">The `plugin_api` is required here !</div>
    ///
    ///  ## Arguments
    /// * `config` -- Scenario config of the External physics.
    /// * `plugin_api` -- Required [`PluginAPI`] implementation.
    /// * `global_config` -- Simulator config.
    /// * `_va_factory` -- Factory for Determinists random variables
    pub fn from_config(
        config: &ExternalPhysicsConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Self {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Config given: {:?}", config);
        }
        Self {
            physics: plugin_api
                .as_ref()
                .expect("Plugin API not set!")
                .get_physics(&config.config, global_config, va_factory),
        }
    }
}

impl Default for ExternalPhysics {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for ExternalPhysics {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalPhysics {{}}")
    }
}

impl Physics for ExternalPhysics {
    fn apply_command(&mut self, command: &Command, time: f32) {
        self.physics.apply_command(command, time);
    }

    fn state(&self, time: f32) -> State {
        self.physics.state(time).clone()
    }

    fn update_state(&mut self, time: f32) {
        self.physics.update_state(time);
    }
}

impl Recordable<PhysicsRecord> for ExternalPhysics {
    fn record(&self) -> PhysicsRecord {
        self.physics.record()
    }
}

impl HasService<GetRealStateReq, GetRealStateResp> for ExternalPhysics {
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
