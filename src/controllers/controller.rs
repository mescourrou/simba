/*!
Module defining the [Controller]
*/

#[cfg(feature = "gui")]
use crate::gui::{utils::string_combobox, UIComponent};
use crate::{
    stateful::Stateful,
    utils::{determinist_random_variable::DeterministRandomVariableFactory, enum_tools::ToVec},
};
use std::sync::{Arc, RwLock};

use crate::{physics::physic::Command, plugin_api::PluginAPI, simulator::SimulatorConfig};

use config_checker::macros::Check;
use serde_derive::{Deserialize, Serialize};
use simba_macros::{EnumToString, ToVec};

/// Errors used by the controllers: lateral, orientation and velocity.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ControllerError {
    /// Lateral error.
    pub lateral: f32,
    /// Orientation error.
    pub theta: f32,
    /// Velocity error.
    pub velocity: f32,
}

impl ControllerError {
    pub fn default() -> Self {
        Self {
            lateral: 0.,
            theta: 0.,
            velocity: 0.,
        }
    }
}

use super::{external_controller, pid};

/// Enumerates the strategies configurations.
#[derive(Serialize, Deserialize, Debug, Clone, Check, ToVec, EnumToString)]
#[serde(deny_unknown_fields)]
pub enum ControllerConfig {
    PID(Box<pid::PIDConfig>),
    External(Box<external_controller::ExternalControllerConfig>),
}

#[cfg(feature = "gui")]
impl UIComponent for ControllerConfig {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        let mut current_str = self.to_string();
        ui.horizontal(|ui| {
            ui.label("Controller:");
            string_combobox(
                ui,
                &ControllerConfig::to_vec()
                    .iter()
                    .map(|x| String::from(*x))
                    .collect(),
                &mut current_str,
                format!("controller-choice-{}", unique_id),
            );
        });
        if current_str != self.to_string() {
            match current_str.as_str() {
                "PID" => *self = ControllerConfig::PID(Box::new(pid::PIDConfig::default())),
                "External" => {
                    *self = ControllerConfig::External(Box::new(
                        external_controller::ExternalControllerConfig::default(),
                    ))
                }
                _ => panic!("Where did you find this value?"),
            };
        }
        match self {
            ControllerConfig::PID(c) => c.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            ControllerConfig::External(c) => c.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
        }
    }
}

/// Enumerates the strategies records.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum ControllerRecord {
    PID(pid::PIDRecord),
    External(external_controller::ExternalControllerRecord),
}

use crate::node::Node;

/// Controller strategy, which compute the [`Command`] to be sent to the
/// [`Physic`](crate::physics::physic::Physic) module, from the given `error`.
pub trait Controller:
    std::fmt::Debug + std::marker::Send + std::marker::Sync + Stateful<ControllerRecord>
{
    /// Compute the command from the given error.
    ///
    /// ## Arguments
    /// * `robot` - Reference to the robot to access modules.
    /// * `error` - Error to be corrected.
    /// * `time` - Current time.
    ///
    /// ## Return
    /// Command to apply to the [`Physic`](crate::physics::physic::Physic).
    fn make_command(&mut self, robot: &mut Node, error: &ControllerError, time: f32) -> Command;
}

/// Helper function to make the right [`Controller`] from the given configuration.
///
/// ## Arguments
/// * `config` - Configuration to use to make the controller.
/// * `plugin_api` - Optional PluginAPI to transmit to the controller.
/// * `meta_config` - Meta configuration of the simulator.
/// * `va_factory` - Random variables factory for determinist behavior.
pub fn make_controller_from_config(
    config: &ControllerConfig,
    plugin_api: &Option<Box<&dyn PluginAPI>>,
    global_config: &SimulatorConfig,
    va_factory: &DeterministRandomVariableFactory,
) -> Arc<RwLock<Box<dyn Controller>>> {
    Arc::new(RwLock::new(match config {
        ControllerConfig::PID(c) => Box::new(pid::PID::from_config(
            c,
            plugin_api,
            global_config,
            va_factory,
        )) as Box<dyn Controller>,
        ControllerConfig::External(c) => {
            Box::new(external_controller::ExternalController::from_config(
                c,
                plugin_api,
                global_config,
                va_factory,
            )) as Box<dyn Controller>
        }
    }))
}
