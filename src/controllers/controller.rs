/*!
Module defining the [Controller]
*/

#[cfg(feature = "gui")]
use crate::{
    gui::{utils::string_combobox, UIComponent},
    utils::enum_tools::ToVec,
};
use crate::{
    controllers::python_controller, stateful::Stateful, utils::determinist_random_variable::DeterministRandomVariableFactory
};
use std::sync::{Arc, RwLock};

use crate::{physics::physics::Command, plugin_api::PluginAPI, simulator::SimulatorConfig};

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

#[cfg(feature = "gui")]
impl UIComponent for ControllerError {
    fn show(
            &self,
            ui: &mut egui::Ui,
            ctx: &egui::Context,
            unique_id: &String,
        ) {
        ui.vertical(|ui| {
            ui.label(format!("lateral: {}", self.lateral));
            ui.label(format!("theta: {}", self.theta));
            ui.label(format!("velocity: {}", self.velocity));
        });
    }
}

use super::{external_controller, pid};

/// Enumerates the strategies configurations.
#[derive(Serialize, Deserialize, Debug, Clone, Check, ToVec, EnumToString)]
#[serde(deny_unknown_fields)]
pub enum ControllerConfig {
    PID(pid::PIDConfig),
    External(external_controller::ExternalControllerConfig),
    Python(python_controller::PythonControllerConfig),
}

#[cfg(feature = "gui")]
impl UIComponent for ControllerConfig {
    fn show_mut(
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
                "PID" => *self = ControllerConfig::PID(pid::PIDConfig::default()),
                "External" => {
                    *self = ControllerConfig::External(external_controller::ExternalControllerConfig::default())
                },
                "Python" => {
                    *self = ControllerConfig::Python(python_controller::PythonControllerConfig::default())
                }
                _ => panic!("Where did you find this value?"),
            };
        }
        match self {
            ControllerConfig::PID(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            ControllerConfig::External(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            ControllerConfig::Python(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
        }
    }

    fn show(
        &self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        unique_id: &String,
    ) {
        ui.horizontal(|ui| {
            ui.label(format!("Controller: {}", self.to_string()));
        });
        match self {
            ControllerConfig::PID(c) => c.show(
                ui,
                ctx,
                unique_id,
            ),
            ControllerConfig::External(c) => c.show(
                ui,
                ctx,
                unique_id,
            ),
            ControllerConfig::Python(c) => c.show(
                ui,
                ctx,
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
    Python(python_controller::PythonControllerRecord),
}

#[cfg(feature = "gui")]
impl UIComponent for ControllerRecord {
    fn show(
            &self,
            ui: &mut egui::Ui,
            ctx: &egui::Context,
            unique_id: &String,
        ) {
        ui.vertical(|ui| {
            match self {
                Self::PID(r) => {
                    egui::CollapsingHeader::new("PID").show(ui, |ui| {
                        r.show(ui, ctx, unique_id);
                    });
                },
                Self::External(r) => {
                    egui::CollapsingHeader::new("ExternalController").show(ui, |ui| {
                        r.show(ui, ctx, unique_id);
                    });
                },
                Self::Python(r) => {
                    egui::CollapsingHeader::new("ExternalPythonController").show(ui, |ui| {
                        r.show(ui, ctx, unique_id);
                    });
                },

            }
        });
    }
}

use crate::node::Node;

/// Controller strategy, which compute the [`Command`] to be sent to the
/// [`Physics`](crate::physics::physics::Physics) module, from the given `error`.
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
    /// Command to apply to the [`Physics`](crate::physics::physics::Physics).
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
        },
        ControllerConfig::Python(c) => {
            Box::new(python_controller::PythonController::from_config(
                c,
                plugin_api,
                global_config,
                va_factory,
            ).unwrap()) as Box<dyn Controller>
        }
    }))
}
