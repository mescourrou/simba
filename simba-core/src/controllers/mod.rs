/*!
Module providing the [`Controller`](controller::Controller) strategy, which computes the
[`Command`](crate::physics::physics::Command) sent to the [`Physics`](crate::physics::physics::Physics).
*/
pub mod external_controller;
pub mod pid;
pub mod python_controller;

pub mod pybinds;

use crate::{
    errors::SimbaResult,
    networking::message_handler::MessageHandler,
    physics::{PhysicsConfig, robot_models::Command},
    recordable::Recordable,
    utils::{SharedRwLock, determinist_random_variable::DeterministRandomVariableFactory},
};
#[cfg(feature = "gui")]
use crate::{
    gui::{UIComponent, utils::string_combobox},
    utils::enum_tools::ToVec,
};
use std::sync::{Arc, RwLock};

use crate::{plugin_api::PluginAPI, simulator::SimulatorConfig};

use serde_derive::{Deserialize, Serialize};
use simba_macros::config_derives;

/// Errors used by the controllers: lateral, orientation and velocity.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ControllerError {
    /// Lateral error.
    pub lateral: f32,
    /// Longitudinal error.
    pub longitudinal: f32,
    /// Orientation error.
    pub theta: f32,
    /// Velocity error.
    pub velocity: f32,
}

#[cfg(feature = "gui")]
impl UIComponent for ControllerError {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!("longitudinal: {}", self.longitudinal));
            ui.label(format!("lateral: {}", self.lateral));
            ui.label(format!("theta: {}", self.theta));
            ui.label(format!("velocity: {}", self.velocity));
        });
    }
}

/// Enumerates the strategies configurations.
#[config_derives]
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
        unique_id: &str,
    ) {
        let mut current_str = self.to_string();
        ui.horizontal(|ui| {
            ui.label("Controller:");
            string_combobox(
                ui,
                &ControllerConfig::to_vec()
                    .iter()
                    .map(|x: &&str| String::from(*x))
                    .collect(),
                &mut current_str,
                format!("controller-choice-{}", unique_id),
            );
        });
        if current_str != self.to_string() {
            match current_str.as_str() {
                "PID" => *self = ControllerConfig::PID(pid::PIDConfig::default()),
                "External" => {
                    *self = ControllerConfig::External(
                        external_controller::ExternalControllerConfig::default(),
                    )
                }
                "Python" => {
                    *self = ControllerConfig::Python(
                        python_controller::PythonControllerConfig::default(),
                    )
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

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.horizontal(|ui| {
            ui.label(format!("Controller: {}", self));
        });
        match self {
            ControllerConfig::PID(c) => c.show(ui, ctx, unique_id),
            ControllerConfig::External(c) => c.show(ui, ctx, unique_id),
            ControllerConfig::Python(c) => c.show(ui, ctx, unique_id),
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
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| match self {
            Self::PID(r) => {
                egui::CollapsingHeader::new("PID").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
            Self::External(r) => {
                egui::CollapsingHeader::new("ExternalController").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
            Self::Python(r) => {
                egui::CollapsingHeader::new("ExternalPythonController").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
        });
    }
}

use crate::node::Node;

/// Controller strategy, which compute the [`Command`] to be sent to the
/// [`Physics`](crate::physics::physics::Physics) module, from the given `error`.
pub trait Controller:
    std::fmt::Debug
    + std::marker::Send
    + std::marker::Sync
    + Recordable<ControllerRecord>
    + MessageHandler
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

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32);
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
    plugin_api: &Option<Arc<dyn PluginAPI>>,
    global_config: &SimulatorConfig,
    va_factory: &Arc<DeterministRandomVariableFactory>,
    physics_config: &PhysicsConfig,
    initial_time: f32,
) -> SimbaResult<SharedRwLock<Box<dyn Controller>>> {
    Ok(Arc::new(RwLock::new(match config {
        ControllerConfig::PID(c) => {
            Box::new(pid::PID::from_config(c, physics_config, initial_time)) as Box<dyn Controller>
        }
        ControllerConfig::External(c) => {
            Box::new(external_controller::ExternalController::from_config(
                c,
                plugin_api,
                global_config,
                va_factory,
                initial_time,
            )?) as Box<dyn Controller>
        }
        ControllerConfig::Python(c) => Box::new(
            python_controller::PythonController::from_config(c, global_config, initial_time)
                .unwrap(),
        ) as Box<dyn Controller>,
    })))
}
