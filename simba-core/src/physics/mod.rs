/*!
This module proposes the [`Physics`](physics::Physics) trait and the perfect physics (no command noise).

Provide the [`Physics`] trait and the utilitary structs ([`PhysicsRecord`] and [`PhysicsConfig`]).

It also defines the [`Command`] that it can take.

The [`Physics`] implementation should take a command, apply it to the internal state, and it can add noise to it.
However, the [`Physics::state`] should provide the real [`State`].
*/

pub mod external_physics;
pub mod internal_physics;
pub mod pybinds;
pub mod python_physics;

pub mod robot_models;

pub mod fault_models;

extern crate confy;
use std::sync::{Arc, RwLock};

use nalgebra::Matrix3;
use serde_derive::{Deserialize, Serialize};
use simba_macros::config_derives;

/// Enumeration of the different physic implementations.
#[config_derives]
pub enum PhysicsConfig {
    Internal(internal_physics::InternalPhysicConfig),
    External(external_physics::ExternalPhysicsConfig),
    Python(python_physics::PythonPhysicsConfig),
}

#[cfg(feature = "gui")]
impl UIComponent for PhysicsConfig {
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
            ui.label("Physics:");
            string_combobox(
                ui,
                &PhysicsConfig::to_vec()
                    .iter()
                    .map(|x: &&str| String::from(*x))
                    .collect(),
                &mut current_str,
                format!("physics-choice-{}", unique_id),
            );
        });
        if current_str != self.to_string() {
            match current_str.as_str() {
                "Internal" => {
                    *self =
                        PhysicsConfig::Internal(internal_physics::InternalPhysicConfig::default())
                }
                "External" => {
                    *self =
                        PhysicsConfig::External(external_physics::ExternalPhysicsConfig::default())
                }
                "Python" => {
                    *self = PhysicsConfig::Python(python_physics::PythonPhysicsConfig::default())
                }
                _ => panic!("Where did you find this value?"),
            };
        }
        match self {
            PhysicsConfig::Internal(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            PhysicsConfig::External(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            PhysicsConfig::Python(c) => c.show_mut(
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
            ui.label(format!("Physics: {}", self));
        });
        match self {
            PhysicsConfig::Internal(c) => c.show(ui, ctx, unique_id),
            PhysicsConfig::External(c) => c.show(ui, ctx, unique_id),
            PhysicsConfig::Python(c) => c.show(ui, ctx, unique_id),
        }
    }
}

/// Enumeration of the records by physics implementations.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum PhysicsRecord {
    Internal(internal_physics::InternalPhysicsRecord),
    External(external_physics::ExternalPhysicsRecord),
    Python(python_physics::PythonPhysicsRecord),
}

impl PhysicsRecord {
    pub fn pose(&self) -> [f32; 3] {
        match self {
            Self::External(_) => [0., 0., 0.], // TODO: Find a way to get info from external record
            Self::Python(_) => [0., 0., 0.],   // TODO: Find a way to get info from external record
            Self::Internal(p) => p.state.pose,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PhysicsRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| match self {
            Self::Internal(r) => {
                egui::CollapsingHeader::new("Internal").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
            Self::External(r) => {
                egui::CollapsingHeader::new("ExternalPhysics").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
            Self::Python(r) => {
                egui::CollapsingHeader::new("ExternalPythonPhysics").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
        });
    }
}

use crate::{
    errors::SimbaResult,
    networking::{network::Network, service::HasService},
    physics::robot_models::Command,
    plugin_api::PluginAPI,
    recordable::Recordable,
    simulator::SimulatorConfig,
    state_estimators::State,
    utils::{SharedRwLock, determinist_random_variable::DeterministRandomVariableFactory},
};
#[cfg(feature = "gui")]
use crate::{
    gui::{UIComponent, utils::string_combobox},
    utils::enum_tools::ToVec,
};

// Services
#[derive(Debug, Clone)]
pub struct GetRealStateReq {}

#[derive(Debug, Clone)]
pub struct GetRealStateResp {
    pub state: State,
}

/// Physics simulation trait.
///
/// Different implementation can either use real robots, add noise to command, etc.
pub trait Physics:
    std::fmt::Debug
    + std::marker::Send
    + std::marker::Sync
    + Recordable<PhysicsRecord>
    + HasService<GetRealStateReq, GetRealStateResp>
{
    /// Apply the given `command` to the internal state from the last update time
    /// to the given `time`
    ///
    /// ## Arguments
    /// * `command` - Command to apply
    /// * `time` - Current time, when to apply the command.
    fn apply_command(&mut self, command: &Command, time: f32);

    /// Update the state to the given time, while keeping the previous command.
    fn update_state(&mut self, time: f32);

    /// Get the current real state, the groundtruth.
    fn state(&self, time: f32) -> State;

    /// Optional: return the time of the next time step. Needed if using messages
    fn next_time_step(&self) -> Option<f32> {
        None
    }
}

/// Helper function to create a physics from the given configuration.
///
/// ## Arguments
/// - `config`: The configuration of the physics.
/// - `plugin_api`: The plugin API, to be used by the physics (if needed).
/// - `meta_config`: The meta configuration of the simulator.
/// - `va_factory`: Random variables factory for determinist behavior.
/// - `time_cv`: Simulator time condition variable, used by services.
pub fn make_physics_from_config(
    config: &PhysicsConfig,
    plugin_api: &Option<Arc<dyn PluginAPI>>,
    global_config: &SimulatorConfig,
    robot_name: &String,
    va_factory: &Arc<DeterministRandomVariableFactory>,
    network: &SharedRwLock<Network>,
    initial_time: f32,
) -> SimbaResult<SharedRwLock<Box<dyn Physics>>> {
    Ok(Arc::new(RwLock::new(match &config {
        PhysicsConfig::Internal(c) => Box::new(internal_physics::InternalPhysics::from_config(
            c,
            robot_name,
            va_factory,
            initial_time,
        )) as Box<dyn Physics>,
        PhysicsConfig::External(c) => Box::new(external_physics::ExternalPhysics::from_config(
            c,
            plugin_api,
            global_config,
            va_factory,
            network,
            initial_time,
        )?),
        PhysicsConfig::Python(c) => Box::new(
            python_physics::PythonPhysics::from_config(c, global_config, initial_time).unwrap(),
        ),
    })))
}
