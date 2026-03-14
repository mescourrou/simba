//! Physics module.
//!
//! This module defines the [`Physics`] trait, physics configuration/record enums,
//! service request/response types, and factory helpers used to instantiate runtime physics
//! implementations.
//! Implementations may include perfect/internal physics, external plugin-backed physics,
//! or Python-backed physics.

pub mod external_physics;
pub mod internal_physics;
pub mod pybinds;
pub mod python_physics;

pub mod robot_models;

pub mod fault_models;

extern crate confy;
use std::sync::{Arc, RwLock};

use config_checker::*;
use serde_derive::{Deserialize, Serialize};
use simba_macros::config_derives;

/// Enumeration of the different physic implementations.
#[config_derives]
pub enum PhysicsConfig {
    /// Built-in Rust internal physics implementation.
    #[check]
    Internal(internal_physics::InternalPhysicConfig),
    /// External plugin-provided physics implementation.
    #[check]
    External(external_physics::ExternalPhysicsConfig),
    /// Python-backed physics implementation.
    #[check]
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
                &PhysicsConfig::to_vec(),
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
    /// Record emitted by internal physics.
    Internal(internal_physics::InternalPhysicsRecord),
    /// Record emitted by external physics.
    External(external_physics::ExternalPhysicsRecord),
    /// Record emitted by Python physics.
    Python(python_physics::PythonPhysicsRecord),
}

impl PhysicsRecord {
    /// Returns the pose encoded in this record when available.
    ///
    /// For external and Python records, this currently returns `[0.0, 0.0, 0.0]`.
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
    networking::service::HasService,
    node::{Node, node_factory::FromConfigArguments},
    physics::robot_models::Command,
    recordable::Recordable,
    simulator::SimulatorConfig,
    state_estimators::State,
    utils::SharedRwLock,
};
#[cfg(feature = "gui")]
use crate::{
    gui::{UIComponent, utils::string_combobox},
    utils::enum_tools::ToVec,
};

// Services
#[derive(Debug, Clone)]
/// Service request for retrieving the real (ground-truth) state.
pub struct GetRealStateReq {}

#[derive(Debug, Clone)]
/// Service response containing the real (ground-truth) state.
pub struct GetRealStateResp {
    /// Ground-truth state.
    pub state: State,
}

/// Physics simulation trait.
pub trait Physics:
    std::fmt::Debug
    + std::marker::Send
    + std::marker::Sync
    + Recordable<PhysicsRecord>
    + HasService<GetRealStateReq, GetRealStateResp>
{
    /// Optional initialization hook called once after node setup.
    #[allow(unused_variables)]
    fn post_init(&mut self, node: &mut Node) -> SimbaResult<()> {
        Ok(())
    }

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
/// - `from_config_args`: Additional arguments needed to create the physics, such as the robot name, random variable factory, initial time, plugin API, global config and network.
pub fn make_physics_from_config(
    config: &PhysicsConfig,
    from_config_args: &FromConfigArguments,
) -> SimbaResult<SharedRwLock<Box<dyn Physics>>> {
    Ok(Arc::new(RwLock::new(match &config {
        PhysicsConfig::Internal(c) => Box::new(internal_physics::InternalPhysics::from_config(
            c,
            from_config_args.node_name,
            from_config_args.va_factory,
            from_config_args.initial_time,
        )) as Box<dyn Physics>,
        PhysicsConfig::External(c) => Box::new(external_physics::ExternalPhysics::from_config(
            c,
            from_config_args.plugin_api,
            from_config_args.global_config,
            from_config_args.va_factory,
            from_config_args.network,
            from_config_args.initial_time,
        )?),
        PhysicsConfig::Python(c) => Box::new(
            python_physics::PythonPhysics::from_config(
                c,
                from_config_args.global_config,
                from_config_args.initial_time,
            )
            .unwrap(),
        ),
    })))
}
