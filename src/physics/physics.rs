/*!
Provide the [`Physics`] trait and the utilitary structs ([`PhysicsRecord`] and [`PhysicsConfig`]).

It also defines the [`Command`] that it can take. The control is done controlling the speed of the two wheels.

The [`Physics`] implementation should take a command, apply it to the internal state, and it can add noise to it.
However, the [`Physics::state`] should provide the real [`State`].
*/

extern crate confy;
use std::sync::{Arc, RwLock};

use config_checker::macros::Check;
use serde_derive::{Deserialize, Serialize};
use simba_macros::{EnumToString, ToVec};

/// Command struct, to control both wheel speed, in m/s.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Command {
    /// Left wheel speed.
    pub left_wheel_speed: f32,
    /// Right wheel speed.
    pub right_wheel_speed: f32,
}

#[cfg(feature = "gui")]
impl UIComponent for Command {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &String) {
        ui.vertical(|ui| {
            ui.label(format!("Left wheel speed: {}", self.left_wheel_speed));
            ui.label(format!("Right wheel speed: {}", self.right_wheel_speed));
        });
    }
}

use super::{external_physics, perfect_physics};

/// Enumeration of the different physic implementations.
#[derive(Serialize, Deserialize, Debug, Clone, Check, ToVec, EnumToString)]
#[serde(deny_unknown_fields)]
pub enum PhysicsConfig {
    Perfect(perfect_physics::PerfectsPhysicConfig),
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
        unique_id: &String,
    ) {
        let mut current_str = self.to_string();
        ui.horizontal(|ui| {
            ui.label("Physics:");
            string_combobox(
                ui,
                &PhysicsConfig::to_vec()
                    .iter()
                    .map(|x| String::from(*x))
                    .collect(),
                &mut current_str,
                format!("physics-choice-{}", unique_id),
            );
        });
        if current_str != self.to_string() {
            match current_str.as_str() {
                "Perfect" => {
                    *self = PhysicsConfig::Perfect(perfect_physics::PerfectsPhysicConfig::default())
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
            PhysicsConfig::Perfect(c) => c.show_mut(
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

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &String) {
        ui.horizontal(|ui| {
            ui.label(format!("Physics: {}", self.to_string()));
        });
        match self {
            PhysicsConfig::Perfect(c) => c.show(ui, ctx, unique_id),
            PhysicsConfig::External(c) => c.show(ui, ctx, unique_id),
            PhysicsConfig::Python(c) => c.show(ui, ctx, unique_id),
        }
    }
}

/// Enumeration of the records by physics implementations.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum PhysicsRecord {
    Perfect(perfect_physics::PerfectPhysicsRecord),
    External(external_physics::ExternalPhysicsRecord),
    Python(python_physics::PythonPhysicsRecord),
}

impl PhysicsRecord {
    pub fn pose(&self) -> [f32; 3] {
        match self {
            Self::External(_) => [0., 0., 0.], // TODO: Find a way to get info from external record
            Self::Python(_) => [0., 0., 0.],   // TODO: Find a way to get info from external record
            Self::Perfect(p) => p.state.pose.into(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PhysicsRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &String) {
        ui.vertical(|ui| match self {
            Self::Perfect(r) => {
                egui::CollapsingHeader::new("Perfect").show(ui, |ui| {
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

#[cfg(feature = "gui")]
use crate::{
    gui::{utils::string_combobox, UIComponent},
    utils::enum_tools::ToVec,
};
use crate::{
    networking::service::HasService, physics::python_physics, plugin_api::PluginAPI,
    recordable::Recordable, simulator::SimulatorConfig, state_estimators::state_estimator::State,
    utils::determinist_random_variable::DeterministRandomVariableFactory,
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
    plugin_api: &Option<Box<&dyn PluginAPI>>,
    global_config: &SimulatorConfig,
    va_factory: &Arc<DeterministRandomVariableFactory>,
) -> Arc<RwLock<Box<dyn Physics>>> {
    Arc::new(RwLock::new(match &config {
        PhysicsConfig::Perfect(c) => {
            Box::new(perfect_physics::PerfectPhysics::from_config(c)) as Box<dyn Physics>
        }
        PhysicsConfig::External(c) => Box::new(external_physics::ExternalPhysics::from_config(
            c,
            plugin_api,
            global_config,
            va_factory,
        )),
        PhysicsConfig::Python(c) => {
            Box::new(python_physics::PythonPhysics::from_config(c, global_config).unwrap())
        }
    }))
}
