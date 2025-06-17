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

use super::{external_physics, perfect_physics};

/// Enumeration of the different physic implementations.
#[derive(Serialize, Deserialize, Debug, Clone, Check, ToVec, EnumToString)]
#[serde(deny_unknown_fields)]
pub enum PhysicsConfig {
    Perfect(Box<perfect_physics::PerfectsPhysicConfig>),
    External(Box<external_physics::ExternalPhysicsConfig>),
}

#[cfg(feature = "gui")]
impl UIComponent for PhysicsConfig {
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
                    *self = PhysicsConfig::Perfect(Box::new(
                        perfect_physics::PerfectsPhysicConfig::default(),
                    ))
                }
                "External" => {
                    *self = PhysicsConfig::External(Box::new(
                        external_physics::ExternalPhysicsConfig::default(),
                    ))
                }
                _ => panic!("Where did you find this value?"),
            };
        }
        match self {
            PhysicsConfig::Perfect(c) => c.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            PhysicsConfig::External(c) => c.show(
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

/// Enumeration of the records by physics implementations.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum PhysicsRecord {
    Perfect(perfect_physics::PerfectPhysicsRecord),
    External(external_physics::ExternalPhysicsRecord),
}

impl PhysicsRecord {
    pub fn pose(&self) -> [f32; 3] {
        match self {
            Self::External(p) => [0., 0., 0.],
            Self::Perfect(p) => p.state.pose.into(),
        }
    }
}

#[cfg(feature = "gui")]
use crate::gui::{utils::string_combobox, UIComponent};
use crate::{
    networking::service::HasService,
    plugin_api::PluginAPI,
    simulator::SimulatorConfig,
    state_estimators::state_estimator::State,
    stateful::Stateful,
    utils::{determinist_random_variable::DeterministRandomVariableFactory, enum_tools::ToVec},
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
    + Stateful<PhysicsRecord>
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
    fn state(&self, time: f32) -> &State;
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
    va_factory: &DeterministRandomVariableFactory,
) -> Arc<RwLock<Box<dyn Physics>>> {
    Arc::new(RwLock::new(match &config {
        PhysicsConfig::Perfect(c) => Box::new(perfect_physics::PerfectPhysics::from_config(
            c,
            plugin_api,
            global_config,
            va_factory,
        )) as Box<dyn Physics>,
        PhysicsConfig::External(c) => Box::new(external_physics::ExternalPhysics::from_config(
            c,
            plugin_api,
            global_config,
            va_factory,
        )),
    }))
}
