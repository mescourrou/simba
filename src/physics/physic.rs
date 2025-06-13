/*!
Provide the [`Physic`] trait and the utilitary structs ([`PhysicRecord`] and [`PhysicConfig`]).

It also defines the [`Command`] that it can take. The control is done controlling the speed of the two wheels.

The [`Physic`] implementation should take a command, apply it to the internal state, and it can add noise to it.
However, the [`Physic::state`] should provide the real [`State`].
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

use super::{external_physic, perfect_physic};

/// Enumeration of the different physic implementations.
#[derive(Serialize, Deserialize, Debug, Clone, Check, ToVec, EnumToString)]
#[serde(deny_unknown_fields)]
pub enum PhysicConfig {
    Perfect(Box<perfect_physic::PerfectPhysicConfig>),
    External(Box<external_physic::ExternalPhysicConfig>),
}

#[cfg(feature = "gui")]
impl UIComponent for PhysicConfig {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::HashMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        let mut current_str = self.to_string();
        ui.horizontal(|ui| {
            ui.label("Physics:");
            string_combobox(
                ui,
                &PhysicConfig::to_vec()
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
                    *self = PhysicConfig::Perfect(Box::new(
                        perfect_physic::PerfectPhysicConfig::default(),
                    ))
                }
                "External" => {
                    *self = PhysicConfig::External(Box::new(
                        external_physic::ExternalPhysicConfig::default(),
                    ))
                }
                _ => panic!("Where did you find this value?"),
            };
        }
        match self {
            PhysicConfig::Perfect(c) => c.show(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            PhysicConfig::External(c) => c.show(
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

/// Enumeration of the records by physic implementations.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum PhysicRecord {
    Perfect(perfect_physic::PerfectPhysicRecord),
    External(external_physic::ExternalPhysicRecord),
}

impl PhysicRecord {
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

/// Physic simulation trait.
///
/// Different implementation can either use real robots, add noise to command, etc.
pub trait Physic:
    std::fmt::Debug
    + std::marker::Send
    + std::marker::Sync
    + Stateful<PhysicRecord>
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

/// Helper function to create a physic from the given configuration.
///
/// ## Arguments
/// - `config`: The configuration of the physic.
/// - `plugin_api`: The plugin API, to be used by the physic (if needed).
/// - `meta_config`: The meta configuration of the simulator.
/// - `va_factory`: Random variables factory for determinist behavior.
/// - `time_cv`: Simulator time condition variable, used by services.
pub fn make_physic_from_config(
    config: &PhysicConfig,
    plugin_api: &Option<Box<&dyn PluginAPI>>,
    global_config: &SimulatorConfig,
    va_factory: &DeterministRandomVariableFactory,
) -> Arc<RwLock<Box<dyn Physic>>> {
    Arc::new(RwLock::new(match &config {
        PhysicConfig::Perfect(c) => Box::new(perfect_physic::PerfectPhysic::from_config(
            c,
            plugin_api,
            global_config,
            va_factory,
        )) as Box<dyn Physic>,
        PhysicConfig::External(c) => Box::new(external_physic::ExternalPhysic::from_config(
            c,
            plugin_api,
            global_config,
            va_factory,
        )),
    }))
}
