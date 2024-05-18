/*!
Module defining the [Controller]
*/

use crate::{stateful::Stateful};
use std::sync::{Arc, RwLock};

use crate::{physics::physic::Command, plugin_api::PluginAPI, simulator::SimulatorMetaConfig};

extern crate confy;
use serde_derive::{Deserialize, Serialize};

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

use super::pid;

/// Enumerates the strategies configurations.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum ControllerConfig {
    PID(Box<pid::PIDConfig>),
}

/// Enumerates the strategies records.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum ControllerRecord {
    PID(pid::PIDRecord),
}

use crate::turtlebot::Turtlebot;

/// Controller strategy, which compute the [`Command`] to be sent to the
/// [`Physic`](crate::physics::physic::Physic) module, from the given `error`.
pub trait Controller:
    std::fmt::Debug + std::marker::Send + std::marker::Sync + Stateful<ControllerRecord>
{
    /// Compute the command from the given error.
    ///
    /// ## Arguments
    /// * `turtle` - Reference to the robot to access modules.
    /// * `error` - Error to be corrected.
    /// * `time` - Current time.
    ///
    /// ## Return
    /// Command to apply to the [`Physic`](crate::physics::physic::Physic).
    fn make_command(
        &mut self,
        turtle: &mut Turtlebot,
        error: &ControllerError,
        time: f32,
    ) -> Command;
}

pub fn make_controller_from_config(
    config: &ControllerConfig,
    plugin_api: &Option<Box<dyn PluginAPI>>,
    meta_config: SimulatorMetaConfig,
) -> Arc<RwLock<Box<dyn Controller>>> {
    Arc::new(RwLock::new(Box::new(match config {
        ControllerConfig::PID(c) => pid::PID::from_config(c, plugin_api, meta_config.clone()),
    })))
}
