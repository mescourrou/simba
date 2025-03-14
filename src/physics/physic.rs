/*!
Provide the [`Physic`] trait and the utilitary structs ([`PhysicRecord`] and [`PhysicConfig`]).

It also defines the [`Command`] that it can take. The control is done controlling the speed of the two wheels.

The [`Physic`] implementation should take a command, apply it to the internal state, and it can add noise to it.
However, the [`Physic::state`] should provide the real [`State`].
*/

extern crate confy;
use std::sync::{Arc, Condvar, Mutex, RwLock};

use config_checker::macros::Check;
use pyo3::{pyclass, pymethods};
use serde_derive::{Deserialize, Serialize};

/// Command struct, to control both wheel speed, in m/s.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass(get_all, set_all)]
pub struct Command {
    /// Left wheel speed.
    pub left_wheel_speed: f32,
    /// Right wheel speed.
    pub right_wheel_speed: f32,
}

#[pymethods]
impl Command {
    #[new]
    pub fn new() -> Command {
        Self {
            left_wheel_speed: 0.,
            right_wheel_speed: 0.,
        }
    }
}

use super::perfect_physic;

/// Enumeration of the different physic implementations.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(deny_unknown_fields)]
pub enum PhysicConfig {
    Perfect(Box<perfect_physic::PerfectPhysicConfig>),
}

/// Enumeration of the records by physic implementations.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass(get_all)]
pub enum PhysicRecord {
    Perfect(perfect_physic::PerfectPhysicRecord),
}

use crate::{
    networking::service::HasService, plugin_api::PluginAPI, simulator::SimulatorConfig,
    state_estimators::state_estimator::State, stateful::Stateful,
    utils::determinist_random_variable::DeterministRandomVariableFactory,
};

// Services
#[derive(Debug)]
pub struct GetRealStateReq {}

#[derive(Debug)]
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
    time_cv: Arc<(Mutex<usize>, Condvar)>,
) -> Arc<RwLock<Box<dyn Physic>>> {
    Arc::new(RwLock::new(Box::new(match &config {
        PhysicConfig::Perfect(c) => perfect_physic::PerfectPhysic::from_config(
            c,
            plugin_api,
            global_config,
            va_factory,
            time_cv,
        ),
    })))
}
