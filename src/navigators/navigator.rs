/*!
Provide the [`Navigator`] trait and the configuration and record enumerations.
*/

extern crate confy;
use std::sync::{Arc, RwLock};

use pyo3::pyclass;
use serde_derive::{Deserialize, Serialize};

use super::trajectory_follower;

use crate::controllers::controller::ControllerError;
use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorMetaConfig;
use crate::state_estimators::state_estimator::State;

/// Enumerate the configuration of the different strategies.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum NavigatorConfig {
    TrajectoryFollower(Box<trajectory_follower::TrajectoryFollowerConfig>),
}

/// Enumeration of the record of the different strategies.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass(get_all)]
pub enum NavigatorRecord {
    TrajectoryFollower(trajectory_follower::TrajectoryFollowerRecord),
}

use crate::stateful::Stateful;
use crate::turtlebot::Turtlebot;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;

/// Trait managing the path planning, and providing the error to the planned path.
pub trait Navigator:
    std::fmt::Debug + std::marker::Send + std::marker::Sync + Stateful<NavigatorRecord>
{
    /// Compute the error ([`ControllerError`]) between the given `state` to the planned path.
    fn compute_error(&mut self, turtle: &mut Turtlebot, state: State) -> ControllerError;
}

pub fn make_navigator_from_config(
    config: &NavigatorConfig,
    plugin_api: &Option<Box<&dyn PluginAPI>>,
    meta_config: SimulatorMetaConfig,
    va_factory: &DeterministRandomVariableFactory,
) -> Arc<RwLock<Box<dyn Navigator>>> {
    Arc::new(RwLock::new(Box::new(match config {
        NavigatorConfig::TrajectoryFollower(c) => {
            trajectory_follower::TrajectoryFollower::from_config(
                c,
                plugin_api,
                meta_config.clone(),
                va_factory,
            )
        }
    })))
}
