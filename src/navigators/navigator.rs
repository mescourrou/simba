/*!
Provide the [`Navigator`] trait and the configuration and record enumerations.
*/

extern crate confy;
use serde_derive::{Deserialize, Serialize};

use super::trajectory_follower;

use crate::controllers::controller::ControllerError;
use crate::state_estimators::state_estimator::State;

/// Enumerate the configuration of the different strategies.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum NavigatorConfig {
    TrajectoryFollower(Box<trajectory_follower::TrajectoryFollowerConfig>),
}

/// Enumeration of the record of the different strategies.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum NavigatorRecord {
    TrajectoryFollower(trajectory_follower::TrajectoryFollowerRecord),
}

use crate::stateful::Stateful;
use crate::turtlebot::Turtlebot;

/// Trait managing the path planning, and providing the error to the planned path.
pub trait Navigator:
    std::fmt::Debug + std::marker::Send + std::marker::Sync + Stateful<NavigatorRecord>
{
    /// Compute the error ([`ControllerError`]) between the given `state` to the planned path.
    fn compute_error(&mut self, turtle: &mut Turtlebot, state: State) -> ControllerError;
}
