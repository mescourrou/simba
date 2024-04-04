extern crate confy;
use serde_derive::{Serialize, Deserialize};

use super::trajectory_follower;

use crate::state_estimators::state_estimator::State;
use crate::controllers::controller::ControllerError;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum NavigatorConfig {
    TrajectoryFollower(Box<trajectory_follower::TrajectoryFollowerConfig>)
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum NavigatorRecord {
    TrajectoryFollower(trajectory_follower::TrajectoryFollowerRecord)
}

use crate::stateful::Stateful;
use crate::turtlebot::Turtlebot;

pub trait Navigator : std::fmt::Debug + std::marker::Send + std::marker::Sync + Stateful<NavigatorRecord> {
    fn compute_error(&mut self, turtle: &mut Turtlebot, state: State) -> ControllerError;
}
