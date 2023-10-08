extern crate confy;
use serde_derive::{Serialize, Deserialize};

use super::trajectory_follower;

use crate::state_estimators::state_estimator::State;
use crate::controllers::controller::ControllerError;

#[derive(Serialize, Deserialize, Debug)]
pub enum NavigatorConfig {
    TrajectoryFollower(Box<trajectory_follower::TrajectoryFollowerConfig>)
}

#[derive(Serialize, Deserialize, Debug)]
pub enum NavigatorRecord {
    TrajectoryFollower(trajectory_follower::TrajectoryFollowerRecord)
}

pub trait Navigator : std::fmt::Debug {
    fn compute_error(&mut self, state: &State) -> ControllerError;
    fn record(&self) ->  NavigatorRecord;
}


// impl std::fmt::Debug for dyn Navigator {
//     fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
//         write!(f, "{}", "derp")
//     }
// }