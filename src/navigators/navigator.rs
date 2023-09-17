extern crate confy;
#[macro_use]
use serde_derive::{Serialize, Deserialize};

use super::trajectory_follower;

#[derive(Serialize, Deserialize, Debug)]
pub enum NavigatorConfig {
    TrajectoryFollower(Box<trajectory_follower::TrajectoryFollowerConfig>)
}

pub trait Navigator : std::fmt::Debug{}

// impl std::fmt::Debug for dyn Navigator {
//     fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
//         write!(f, "{}", "derp")
//     }
// }