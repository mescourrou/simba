extern crate nalgebra as na;
use na::{SVector};

extern crate confy;
use serde_derive::{Serialize, Deserialize};

use super::trajectory_follower;

#[derive(Serialize, Deserialize, Debug)]
pub enum NavigatorConfig {
    TrajectoryFollower(Box<trajectory_follower::TrajectoryFollowerConfig>)
}

pub trait Navigator : std::fmt::Debug{
    fn compute_error(&self, pose: SVector<f32, 3>) -> SVector<f32, 2>;
}


// impl std::fmt::Debug for dyn Navigator {
//     fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
//         write!(f, "{}", "derp")
//     }
// }