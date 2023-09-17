extern crate confy;
#[macro_use]
use serde_derive::{Serialize, Deserialize};

// #[derive(Serialize, Deserialize, Debug)]
// pub enum ControllerConfig {
//     TrajectoryFollower(TrajectoryFollowerConfig)
// }

// impl Default for NavigatorConfig {
//     fn default() -> Self {
//         Self {
//             TrajectoryFollower(TrajectoryFollowerConfig::default())
//         }
//     }
// }

pub trait Controller {}