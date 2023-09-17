use super::trajectory::{Trajectory, TrajectoryConfig};
use super::navigator::Navigator;

// Configuration for TrajectoryFollower
#[macro_use]
use serde_derive::{Serialize, Deserialize};

use std::path::Path;

#[derive(Serialize, Deserialize, Debug)]
#[serde(default)]
pub struct TrajectoryFollowerConfig {
    pub trajectory_path: String
}

impl Default for TrajectoryFollowerConfig {
    fn default() -> Self {
        Self {
            trajectory_path: String::from(""),
        }
    }
}




#[derive(Debug)]
pub struct TrajectoryFollower {
    trajectory: Trajectory
}

impl TrajectoryFollower {
    pub fn new() -> Self {
        Self {
            trajectory: Trajectory::new()
        }
    }

    pub fn from_config(config: &TrajectoryFollowerConfig) -> Self {
        let mut path = Path::new(&config.trajectory_path);
        if config.trajectory_path == "" {
            return Self::new();
        }
        let joined_path = Path::new("./configs").join(&config.trajectory_path);
        if path.is_relative() {
            path = joined_path.as_path();
        }
        Self::load_from_path(&path)
    }

    pub fn load_from_path(path: &Path) -> Self {
        let trajectory: TrajectoryConfig =  match confy::load_path(&path) {
            Ok(config) => config,
            Err(error) => {
                println!("Error from Confy while loading the trajectory file {} : {}", path.display(), error);
                return Self::new();
            }
        };
        TrajectoryFollower{
            trajectory: Trajectory::from_config(&trajectory)
        }
    }
}

impl Navigator for TrajectoryFollower {

}