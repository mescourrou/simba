use super::navigators::navigator::{Navigator, NavigatorConfig};
use super::navigators::trajectory_follower;

// Configuration for Turtlebot
extern crate confy;
use serde_derive::{Serialize, Deserialize};


#[derive(Serialize, Deserialize)]
#[serde(default)]
pub struct TurtlebotConfig {
    pub name: String,
    pub navigator: NavigatorConfig
}

impl Default for TurtlebotConfig {
    fn default() -> Self {
        TurtlebotConfig {
            name: String::from("NoName"),
            navigator: NavigatorConfig::TrajectoryFollower(Box::new(trajectory_follower::TrajectoryFollowerConfig::default()))
        }
    }
}


// Turtlebot itself

#[derive(Debug)]
pub struct Turtlebot {
    name: String,
    navigator: Box<dyn Navigator>
}

impl Turtlebot {
    pub fn new(name:String) -> Turtlebot {
        Turtlebot { 
            name: name,
            navigator: Box::new(trajectory_follower::TrajectoryFollower::new())
        }
    }

    pub fn from_config(config:&TurtlebotConfig) -> Turtlebot {
        Turtlebot {
            name: config.name.clone(),
            navigator: Box::new(
                match &config.navigator {
                    NavigatorConfig::TrajectoryFollower(c) => trajectory_follower::TrajectoryFollower::from_config(c)
                }
            )
        }
    }

    pub fn name(&self) -> &String {
        &self.name
    }

    pub fn set_name(&mut self, name:String) {
        self.name = name;
    }

    
}
