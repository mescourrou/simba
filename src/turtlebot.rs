// Configuration for Turtlebot
extern crate confy;
#[macro_use]
use serde_derive::{Serialize, Deserialize};


#[derive(Serialize, Deserialize)]
pub struct TurtlebotConfig {
    pub name: String
}

impl Default for TurtlebotConfig {
    fn default() -> Self {
        TurtlebotConfig {
            name: String::from("NoName")
        }
    }
}


// Turtlebot itself
use ndarray::{Array1, arr1};
use std::path::Path;

use super::configurable::ConfigurationLoadingError;

pub struct Turtlebot {
    name: String,
    pose: Array1<f64>
}

impl Turtlebot {
    pub fn new(name:String) -> Turtlebot {
        Turtlebot { name: name, pose: arr1(&[0.0,0.0,0.0]) }
    }

    pub fn from_config(config:&TurtlebotConfig) -> Turtlebot {
        let mut turtlebot = Turtlebot::new(String::new());
        turtlebot.name = config.name.clone();
        return turtlebot
    }

    pub fn name(&self) -> &String {
        &self.name
    }

    pub fn set_name(&mut self, name:String) {
        self.name = name;
    }
}
