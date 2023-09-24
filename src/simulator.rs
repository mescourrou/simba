// Configuration for Simulator
extern crate confy;
use serde_derive::{Serialize, Deserialize};

use super::turtlebot::{Turtlebot,TurtlebotConfig};
use std::path::Path;

use std::default::Default;

#[derive(Serialize, Deserialize)]
#[serde(default)]
pub struct SimulatorConfig {
    pub turtles: Vec<Box<TurtlebotConfig>>
}

impl Default for SimulatorConfig {
    fn default() -> Self {
        Self {
            turtles: Vec::new()
        }
    }
}


pub struct Simulator {
    turtles: Vec<Box<Turtlebot>>
}

impl Simulator {
    pub fn new() -> Simulator {
        Simulator {
            turtles: Vec::new()
        }
    }

    pub fn from_config_path(config_path:&Path) -> Simulator {
        let config: SimulatorConfig = match confy::load_path(&config_path) {
            Ok(config) => config,
            Err(error) => {
                println!("Error from Confy while loading the config file : {}", error);
                return Simulator::new();
            }
        };
        Simulator::from_config(&config)
    }

    pub fn from_config(config:&SimulatorConfig) -> Simulator {
        let mut simulator = Simulator::new();

        // Create turtles
        for turtle_config in &config.turtles {
            simulator.turtles.push(Box::new(Turtlebot::from_config(turtle_config)));
        }
        simulator
    }

    pub fn show(&self) {
        println!("Simulator:");
        for turtle in &self.turtles {
            println!("- {:?}", turtle);
        }

    }
}
