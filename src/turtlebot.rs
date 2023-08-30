// Configuration for Turtlebot
extern crate confy;
//#[macro_use]
use serde_derive::{Serialize, Deserialize};


#[derive(Serialize, Deserialize)]
struct TurtlebotConfig {
    name: String
}

impl ::std::default::Default for TurtlebotConfig {
    fn default() -> Self { TurtlebotConfig { name: String::from("NoName") } }
}


// Turtlebot itself
use ndarray::{Array1, arr1};
use std::path::Path;

use super::configurable::{Configurable,ConfigurationLoadingError};

pub struct Turtlebot {
    name: String,
    pose: Array1<f64>
}

impl Turtlebot {
    pub fn new(name:String) -> Turtlebot {
        Turtlebot { name: name, pose: arr1(&[0.0,0.0,0.0]) }
    }

    pub fn from_config(config_path:&Path) -> Turtlebot {
        let mut turtlebot = Turtlebot::new(String::new());
        match turtlebot.load_config(config_path) {
            Err(e) => eprintln!("{}", e), // An Error Occurred, Please Try Again!
            _ => println!("Turtlebot loading successfull"),
        };
        return turtlebot
    }

    pub fn name(&self) -> &String {
        &self.name
    }

    pub fn set_name(&mut self, name:String) {
        self.name = name;
    }
}

impl Configurable for Turtlebot {
    fn load_config(&mut self, config_path:&Path) -> Result<(), ConfigurationLoadingError> {
        let config:TurtlebotConfig = match confy::load_path(&config_path) {
            Ok(config) => config,
            Err(error) => return Err(ConfigurationLoadingError{what:String::from(format!("Error from Confy while loading the config file : {}", error))})
        };
        
        self.name = config.name;
        Ok(())
    }
}