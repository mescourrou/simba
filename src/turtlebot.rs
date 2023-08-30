use ndarray::{Array1, arr1};

use super::configurable::{Configurable,ConfigurationLoadingError};
use std::path::Path;


pub struct Turtlebot {
    name: String,
    pose: Array1<f64>
}

impl Turtlebot {
    pub fn new(name:String) -> Turtlebot {
        Turtlebot { name: name, pose: arr1(&[0.0,0.0,0.0]) }
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
        Err(ConfigurationLoadingError{what: String::from("Need to do something here!")})
    }
}