pub mod simulator;
mod configurable;
mod turtlebot;
mod navigators;
mod utils;
mod controllers;
mod physics;
mod state_estimators;

mod test_config;

use std::path::Path;

use simulator::Simulator;


fn main() {
    println!("Hello, world!");


    let config_path = Path::new("configs/config.yaml");
    let simulator = Simulator::from_config_path(config_path);

    simulator.show();

    // test_config::test();

}
