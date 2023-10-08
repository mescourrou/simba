pub mod simulator;
mod configurable;
mod turtlebot;
mod navigators;
mod utils;
mod controllers;
mod physics;
mod state_estimators;
mod plots;

mod test_config;


use simulator::Simulator;
use std::path::Path;


fn main() {
    println!("Hello, world!");


    let config_path = Path::new("configs/config.yaml");
    let mut simulator = Simulator::from_config_path(config_path);

    simulator.show();

    simulator.run(1.);

    // plots::real_time_plots::test();

    // test_config::test();

}
