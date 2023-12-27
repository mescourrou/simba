// pub mod simulator;
// mod configurable;
// mod turtlebot;
// mod navigators;
// mod utils;
// mod controllers;
// mod physics;
// mod state_estimators;
// mod plots;

// mod test_config;


use turtlebot_simulator::simulator::Simulator;
use turtlebot_simulator::gui;
use std::path::Path;
use turtlebot_simulator::test_config;


fn main() {
    println!("Hello, world!");


    let config_path = Path::new("configs/config.yaml");
    let mut simulator = Simulator::from_config_path(config_path, None);

    simulator.show();

    simulator.run(1.);

    // gui::run_gui();
    

    // test_config::test();

}
