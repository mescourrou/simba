use turtlebot_simulator::simulator::Simulator;
use turtlebot_simulator::gui;
use std::path::Path;


use std::sync::mpsc;
use std::thread;
use std::time::Duration;

fn main() {
    println!("Hello, world!");

    Simulator::init_environment();
    
    let config_path = Path::new("config_example/config.yaml");
    let mut simulator = Simulator::from_config_path(config_path, None);

    simulator.show();

    simulator.run(1.);

    // gui::run_gui();
    

}
