pub mod simulator;
mod configurable;
mod turtlebot;

use std::path::Path;

use simulator::Simulator;


fn main() {
    println!("Hello, world!");


    let config_path = Path::new("./config.yaml");
    let simulator = Simulator::from_config_path(config_path);

    simulator.show();
    // println!("{}", turtlebot.name());

}
