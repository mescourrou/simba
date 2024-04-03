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
    for i in 1..10 {
        let mut simulator = Simulator::from_config_path(config_path, None);

        simulator.show();

        simulator.run(60.);

        simulator.save_results(Path::new(&format!("result{i}.json")));
    }
    // gui::run_gui();
    

}
