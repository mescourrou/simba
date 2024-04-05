use turtlebot_simulator::result_analyser;
use turtlebot_simulator::simulator::Simulator;

use std::path::Path;

fn main() {
    Simulator::init_environment();

    let config_path = Path::new("config_example/config.yaml");
    let mut simulator = Simulator::from_config_path(
        config_path, 
        None, 
        Path::new("result.json"),
        true,
        false
    );

    simulator.show();

    simulator.run(60.);
    // gui::run_gui();
}
