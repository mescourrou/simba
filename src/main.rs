use std::path::Path;
use turtlebot_simulator::simulator::Simulator;

fn main() {
    // Initialize the environment, essentially the logging part
    Simulator::init_environment();

    // Load the configuration
    let config_path = Path::new("config_example/config.yaml");
    let mut simulator = Simulator::from_config_path(
        config_path,                               //<- configuration path
        None,                                      //<- plugin API, to load external modules
        Some(Box::from(Path::new("result.json"))), //<- path to save the results (None to not save)
        true,                                      //<- Analyse the results
        false,                                     //<- Show the figures after analyse
    );

    // Show the simulator loaded configuration
    simulator.show();

    // Run the simulation for 60 seconds.
    // It also save the results to "result.json",
    // compute the results and show the figures.
    simulator.run(60.);
}
