use simba::simulator::{Simulator, SimulatorConfig};
use simba::time_analysis::TimeAnalysisConfig;
use std::path::Path;

fn main() {
    // Initialize the environment
    Simulator::init_environment();
    println!("Load configuration...");
    let mut simulator = Simulator::from_config_path(
        Path::new("config_example/config.yaml"),
        &None, //<- plugin API, to load external modules
    )
    .unwrap();

    // Show the simulator loaded configuration
    simulator.show();

    // Run the simulator for the time given in the configuration
    // It also save the results to json
    simulator.run().unwrap();

    simulator.compute_results().unwrap();
}
