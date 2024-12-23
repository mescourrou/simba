use log::info;
use simba::simulator::{Simulator, SimulatorConfig};
use simba::time_analysis::TimeAnalysisConfig;
use std::path::Path;

fn main() {
    // Initialize the environment, essentially the logging part
    Simulator::init_environment(log::LevelFilter::Debug);
    info!("Load configuration...");
    let mut simulator = Simulator::from_config_path(
        Path::new("example/config_example/config.yaml"),
        None, //<- plugin API, to load external modules
    );

    // Show the simulator loaded configuration
    simulator.show();

    // Run the simulation for 60 seconds.
    // It also save the results to "result.json",
    // compute the results and show the figures.
    simulator.run(60.);
}
