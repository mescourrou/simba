use log::info;
use simba::simulator::{Simulator, SimulatorMetaConfig};
use simba::time_analysis::TimeAnalysisConfig;
use std::path::Path;

fn main() {
    // Initialize the environment, essentially the logging part
    Simulator::init_environment(log::LevelFilter::Debug);
    info!("Load configuration...");
    let mut simulator = Simulator::from_config_path(
        SimulatorMetaConfig {
            config_path: Some(Box::from(Path::new("example/config_example/config.yaml"))),
            result_path: Some(Box::from(Path::new("result.json"))),
            compute_results: true,
            no_gui: false,
            analyse_script: Some(
                Path::new(concat!(
                    env!("CARGO_MANIFEST_DIR"),
                    "/../python_scripts/analyse_results.py"
                ))
                .into(),
            ),
            time_analysis_config: TimeAnalysisConfig {
                exporter: simba::time_analysis::ProfileExporterConfig::TraceEventExporter,
                output_path: "time_performance".to_string(),
                keep_last: true,
            },
        },
        None, //<- plugin API, to load external modules
    );

    // Show the simulator loaded configuration
    simulator.show();

    // Run the simulation for 60 seconds.
    // It also save the results to "result.json",
    // compute the results and show the figures.
    simulator.run(60.);
}
