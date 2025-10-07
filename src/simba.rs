use std::{env, path::Path};
use clap::Parser;

use simba::{
    gui, simulator::{Simulator, SimulatorConfig},
    state_estimators::state_estimator::StateEstimator,
};

#[derive(Parser)]
struct Cli {
    #[arg(long, default_value_t = false)]
    no_gui: bool,
    config_path: Option<String>,
}

fn main() {
    let args = Cli::parse();

    

    if args.no_gui {
        let config_path = if let Some(p) = &args.config_path {
            Some(Path::new(unsafe {
                std::mem::transmute::<&String, &'static String>(p)
            }))
        } else {
            None
        };
        if config_path.is_none() {
            println!("If no gui is used, provide a config path!");
            return;
        }
        // Initialize the environment
        Simulator::init_environment();
        println!("Load configuration...");
        let mut simulator = Simulator::from_config_path(
            config_path.unwrap(),
            &None, //<- plugin API, to load external modules
        )
        .unwrap();

        // Show the simulator loaded configuration
        simulator.show();

        // Run the simulator for the time given in the configuration
        // It also save the results to json
        simulator.run().unwrap();

        simulator.compute_results().unwrap();

        return;
    }

    let config_path = if let Some(p) = &args.config_path {
        Some(Box::new(Path::new(unsafe {
            std::mem::transmute::<&String, &'static String>(p)
        })))
    } else {
        None
    };

    gui::run_gui(config_path, None);
}
