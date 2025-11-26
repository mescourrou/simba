use clap::Parser;
use std::path::Path;

use simba::{errors::SimbaResult, gui, simulator::Simulator};

#[derive(Parser)]
#[command(version, about)]
struct Cli {
    /// Disable GUI: will load the config, run the simulation and compute the results
    #[arg(long, default_value_t = false)]
    no_gui: bool,
    /// Default config path. Required if no-gui is used
    config_path: Option<String>,
    /// Load result from the result file specified in the configuration. Without GUI, processes the results directly.
    #[arg(long, default_value_t = false)]
    load_results: bool,
}

fn doit(args: Cli) -> SimbaResult<()> {
    if args.no_gui {
        let config_path = args
            .config_path
            .as_ref()
            .map(|p| Path::new(unsafe { std::mem::transmute::<&String, &'static String>(p) }));
        if config_path.is_none() {
            println!("If no gui is used, provide a config path!");
            return Ok(());
        }
        // Initialize the environment
        Simulator::init_environment();
        println!("Load configuration...");
        let mut simulator = Simulator::from_config_path(
            config_path.unwrap(),
            &None, //<- plugin API, to load external modules
        )?;

        // Show the simulator loaded configuration
        simulator.show();

        if !args.load_results {
            // Run the simulator for the time given in the configuration
            // It also save the results to json
            simulator.run()?;
        } else {
            simulator.load_results()?;
        }
        simulator.compute_results()?;

        return Ok(());
    }

    let config_path = args
        .config_path
        .as_ref()
        .map(|p| Path::new(unsafe { std::mem::transmute::<&String, &'static String>(p) }));

    gui::run_gui(config_path, None, args.load_results);
    Ok(())
}

fn main() {
    let args = Cli::parse();

    let res = doit(args);
    if let Err(e) = res {
        println!("{}", e.detailed_error());
    }
}
