/*!
Module serving the [`Simulator`] with the configuration and record structures.

The [`Simulator`] is the primary struct to be called to start the simulator,
the simulator can be used as follows:
```
use std::path::Path;
use turtlebot_simulator::simulator::Simulator;

fn main() {
    // Initialize the environment, essentially the logging part
    Simulator::init_environment();

    // Load the configuration
    let config_path = Path::new("config_example/config.yaml");
    let mut simulator = Simulator::from_config_path(
        config_path,              //<- configuration path
        None,                     //<- plugin API, to load external modules
        Some(Box::from(Path::new("result.json"))), //<- path to save the results (None to not save)
        true,                     //<- Analyse the results
        false,                    //<- Show the figures after analyse
    );

    // Show the simulator loaded configuration
    simulator.show();

    // Run the simulation for 60 seconds.
    // It also save the results to "result.json",
    // compute the results and show the figures.
    simulator.run(60.);
}


```


*/

// Configuration for Simulator
extern crate confy;
use serde_derive::{Deserialize, Serialize};

use crate::networking::network_manager::NetworkManager;
use crate::plugin_api::PluginAPI;
use crate::result_analyser;

use super::turtlebot::{Turtlebot, TurtlebotConfig, TurtlebotRecord};
use std::path::Path;

use serde_json;
use std::default::Default;
use std::fs::File;
use std::io::prelude::*;
use std::sync::{Arc, RwLock};
use std::thread;

use log::{debug, error, info};

/// Meta configuration of [`Simulator`], to configure how the simulation
/// is run, outside of the scenario to run.
#[derive(Clone)]
pub struct SimulatorMetaConfig {
    /// Path to the [`SimulatorConfig`] file, containing the configuration
    /// of the scebarui to run
    pub config_path: Option<Box<Path>>,
    /// Filename to save the results, in JSON format. The directory of this
    /// file is used to save the figures if results are computed.
    /// Use [`Option::None`] to disable result save.
    pub result_path: Option<Box<Path>>,
    /// Compute the results or not at the end of the run. It uses a python
    /// script located in `python_scripts/analyse_results.py`.
    pub compute_results: bool,
    /// Do not show GUI (restricted for now to the result figures).
    pub no_gui: bool,
}

impl SimulatorMetaConfig {
    /// Default [`SimulatorMetaConfig`], doing nothing.
    pub fn default() -> Self {
        Self {
            config_path: None,
            result_path: None,
            compute_results: false,
            no_gui: true,
        }
    }
}

/// Scenario configuration for the simulator.
/// The Simulator configuration is the root of the scenario configuration.
///
/// This config contains an item, `turtles`, which list the robots [`TurtlebotConfig`].
///
/// ## Example in yaml:
/// ```
/// turtles:
///     - TurtlebotConfig 1
///     - TurtlebotConfig 2
/// ```
///
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct SimulatorConfig {
    /// List of the turtles (robots) to run, with their specific configuration.
    pub turtles: Vec<Box<TurtlebotConfig>>,
}

impl Default for SimulatorConfig {
    /// Default scenario configuration: no turtles.
    fn default() -> Self {
        Self {
            turtles: Vec::new(),
        }
    }
}

/// One time record of a turtle. The record is the state of the turtle with the
/// associated time.
///
/// This is a line for one turtle ([`TurtlebotRecord`]) at a given time.
#[derive(Debug, Serialize, Deserialize)]
pub struct Record {
    /// Time of the record.
    pub time: f32,
    /// Record of a turtle.
    pub turtle: TurtlebotRecord,
}

/// This is the central structure which manages the run of the scenario.
///
/// To run the scenario, there are two mandatory steps:
/// * Load the config using [`Simulator::from_config_path`] (from a file), or using
/// [`Simulator::from_config`] with the [`SimulatorConfig`] and [`SimulatorMetaConfig`]
/// structs directly.
/// * Run the scenario, once the config is loaded, the scenario can be run using
/// [`Simulator::run`].
///
/// Optionnal steps are the following:
/// * Initialize the environment with [`Simulator::init_environment`]. It initialize the logging environment.
/// * Use [`Simulator::show`] to print in the console the configuration loaded.
/// * Get the results with [`Simulator::get_results`], providing the full list of all [`Record`]s.
///
/// ## Example
/// ```
/// use std::path::Path;
/// use turtlebot_simulator::simulator::Simulator;
///
/// fn main() {
///     // Initialize the environment, essentially the logging part
///     Simulator::init_environment();
///
///     // Load the configuration
///     let config_path = Path::new("config_example/config.yaml");
///     let mut simulator = Simulator::from_config_path(
///         config_path,              //<- configuration path
///         None,                     //<- plugin API, to load external modules
///         Some(Box::from(Path::new("result.json"))), //<- path to save the results (None to not save)
///         true,                     //<- Analyse the results
///         false,                    //<- Show the figures after analyse
///     );
///
///     // Show the simulator loaded configuration
///     simulator.show();
///
///     // Run the simulation for 60 seconds.
///     // It also save the results to "result.json",
///     // compute the results and show the figures.
///     simulator.run(60.);
/// }
///
/// ```
pub struct Simulator {
    /// List of the [`Turtlebot`]. Using `Arc` and `RwLock` for multithreading.
    turtles: Vec<Arc<RwLock<Turtlebot>>>,
    /// Scenario configuration.
    config: SimulatorConfig,
    /// Simulation configuration.
    meta_config: SimulatorMetaConfig,
    /// Network Manager
    network_manager: Arc<RwLock<NetworkManager>>,
}

impl Simulator {
    /// Create a new [`Simulator`] with no turtles, and empty config.
    pub fn new() -> Simulator {
        Simulator {
            turtles: Vec::new(),
            config: SimulatorConfig::default(),
            meta_config: SimulatorMetaConfig::default(),
            network_manager: Arc::new(RwLock::new(NetworkManager::new())),
        }
    }

    /// Load the config from a file compatible with [`confy`]. Initialize the [`Simulator`].
    ///
    /// ## Arguments
    /// * `config_path` - `Path` to the config file (see example in [`config_example/config.yaml`]).
    /// * `plugin_api`  - Provide an implementation of [`PluginAPI`] if you want to use external modules.
    /// * `result_path` - Path to the file to save the results.
    /// * `compute_results` - Enable the computation of the results, using python script.
    /// * `no_gui` - Disable the GUI, and the opening of the figures.
    ///
    /// ## Return
    /// Returns a [`Simulator`] ready to be run.
    pub fn from_config_path(
        config_path: &Path,
        plugin_api: Option<Box<dyn PluginAPI>>,
        result_path: Option<Box<Path>>,
        compute_results: bool,
        no_gui: bool,
    ) -> Simulator {
        let config: SimulatorConfig = match confy::load_path(&config_path) {
            Ok(config) => config,
            Err(error) => {
                error!("Error from Confy while loading the config file : {}", error);
                return Simulator::new();
            }
        };
        debug!("Config: {:?}", config);
        let meta_config = SimulatorMetaConfig {
            config_path: Some(Box::from(config_path)),
            result_path,
            compute_results,
            no_gui,
        };
        Simulator::from_config(&config, plugin_api, meta_config)
    }

    /// Load the config from structure instance.
    ///
    /// ## Arguments
    /// * `config` - Scenario configuration ([`SimulatorConfig`]).
    /// * `plugin_api`  - Provide an implementation of [`PluginAPI`] if you want to use external modules.
    /// * `meta_config` - Simulator run configuration ([`SimulatorMetaConfig`]).
    ///
    /// ## Return
    /// Returns a [`Simulator`] ready to be run.
    pub fn from_config(
        config: &SimulatorConfig,
        plugin_api: Option<Box<dyn PluginAPI>>,
        meta_config: SimulatorMetaConfig,
    ) -> Simulator {
        let mut simulator = Simulator::new();
        simulator.config = config.clone();
        simulator.meta_config = meta_config.clone();

        // Create turtles
        for turtle_config in &config.turtles {
            simulator.add_turtlebot(turtle_config, &plugin_api, meta_config.clone());
            // simulator.turtles.push(Box::new(Turtlebot::from_config(turtle_config, &plugin_api)));
            // simulator.network_manager.register_turtle_network(simulator.turtles.last().expect("No turtle added to the vector, how is it possible ??").name(), simulator.turtles.last().expect("No turtle added to the vector, how is it possible ??").network());
        }
        simulator
    }

    /// Initialize the simulator environment.
    ///
    /// For now, only start the logging environment.
    pub fn init_environment() {
        env_logger::builder()
            .target(env_logger::Target::Stdout)
            .init();
    }

    /// Add a [`Turtlebot`] to the [`Simulator`].
    ///
    /// This function add the [`Turtlebot`] to the [`Simulator`] list and to the [`NetworkManager`].
    /// It also adds the [`NetworkManager`] to the new [`Turtlebot`].
    ///
    /// ## Argumants
    /// * `turtle_config` - Configuration of the [`Turtlebot`].
    /// * `plugin_api` - Implementation of [`PluginAPI`] for the use of external modules.
    /// * `meta_config` - Configuration of the simulation run.
    fn add_turtlebot(
        &mut self,
        turtle_config: &TurtlebotConfig,
        plugin_api: &Option<Box<dyn PluginAPI>>,
        meta_config: SimulatorMetaConfig,
    ) {
        self.turtles.push(Turtlebot::from_config(
            turtle_config,
            &plugin_api,
            meta_config,
        ));
        let last_turtle = self
            .turtles
            .last()
            .expect("No turtle added to the vector, how is it possible ??")
            .write()
            .unwrap();
        self.network_manager
            .write()
            .unwrap()
            .register_turtle_network(last_turtle.name(), last_turtle.network());
        last_turtle
            .network()
            .write()
            .unwrap()
            .set_network_manager(Arc::clone(&self.network_manager));
    }

    /// Simply print the Simulator state, using the info channel and the debug print.
    pub fn show(&self) {
        info!("Simulator:");
        for turtle in &self.turtles {
            info!("- {:?}", turtle);
        }
    }

    /// Run the scenario until the given time.
    ///
    /// This function starts one thread by [`Turtlebot`]. It waits that the thread finishes.
    ///
    /// After the scenario is done, the results are saved, and they are analysed, following
    /// the configuration give ([`SimulatorMetaConfig`]).
    ///
    /// ## Arguments
    /// * `max_time` - Run the scenario until this time is reached.
    pub fn run(&mut self, max_time: f32) {
        let mut handles = vec![];

        for turtle in &self.turtles {
            let new_turtle = Arc::clone(turtle);
            let new_max_time = max_time.clone();
            let handle = thread::spawn(move || Self::run_one_turtle(new_turtle, new_max_time));
            handles.push(handle);
        }

        for handle in handles {
            let _ = handle.join();
        }

        self.save_results();
        self.compute_results();
    }

    /// Returns the list of all [`Record`]s produced by [`Simulator::run`].
    pub fn get_results(&self) -> Vec<Record> {
        let mut records = Vec::new();
        for turtle in &self.turtles {
            let turtle_r = turtle.read().unwrap();
            let turtle_history = turtle_r.record_history();
            for (time, record) in turtle_history.iter() {
                records.push(Record {
                    time: time.clone(),
                    turtle: record.clone(),
                });
            }
        }
        records
    }

    /// Save the results to the file given during the configuration.
    ///
    /// If the configuration of the [`Simulator`] do not contain a result path, no results are saved.
    fn save_results(&mut self) {
        let filename = match &self.meta_config.result_path {
            Some(f) => f,
            None => return,
        };
        let mut recording_file = File::create(filename).expect("Impossible to create record file");

        let _ = recording_file.write(b"{\"config\": ");
        serde_json::to_writer(&recording_file, &self.config)
            .expect("Error during json serialization");
        let _ = recording_file.write(b",\n\"record\": [\n");

        let results = self.get_results();

        let mut first_row = true;
        for row in &results {
            if first_row {
                first_row = false;
            } else {
                let _ = recording_file.write(b",\n");
            }
            serde_json::to_writer(
                &recording_file,
                &Record {
                    time: row.time.clone(),
                    turtle: row.turtle.clone(),
                },
            )
            .expect("Error during json serialization");
        }
        let _ = recording_file.write(b"\n]}");
    }

    /// Run the loop for the given `turtle` until reaching `max_time`.
    ///
    /// ## Arguments
    /// * `turtle` - Turtle to be run.
    /// * `max_time` - Time to stop the loop.
    fn run_one_turtle(turtle: Arc<RwLock<Turtlebot>>, max_time: f32) {
        info!("Start thread of turtle {}", turtle.read().unwrap().name());

        loop {
            let next_time = turtle.read().unwrap().next_time_step();
            if next_time > max_time {
                break;
            }

            let mut turtle_open = turtle.write().unwrap();
            turtle_open.run_next_time_step(next_time);
        }
    }

    /// Compute the results from the file where it was saved before.
    ///
    /// If the [`Simulator`] config disabled the computation of the results, this function
    /// does nothing.
    ///
    /// The results are analysed using a python script, through [`result_analyser::execute_python_analyser`].
    fn compute_results(&self) {
        if !self.meta_config.compute_results {
            return;
        }

        let result_filename = match &self.meta_config.result_path {
            Some(f) => f,
            None => return,
        };

        info!("Starting result analyse...");
        let show_figures = !self.meta_config.no_gui;
        match result_analyser::execute_python_analyser(
            result_filename.as_ref(),
            show_figures,
            result_filename.parent().unwrap_or(Path::new(".")),
            String::from(".pdf"),
        ) {
            Ok(()) => info!("Results analysed!"),
            Err(e) => error!("Error during python execution: {e}"),
        };
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;

    #[test]
    fn replication_test() {
        let nb_replications = 100;
        env_logger::builder()
            .target(env_logger::Target::Stdout)
            .is_test(true)
            .filter_level(log::LevelFilter::Warn)
            .init();

        let mut results: Vec<Vec<Record>> = Vec::new();

        let config_path = Path::new("config_example/config.yaml");
        for i in 0..nb_replications {
            let mut simulator = Simulator::from_config_path(config_path, None, None, false, false);

            simulator.show();

            simulator.run(60.);

            results.push(simulator.get_results());
        }

        let reference_result = &results[0];
        for i in 1..nb_replications {
            assert_eq!(results[i].len(), reference_result.len());
            for j in 0..reference_result.len() {
                let result_as_str = format!("{:?}", results[i][j]);
                let reference_result_as_str = format!("{:?}", reference_result[j]);
                assert_eq!(result_as_str, reference_result_as_str);
            }
        }
    }
}
