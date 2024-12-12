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
use pyo3::prelude::*;
use serde_derive::{Deserialize, Serialize};

use crate::networking::network_manager::NetworkManager;
use crate::plugin_api::PluginAPI;
use crate::time_analysis;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;

use super::turtlebot::{Turtlebot, TurtlebotConfig, TurtlebotRecord};
use std::path::Path;
use std::time::SystemTime;

use serde_json;
use std::default::Default;
use std::fs::{self, File};
use std::io::prelude::*;
use std::sync::{Arc, Condvar, Mutex, RwLock};
use std::thread;

use log::{debug, error, info};

use pyo3::prepare_freethreaded_python;

/// Meta configuration of [`Simulator`], to configure how the simulation
/// is run, outside of the scenario to run.
#[derive(Serialize, Deserialize, Clone)]
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
    /// Path to the python analyse scrit.
    /// This script should have the following entry point:
    /// ```def analyse(result_data: Record, figure_path: str, figure_type: str)```
    pub analyse_script: Option<Box<Path>>,
}

impl SimulatorMetaConfig {
    /// Default [`SimulatorMetaConfig`], doing nothing.
    pub fn default() -> Self {
        Self {
            config_path: None,
            result_path: None,
            compute_results: false,
            no_gui: true,
            analyse_script: None,
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
    pub random_seed: Option<f32>,
    pub turtles: Vec<Box<TurtlebotConfig>>,
}

impl Default for SimulatorConfig {
    /// Default scenario configuration: no turtles.
    fn default() -> Self {
        Self {
            random_seed: None,
            turtles: Vec::new(),
        }
    }
}

/// One time record of a turtle. The record is the state of the turtle with the
/// associated time.
///
/// This is a line for one turtle ([`TurtlebotRecord`]) at a given time.
#[derive(Debug, Serialize, Deserialize)]
#[pyclass(get_all)]
pub struct Record {
    /// Time of the record.
    pub time: f32,
    /// Record of a turtle.
    pub turtle: TurtlebotRecord,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Results {
    pub config: SimulatorConfig,
    pub records: Vec<Record>,
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
    turtles: Arc<RwLock<Vec<Arc<RwLock<Turtlebot>>>>>,
    /// Scenario configuration.
    config: SimulatorConfig,
    /// Simulation configuration.
    meta_config: SimulatorMetaConfig,
    /// Network Manager
    network_manager: Arc<RwLock<NetworkManager>>,
    /// Factory for components to make random variables generators
    determinist_va_factory: DeterministRandomVariableFactory,

    time_cv: Arc<(Mutex<usize>, Condvar)>,
}

impl Simulator {
    /// Create a new [`Simulator`] with no turtles, and empty config.
    pub fn new() -> Simulator {
        Simulator {
            turtles: Arc::new(RwLock::new(Vec::new())),
            config: SimulatorConfig::default(),
            meta_config: SimulatorMetaConfig::default(),
            network_manager: Arc::new(RwLock::new(NetworkManager::new())),
            determinist_va_factory: DeterministRandomVariableFactory::new(
                SystemTime::now()
                    .elapsed()
                    .expect("Can't get the system time")
                    .as_secs_f32(),
            ),
            time_cv: Arc::new((Mutex::new(0), Condvar::new())),
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
        plugin_api: Option<Box<&dyn PluginAPI>>,
        result_path: Option<Box<Path>>,
        compute_results: bool,
        no_gui: bool,
        analyse_script: Option<Box<Path>>,
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
            analyse_script,
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
        plugin_api: Option<Box<&dyn PluginAPI>>,
        meta_config: SimulatorMetaConfig,
    ) -> Simulator {
        let mut simulator = Simulator::new();
        simulator.config = config.clone();
        simulator.meta_config = meta_config.clone();
        if let Some(seed) = config.random_seed {
            simulator.determinist_va_factory.global_seed = seed;
        }

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
    pub fn init_environment(level: log::LevelFilter) {
        env_logger::builder()
            .target(env_logger::Target::Stdout)
            .filter_level(level)
            .init();
        time_analysis::set_turtle_name("simulator".to_string());
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
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        meta_config: SimulatorMetaConfig,
    ) {
        self.turtles.write().unwrap().push(Turtlebot::from_config(
            turtle_config,
            plugin_api,
            meta_config,
            &self.determinist_va_factory,
            self.time_cv.clone(),
        ));

        let turtle_list = self.turtles.read().unwrap();

        self.network_manager
            .write()
            .unwrap()
            .register_turtle_network(Arc::clone(&turtle_list.last().unwrap()));
        let last_turtle_write = turtle_list
            .last()
            .expect("No turtle added to the vector, how is it possible ??")
            .write()
            .unwrap();
        last_turtle_write
            .network()
            .write()
            .unwrap()
            .set_network_manager(Arc::clone(&self.network_manager));
    }

    /// Simply print the Simulator state, using the info channel and the debug print.
    pub fn show(&self) {
        println!("Simulator:");
        let turtle_list = self.turtles.read().unwrap();
        for turtle in turtle_list.iter() {
            println!("- {:?}", turtle);
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

        let mut i = 0;
        for turtle in self.turtles.read().unwrap().iter() {
            let new_turtle = Arc::clone(turtle);
            let new_max_time = max_time.clone();
            let turtle_list = Arc::clone(&self.turtles);
            let time_cv = self.time_cv.clone();
            let handle = thread::spawn(move || {
                Self::run_one_turtle(new_turtle, new_max_time, turtle_list, i, time_cv)
            });
            handles.push(handle);
            i += 1;
        }

        for handle in handles {
            let _ = handle.join();
        }

        self.save_results();
        let results = self.get_results();
        self.compute_results(results, &self.config);
    }

    /// Returns the list of all [`Record`]s produced by [`Simulator::run`].
    pub fn get_results(&self) -> Vec<Record> {
        let mut records = Vec::new();
        for turtle in self.turtles.read().unwrap().iter() {
            let turtle_r = turtle.read().expect("Turtle cannot be read");
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

        time_analysis::save_results(Path::new("time_performance.json"));
        info!(
            "Saving results to {}",
            filename.to_str().unwrap_or_default()
        );
        let mut recording_file = File::create(filename).expect("Impossible to create record file");

        let _ = recording_file.write(b"{\"config\": ");
        serde_json::to_writer(&recording_file, &self.config)
            .expect("Error during json serialization");
        let _ = recording_file.write(b",\n\"records\": [\n");

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

    pub fn load_results_and_analyse(&mut self, filename: &Path) {
        let mut recording_file = File::open(filename).expect("Impossible to open record file");
        let mut content = String::new();
        recording_file
            .read_to_string(&mut content)
            .expect("Impossible to read record file");

        let results: Results = serde_json::from_str(&content).expect("Error during json parsing");

        self.compute_results(results.records, &results.config);
    }

    /// Wait the end of the simulation. If other turtle send messages, the simulation
    /// will go back to the time of the last message received.
    fn wait_the_end(
        turtle: &Arc<RwLock<Turtlebot>>,
        max_time: f32,
        cv_mtx: &Mutex<usize>,
        cv: &Condvar,
        nb_turtles: usize,
    ) -> bool {
        let mut finished_turtle = cv_mtx.lock().unwrap();
        *finished_turtle = *finished_turtle + 1;
        if *finished_turtle == nb_turtles {
            cv.notify_all();
            return true;
        }
        loop {
            let buffered_msgs = turtle.read().unwrap().process_messages();
            if buffered_msgs > 0 {
                *finished_turtle = *finished_turtle - 1;
                return false;
            }
            finished_turtle = cv.wait(finished_turtle).unwrap();
            if *finished_turtle == nb_turtles {
                return true;
            } else {
                *finished_turtle = *finished_turtle - 1;
                let turtle_open = turtle.read().unwrap();
                turtle_open.process_messages();
                let next_time = turtle_open.next_time_step();
                if next_time < max_time {
                    return false;
                }
                *finished_turtle = *finished_turtle + 1;
            }
        }
    }
    /// Run the loop for the given `turtle` until reaching `max_time`.
    ///
    /// ## Arguments
    /// * `turtle` - Turtle to be run.
    /// * `max_time` - Time to stop the loop.
    fn run_one_turtle(
        turtle: Arc<RwLock<Turtlebot>>,
        max_time: f32,
        turtle_list: Arc<RwLock<Vec<Arc<RwLock<Turtlebot>>>>>,
        turtle_idx: usize,
        time_cv: Arc<(Mutex<usize>, Condvar)>,
    ) {
        info!("Start thread of turtle {}", turtle.read().unwrap().name());
        time_analysis::set_turtle_name(turtle.read().unwrap().name());
        let nb_turtles = turtle_list.read().unwrap().len();
        info!(
            "[{}] Finishing initialization",
            turtle.read().unwrap().name()
        );
        turtle
            .write()
            .unwrap()
            .post_creation_init(&turtle_list, turtle_idx);
        
        let (cv_mtx, cv) = &*time_cv;

        loop {
            let mut turtle_open = turtle.write().unwrap();
            let next_time = turtle_open.next_time_step();
            if next_time > max_time {
                std::mem::drop(turtle_open);
                if Self::wait_the_end(&turtle, max_time, &cv_mtx, &cv, nb_turtles) {
                    break;
                }
                let mut turtle_open = turtle.write().unwrap();
                let next_time = turtle_open.next_time_step();
                info!("[{}] Return to time {next_time}", turtle_open.name());
                turtle_open.run_next_time_step(next_time);
            } else {
                turtle_open.run_next_time_step(next_time);
            }
        }
    }

    /// Compute the results from the file where it was saved before.
    ///
    /// If the [`Simulator`] config disabled the computation of the results, this function
    /// does nothing.
    fn compute_results(&self, results: Vec<Record>, config: &SimulatorConfig) {
        if !self.meta_config.compute_results {
            return;
        }

        info!("Starting result analyse...");
        let show_figures = !self.meta_config.no_gui;

        prepare_freethreaded_python();

        let json_results =
            serde_json::to_string(&results).expect("Error during converting results to json");
        let json_config =
            serde_json::to_string(&config).expect("Error during converting results to json");
        let json_metaconfig = serde_json::to_string(&self.meta_config)
            .expect("Error during converting results to json");

        let show_figure_py = r#"
import matplotlib.pyplot as plt

def show():
    plt.show()
"#;

        let convert_to_dict = r#"
import json
class NoneDict(dict):
    """ dict subclass that returns a value of None for missing keys instead
        of raising a KeyError. Note: doesn't add item to dictionary.
    """
    def __missing__(self, key):
        return None


def converter(decoded_dict):
    """ Convert any None values in decoded dict into empty NoneDict's. """
    return {k: NoneDict() if v is None else v for k,v in decoded_dict.items()}

def convert(records):
    return json.loads(records, object_hook=converter)
"#;

        let script_path = self.meta_config.analyse_script.clone().unwrap_or(
            Path::new(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/python_scripts/analyse_results.py"
            ))
            .into(),
        );
        let python_script = fs::read_to_string(script_path).expect("File not found");
        let res = Python::with_gil(|py| -> PyResult<()> {
            let script = PyModule::from_code_bound(py, &convert_to_dict, "", "")?;
            let convert_fn: Py<PyAny> = script.getattr("convert")?.into();
            let result_dict = convert_fn.call_bound(py, (json_results,), None)?;
            let config_dict = convert_fn.call_bound(py, (json_config,), None)?;
            let metaconfig_dict = convert_fn.call_bound(py, (json_metaconfig,), None)?;
            let script = PyModule::from_code_bound(py, &python_script, "", "")?;
            let analyse_fn: Py<PyAny> = script.getattr("analyse")?.into();
            info!("Analyse the results...");
            let res = analyse_fn.call_bound(
                py,
                (
                    result_dict,
                    config_dict,
                    metaconfig_dict,
                    Path::new(""),
                    ".pdf",
                ),
                None,
            );
            if let Err(err) = res {
                err.display(py);
                return Err(err);
            }
            if show_figures {
                info!("Showing figures...");
                let show_script = PyModule::from_code_bound(py, &show_figure_py, "", "")?;
                let show_fn: Py<PyAny> = show_script.getattr("show")?.into();
                show_fn.call_bound(py, (), None)?;
            }
            Ok(())
        });
        if let Some(err) = res.err() {
            error!("{}", err);
        }
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
            let mut simulator =
                Simulator::from_config_path(config_path, None, None, false, false, None);

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
