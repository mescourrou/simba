/*!
Module serving the [`Simulator`] with the configuration and record structures.

The [`Simulator`] is the primary struct to be called to start the simulator,
the simulator can be used as follows:
```
use std::path::Path;
use simba::simulator::Simulator;

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

    // It also save the results to "result.json",
    // compute the results and show the figures.
    simulator.run();
}


```


*/

// Configuration for Simulator
extern crate confy;
use config_checker::macros::Check;
use config_checker::ConfigCheckable;
use pyo3::prelude::*;
use serde_derive::{Deserialize, Serialize};

use crate::networking::network_manager::NetworkManager;
use crate::plugin_api::PluginAPI;
use crate::time_analysis::{self, TimeAnalysisConfig};
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;

use crate::robot::{Robot, RobotConfig, RobotRecord};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

use colored::Colorize;
use serde_json::{self, Value};
use std::default::Default;
use std::fs::{self, File};
use std::io::prelude::*;
use std::sync::{Arc, Condvar, Mutex, RwLock};
use std::thread::{self, ThreadId};

use log::{error, info};

use pyo3::prepare_freethreaded_python;

#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct ResultConfig {
    /// Filename to save the results, in JSON format. The directory of this
    /// file is used to save the figures if results are computed.
    pub result_path: Box<Path>,
    /// Show the matplotlib figures
    pub show_figures: bool,
    /// Path to the python analyse scrit.
    /// This script should have the following entry point:
    /// ```def analyse(result_data: Record, figure_path: str, figure_type: str)```
    /// If the option is none, the script is not run
    pub analyse_script: Option<Box<Path>>,
    pub figures_path: Option<Box<Path>>,
    pub python_params: Value,
}

impl Default for ResultConfig {
    /// Default scenario configuration: no robots.
    fn default() -> Self {
        Self {
            result_path: Box::from(Path::new("../results.json")),
            show_figures: true,
            analyse_script: None,
            figures_path: None,
            python_params: Value::Null,
        }
    }
}

/// Scenario configuration for the simulator.
/// The Simulator configuration is the root of the scenario configuration.
///
/// This config contains an item, `robots`, which list the robots [`RobotConfig`].
///
/// ## Example in yaml:
/// ```
/// robots:
///     - RobotConfig 1
///     - RobotConfig 2
/// ```
///
/// 
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct SimulatorConfig {
    pub results: Option<ResultConfig>,

    pub base_path: Box<Path>,

    pub max_time: f32,

    #[check]
    pub time_analysis: TimeAnalysisConfig,
    pub random_seed: Option<f32>,
    /// List of the robots to run, with their specific configuration.
    #[check]
    pub robots: Vec<Box<RobotConfig>>,
}

impl Default for SimulatorConfig {
    /// Default scenario configuration: no robots.
    fn default() -> Self {
        Self {
            base_path: Box::from(Path::new(".")),
            results: None,
            time_analysis: TimeAnalysisConfig::default(),
            random_seed: None,
            robots: Vec::new(),
            max_time: 60.,
        }
    }
}

/// One time record of a robot. The record is the state of the robot with the
/// associated time.
///
/// This is a line for one robot ([`RobotRecord`]) at a given time.
#[derive(Debug, Serialize, Deserialize)]
#[pyclass(get_all)]
pub struct Record {
    /// Time of the record.
    pub time: f32,
    /// Record of a robot.
    pub robot: RobotRecord,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Results {
    pub config: SimulatorConfig,
    pub records: Vec<Record>,
}

static THREAD_IDS: RwLock<Vec<ThreadId>> = RwLock::new(Vec::new());
static THREAD_NAMES: RwLock<Vec<String>> = RwLock::new(Vec::new());
static THREAD_TIMES: RwLock<Vec<f32>> = RwLock::new(Vec::new());
static EXCLUDE_ROBOTS: RwLock<Vec<String>> = RwLock::new(Vec::new());
static INCLUDE_ROBOTS: RwLock<Vec<String>> = RwLock::new(Vec::new());

pub struct SimulatorAsyncApi {
    pub current_time: Arc<Mutex<HashMap<String, f32>>>,
}

#[derive(Clone)]
struct SimulatorAsyncApiServer {
    pub current_time: Arc<Mutex<HashMap<String, f32>>>,
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
/// fn main() {
///
///     // Initialize the environment, essentially the logging part
///     Simulator::init_environment(log::LevelFilter::Debug);
///     info!("Load configuration...");
///     let mut simulator = Simulator::from_config_path(
///         Path::new("config_example/config.yaml"))), //<- configuration path
///         None,                                      //<- plugin API, to load external modules
///     );
///
///     // Show the simulator loaded configuration
///     simulator.show();
///
///     // It also save the results to "result.json",
///     // compute the results and show the figures.
///     simulator.run();
///
/// }
///
/// ```
pub struct Simulator {
    /// List of the [`Robot`]. Using `Arc` and `RwLock` for multithreading.
    robots: Arc<RwLock<Vec<Arc<RwLock<Robot>>>>>,
    /// Scenario configuration.
    config: SimulatorConfig,
    /// Network Manager
    network_manager: Arc<RwLock<NetworkManager>>,
    /// Factory for components to make random variables generators
    determinist_va_factory: DeterministRandomVariableFactory,

    time_cv: Arc<(Mutex<usize>, Condvar)>,

    async_api: Option<Arc<SimulatorAsyncApi>>,
    async_api_server: Option<SimulatorAsyncApiServer>,
}

impl Simulator {
    /// Create a new [`Simulator`] with no robots, and empty config.
    pub fn new() -> Simulator {
        let rng = rand::random();
        Simulator {
            robots: Arc::new(RwLock::new(Vec::new())),
            config: SimulatorConfig::default(),
            network_manager: Arc::new(RwLock::new(NetworkManager::new())),
            determinist_va_factory: DeterministRandomVariableFactory::new(rng),
            time_cv: Arc::new((Mutex::new(0), Condvar::new())),
            async_api: None,
            async_api_server: None,
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
        plugin_api: &Option<Box<&dyn PluginAPI>>,
    ) -> Simulator {
        let mut sim = Simulator::new();
        sim.load_config_path(config_path, plugin_api);
        sim
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
        plugin_api: &Option<Box<&dyn PluginAPI>>,
    ) -> Simulator {
        info!("Checking configuration:");
        if config.check() {
            info!("Config valid");
        } else {
            error!("Error in config");
            return Simulator::new();
        }
        let mut simulator = Simulator::new();
        simulator.load_config(config, plugin_api);
        simulator
    }

    pub fn reset(&mut self, plugin_api: &Option<Box<&dyn PluginAPI>>) {
        self.robots = Arc::new(RwLock::new(Vec::new()));
        self.network_manager = Arc::new(RwLock::new(NetworkManager::new()));
        let config = self.config.clone();
        self.time_cv = Arc::new((Mutex::new(0), Condvar::new()));
        // Create robots
        for robot_config in &config.robots {
            self.add_robot(robot_config, plugin_api, &config);
        }
    }

    pub fn load_config_path(
        &mut self,
        config_path: &Path,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
    ) {
        info!("Load configuration from {:?}", config_path);
        let mut config: SimulatorConfig = match confy::load_path(config_path) {
            Ok(config) => config,
            Err(error) => {
                error!("Error from Confy while loading the config file : {}", error);
                return;
            }
        };

        config.base_path = Box::from(config_path.parent().unwrap());
        config.time_analysis.output_path = config
            .base_path
            .as_ref()
            .join(&config.time_analysis.output_path)
            .to_str()
            .unwrap()
            .to_string();
        self.load_config(&config, plugin_api);
    }

    pub fn load_config(
        &mut self,
        config: &SimulatorConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
    ) {
        self.config = config.clone();
        if let Some(seed) = config.random_seed {
            self.determinist_va_factory.global_seed = seed;
        } else {
            self.config.random_seed = Some(self.determinist_va_factory.global_seed);
        }

        time_analysis::init_from_config(&self.config.time_analysis);

        self.reset(plugin_api);
    }

    /// Initialize the simulator environment.
    ///
    /// - start the logging environment.
    /// - Time analysis setup
    pub fn init_environment(
        level: log::LevelFilter,
        exclude_robots: Vec<String>,
        include_only: Vec<String>,
    ) {
        THREAD_IDS.write().unwrap().push(thread::current().id());
        THREAD_NAMES.write().unwrap().push("simulator".to_string());
        THREAD_TIMES.write().unwrap().push(0.);
        EXCLUDE_ROBOTS.write().unwrap().clone_from(&exclude_robots);
        INCLUDE_ROBOTS.write().unwrap().clone_from(&include_only);
        if include_only.len() > 0 {
            INCLUDE_ROBOTS
                .write()
                .unwrap()
                .push("simulator".to_string());
        }
        env_logger::builder()
            .target(env_logger::Target::Stdout)
            .format(|buf, record| {
                let thread_idx = THREAD_IDS
                    .read()
                    .unwrap()
                    .iter()
                    .position(|&x| x == thread::current().id())
                    .unwrap_or(0);
                let thread_name = THREAD_NAMES.read().unwrap()[thread_idx].clone();
                if EXCLUDE_ROBOTS.read().unwrap().contains(&thread_name) {
                    return Ok(());
                }

                let included_robots = INCLUDE_ROBOTS.read().unwrap();
                if included_robots.len() > 0 && !included_robots.contains(&thread_name) {
                    return Ok(());
                }
                drop(included_robots);
                let mut time = "".to_string();
                if thread_idx != 0 {
                    let time_f32 = THREAD_TIMES.read().unwrap()[thread_idx];
                    time = format!("{:.4}", time_f32) + ", ";
                };
                writeln!(
                    buf,
                    "[{:5}][{}{}] {}",
                    match record.level() {
                        log::Level::Error => "ERROR".red(),
                        log::Level::Warn => "WARN".yellow(),
                        log::Level::Info => "INFO".green(),
                        log::Level::Debug => "DEBUG".blue(),
                        log::Level::Trace => "TRACE".black(),
                    },
                    time,
                    &thread_name,
                    record.args()
                )
            })
            .format_timestamp(None)
            .format_module_path(false)
            .format_target(false)
            .filter_level(level)
            .init();
        time_analysis::set_robot_name("simulator".to_string());
    }

    /// Add a [`Robot`] to the [`Simulator`].
    ///
    /// This function add the [`Robot`] to the [`Simulator`] list and to the [`NetworkManager`].
    /// It also adds the [`NetworkManager`] to the new [`Robot`].
    ///
    /// ## Argumants
    /// * `robot_config` - Configuration of the [`Robot`].
    /// * `plugin_api` - Implementation of [`PluginAPI`] for the use of external modules.
    /// * `meta_config` - Configuration of the simulation run.
    fn add_robot(
        &mut self,
        robot_config: &RobotConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
    ) {
        self.robots.write().unwrap().push(Robot::from_config(
            robot_config,
            plugin_api,
            &global_config,
            &self.determinist_va_factory,
            self.time_cv.clone(),
        ));

        let robot_list = self.robots.read().unwrap();

        self.network_manager
            .write()
            .unwrap()
            .register_robot_network(Arc::clone(&robot_list.last().unwrap()));
        let last_robot_write = robot_list
            .last()
            .expect("No robot added to the vector, how is it possible ??")
            .write()
            .unwrap();
        last_robot_write
            .network()
            .write()
            .unwrap()
            .set_network_manager(Arc::clone(&self.network_manager));
    }

    /// Simply print the Simulator state, using the info channel and the debug print.
    pub fn show(&self) {
        println!("Simulator:");
        let robot_list = self.robots.read().unwrap();
        for robot in robot_list.iter() {
            println!("- {:?}", robot);
        }
    }

    pub fn set_max_time(&mut self, max_time: f32) {
        self.config.max_time = max_time;
    }

    /// Run the scenario until the given time.
    ///
    /// This function starts one thread by [`Robot`]. It waits that the thread finishes.
    ///
    /// After the scenario is done, the results are saved, and they are analysed, following
    /// the configuration give ([`SimulatorMetaConfig`]).
    pub fn run(&mut self) {
        let mut handles = vec![];
        let max_time = self.config.max_time;
        let mut i = 0;
        for robot in self.robots.read().unwrap().iter() {
            let new_robot = Arc::clone(robot);
            let new_max_time = max_time.clone();
            let robot_list = Arc::clone(&self.robots);
            let time_cv = self.time_cv.clone();
            let async_api_server = self.async_api_server.clone();
            let handle = thread::spawn(move || {
                Self::run_one_robot(
                    new_robot,
                    new_max_time,
                    robot_list,
                    i,
                    time_cv,
                    async_api_server,
                )
            });
            handles.push(handle);
            i += 1;
        }

        for handle in handles {
            handle.join().unwrap();
        }

        self.save_results();
    }

    /// Returns the list of all [`Record`]s produced by [`Simulator::run`].
    pub fn get_results(&self) -> Vec<Record> {
        let mut records = Vec::new();
        for robot in self.robots.read().unwrap().iter() {
            let robot_r = robot.read().expect("Robot cannot be read");
            let robot_history = robot_r.record_history();
            for (time, record) in robot_history.iter() {
                records.push(Record {
                    time: time.clone(),
                    robot: record.clone(),
                });
            }
        }
        records
    }

    /// Save the results to the file given during the configuration.
    ///
    /// If the configuration of the [`Simulator`] do not contain a result path, no results are saved.
    fn save_results(&mut self) {
        if self.config.results.is_none() {
            return;
        }
        let result_config = self.config.results.clone().unwrap();
        let filename = result_config.result_path;
        let filename = self.config.base_path.as_ref().join(filename);

        time_analysis::save_results();
        info!(
            "Saving results to {}",
            filename.to_str().unwrap_or_default()
        );
        let mut recording_file = File::create(filename).expect("Impossible to create record file");

        recording_file.write(b"{\"config\": ").unwrap();
        serde_json::to_writer(&recording_file, &self.config)
            .expect("Error during json serialization");
        recording_file.write(b",\n\"records\": [\n").unwrap();

        let results = self.get_results();

        let mut first_row = true;
        for row in &results {
            if first_row {
                first_row = false;
            } else {
                recording_file.write(b",\n").unwrap();
            }
            serde_json::to_writer(
                &recording_file,
                &Record {
                    time: row.time.clone(),
                    robot: row.robot.clone(),
                },
            )
            .expect("Error during json serialization");
        }
        recording_file.write(b"\n]}").unwrap();
    }

    pub fn load_results_and_analyse(&mut self) {
        if self.config.results.is_none() {
            return;
        }
        let result_config = self.config.results.clone().unwrap();
        let filename = result_config.result_path;
        let filename = self.config.base_path.as_ref().join(filename);
        let mut recording_file = File::open(filename).expect("Impossible to open record file");
        let mut content = String::new();
        recording_file
            .read_to_string(&mut content)
            .expect("Impossible to read record file");

        let results: Results = serde_json::from_str(&content).expect("Error during json parsing");

        self._compute_results(results.records, &results.config);
    }

    /// Wait the end of the simulation. If other robot send messages, the simulation
    /// will go back to the time of the last message received.
    fn wait_the_end(
        robot: &Arc<RwLock<Robot>>,
        max_time: f32,
        cv_mtx: &Mutex<usize>,
        cv: &Condvar,
        nb_robots: usize,
    ) -> bool {
        let mut finished_robot = cv_mtx.lock().unwrap();
        *finished_robot = *finished_robot + 1;
        if *finished_robot == nb_robots {
            cv.notify_all();
            return true;
        }
        loop {
            let buffered_msgs = robot.read().unwrap().process_messages();
            if buffered_msgs > 0 {
                *finished_robot = *finished_robot - 1;
                return false;
            }
            finished_robot = cv.wait(finished_robot).unwrap();
            if *finished_robot == nb_robots {
                return true;
            } else {
                *finished_robot = *finished_robot - 1;
                let robot_open = robot.read().unwrap();
                robot_open.process_messages();
                let next_time = robot_open.next_time_step().0;
                if next_time < max_time {
                    return false;
                }
                *finished_robot = *finished_robot + 1;
            }
        }
    }
    /// Run the loop for the given `robot` until reaching `max_time`.
    ///
    /// ## Arguments
    /// * `robot` - Robot to be run.
    /// * `max_time` - Time to stop the loop.
    fn run_one_robot(
        robot: Arc<RwLock<Robot>>,
        max_time: f32,
        robot_list: Arc<RwLock<Vec<Arc<RwLock<Robot>>>>>,
        robot_idx: usize,
        time_cv: Arc<(Mutex<usize>, Condvar)>,
        async_api_server: Option<SimulatorAsyncApiServer>,
    ) {
        info!("Start thread of robot {}", robot.read().unwrap().name());
        let mut thread_ids = THREAD_IDS.write().unwrap();
        thread_ids.push(thread::current().id());
        let thread_idx = thread_ids.len() - 1;
        THREAD_NAMES
            .write()
            .unwrap()
            .push(robot.read().unwrap().name());
        THREAD_TIMES.write().unwrap().push(0.);
        drop(thread_ids);
        time_analysis::set_robot_name(robot.read().unwrap().name());
        let nb_robots = robot_list.read().unwrap().len();
        info!("Finishing initialization");
        robot
            .write()
            .unwrap()
            .post_creation_init(&robot_list, robot_idx);

        let (cv_mtx, cv) = &*time_cv;

        let mut previous_time = 0.;
        loop {
            let mut robot_open = robot.write().unwrap();
            let (mut next_time, mut read_only) = robot_open.next_time_step();
            if let Some(api) = &async_api_server {
                api.current_time
                    .lock()
                    .unwrap()
                    .insert(robot_open.name(), next_time);
            }
            if next_time > max_time {
                std::mem::drop(robot_open);
                if Self::wait_the_end(&robot, max_time, &cv_mtx, &cv, nb_robots) {
                    break;
                }
                robot_open = robot.write().unwrap();
                (next_time, read_only) = robot_open.next_time_step();
                info!("Return to time {next_time}");
            }
            THREAD_TIMES.write().unwrap()[thread_idx] = next_time;
            robot_open.run_next_time_step(next_time, read_only);
            if read_only {
                robot_open.set_in_state(previous_time);
            } else {
                previous_time = next_time;
            }
        }
    }

    pub fn compute_results(&self) {
        let results = self.get_results();
        self._compute_results(results, &self.config);
    }

    /// Compute the results from the file where it was saved before.
    ///
    /// If the [`Simulator`] config disabled the computation of the results, this function
    /// does nothing.
    fn _compute_results(&self, results: Vec<Record>, config: &SimulatorConfig) {
        if self.config.results.is_none()
            || self
                .config
                .results
                .as_ref()
                .unwrap()
                .analyse_script
                .is_none()
        {
            return;
        }
        let result_config = self.config.results.clone().unwrap();

        info!("Starting result analyse...");
        let show_figures = result_config.show_figures;

        prepare_freethreaded_python();

        let json_results =
            serde_json::to_string(&results).expect("Error during converting results to json");
        let json_config =
            serde_json::to_string(&config).expect("Error during converting results to json");

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

        let script_path = self
            .config
            .base_path
            .as_ref()
            .join(&result_config.analyse_script.unwrap());
        let python_script = fs::read_to_string(script_path.clone())
            .expect(format!("File not found: {}", script_path.to_str().unwrap()).as_str());
        let res = Python::with_gil(|py| -> PyResult<()> {
            let script = PyModule::from_code_bound(py, &convert_to_dict, "", "")?;
            let convert_fn: Py<PyAny> = script.getattr("convert")?.into();
            let result_dict = convert_fn.call_bound(py, (json_results,), None)?;
            let config_dict = convert_fn.call_bound(py, (json_config,), None)?;
            let param_dict =
                convert_fn.call_bound(py, (&result_config.python_params.to_string(),), None)?;

            let script = PyModule::from_code_bound(py, &python_script, "", "")?;
            let analyse_fn: Py<PyAny> = script.getattr("analyse")?.into();
            info!("Analyse the results...");
            let figure_path;
            if let Some(p) = &result_config.figures_path {
                figure_path = self.config.base_path.as_ref().join(p);
                fs::create_dir_all(&figure_path).expect(
                    format!(
                        "Impossible to create figure directory ({:#?})",
                        &figure_path
                    )
                    .as_str(),
                );
            } else {
                figure_path = PathBuf::new();
            }
            let res = analyse_fn.call_bound(
                py,
                (result_dict, config_dict, figure_path, ".pdf", param_dict),
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

    pub fn get_async_api(&mut self) -> Arc<SimulatorAsyncApi> {
        if self.async_api_server.is_none() {
            let map = Arc::new(Mutex::new(HashMap::new()));
            self.async_api_server = Some(SimulatorAsyncApiServer {
                current_time: Arc::clone(&map),
            });
            self.async_api = Some(Arc::new(SimulatorAsyncApi {
                current_time: Arc::clone(&map),
            }));
        }
        self.async_api.as_ref().unwrap().clone()
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

        for i in 0..nb_replications {
            let mut simulator =
                Simulator::from_config_path(Path::new("config_example/config.yaml"), &None);

            simulator.show();

            simulator.run();

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
