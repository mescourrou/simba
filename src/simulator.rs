/*!
Module serving the [`Simulator`] with the configuration and record structures.

The [`Simulator`] is the primary struct to be called to start the simulator,
the simulator can be used as follows:
```no_run
use std::path::Path;
use simba::simulator::Simulator;

fn main() {
    // Initialize the environment
    Simulator::init_environment();
    println!("Load configuration...");
    let mut simulator = Simulator::from_config_path(
        Path::new("config_example/config.yaml"),
        &None, //<- plugin API, to load external modules
    );

    // Show the simulator loaded configuration
    simulator.show();

    // Run the simulator for the time given in the configuration
    // It also save the results to json
    simulator.run();

    simulator.compute_results();
}

```


*/

// Configuration for Simulator
extern crate confy;
use config_checker::macros::Check;
use config_checker::ConfigCheckable;
use pyo3::prelude::*;
use serde_derive::{Deserialize, Serialize};

use crate::api::internal_api::NodeClient;
use crate::constants::TIME_ROUND;
use crate::errors::{SimbaError, SimbaResult};
use crate::logger::{init_log, is_enabled, LogLevel, LoggerConfig};
use crate::networking::network_manager::NetworkManager;
use crate::networking::service_manager::ServiceManager;
use crate::node_factory::{ComputationUnitConfig, NodeFactory, NodeRecord, RobotConfig};
use crate::plugin_api::PluginAPI;
use crate::state_estimators::state_estimator::State;
use crate::stateful::Stateful;
use crate::time_analysis::{self, TimeAnalysisConfig};
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;

use crate::node::Node;
use crate::utils::time_ordered_data::TimeOrderedData;
use core::f32;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::Duration;

use colored::Colorize;
use serde_json::{self, Value};
use std::default::Default;
use std::fs::{self, File};
use std::io::prelude::*;
use std::sync::{mpsc, Arc, Barrier, Condvar, Mutex, RwLock};
use std::thread::{self, sleep, ThreadId};

use log::{debug, error, info, warn};

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
    /// Default scenario configuration: no nodes.
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TimeMode {
    Centralized,
    Decentralized,
}

/// Scenario configuration for the simulator.
/// The Simulator configuration is the root of the scenario configuration.
///
/// This config contains an item, `nodes`, which list the nodes [`NodeConfig`].
///
/// ## Example in yaml:
/// ```ignore
/// nodes:
///     - NodeConfig 1
///     - NodeConfig 2
/// ```
///
///
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct SimulatorConfig {
    #[check]
    pub log: LoggerConfig,
    #[check]
    pub results: Option<ResultConfig>,

    pub base_path: Box<Path>,

    pub max_time: f32,
    pub time_mode: TimeMode,

    #[check]
    pub time_analysis: TimeAnalysisConfig,
    pub random_seed: Option<f32>,
    /// List of the robots to run, with their specific configuration.
    #[check]
    pub robots: Vec<RobotConfig>,
    #[check]
    pub computation_units: Vec<ComputationUnitConfig>,
}

impl Default for SimulatorConfig {
    /// Default scenario configuration: no nodes.
    fn default() -> Self {
        Self {
            log: LoggerConfig::default(),
            base_path: Box::from(Path::new(".")),
            results: None,
            time_analysis: TimeAnalysisConfig::default(),
            random_seed: None,
            robots: Vec::new(),
            computation_units: Vec::new(),
            max_time: 60.,
            time_mode: TimeMode::Centralized,
        }
    }
}

/// One time record of a node. The record is the state of the node with the
/// associated time.
///
/// This is a line for one node ([`NodeRecord`]) at a given time.
#[derive(Debug, Serialize, Deserialize)]
pub struct Record {
    /// Time of the record.
    pub time: f32,
    /// Record of a node.
    pub node: NodeRecord,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Results {
    pub config: SimulatorConfig,
    pub records: Vec<Record>,
}

static THREAD_IDS: RwLock<Vec<ThreadId>> = RwLock::new(Vec::new());
static THREAD_NAMES: RwLock<Vec<String>> = RwLock::new(Vec::new());
static THREAD_TIMES: RwLock<Vec<f32>> = RwLock::new(Vec::new());
static EXCLUDE_NODES: RwLock<Vec<String>> = RwLock::new(Vec::new());
static INCLUDE_NODES: RwLock<Vec<String>> = RwLock::new(Vec::new());

pub struct SimulatorAsyncApi {
    pub current_time: Arc<Mutex<HashMap<String, f32>>>,
    pub records: Arc<Mutex<mpsc::Receiver<Record>>>,
}

#[derive(Clone)]
struct SimulatorAsyncApiServer {
    pub current_time: Arc<Mutex<HashMap<String, f32>>>,
    pub records: mpsc::Sender<Record>,
}

#[derive(Debug)]
pub struct TimeCv {
    pub finished_nodes: Mutex<usize>,
    pub circulating_messages: Mutex<usize>,
    pub condvar: Condvar,
}

impl TimeCv {
    pub fn new() -> Self {
        Self {
            finished_nodes: Mutex::new(0),
            circulating_messages: Mutex::new(0),
            condvar: Condvar::new(),
        }
    }
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
/// ```no_run
/// use log::info;
/// use simba::simulator::Simulator;
/// use std::path::Path;
///
/// fn main() {
///
///     // Initialize the environment, essentially the logging part
///     Simulator::init_environment();
///     info!("Load configuration...");
///     let mut simulator = Simulator::from_config_path(
///         Path::new("config_example/config.yaml"), //<- configuration path
///         &None,                                      //<- plugin API, to load external modules
///     );
///
///     // Show the simulator loaded configuration
///     simulator.show();
///
///     // It also save the results to "result.json",
///     simulator.run();
///
///     // compute the results and show the figures.
///     simulator.compute_results();
///
/// }
///
/// ```
pub struct Simulator {
    /// List of the [`Node`]. Using `Arc` and `RwLock` for multithreading.
    nodes: Vec<Node>,
    /// Scenario configuration.
    config: SimulatorConfig,
    /// Network Manager
    network_manager: NetworkManager,
    /// Factory for components to make random variables generators
    determinist_va_factory: DeterministRandomVariableFactory,

    time_cv: Arc<TimeCv>,
    common_time: Option<Arc<Mutex<f32>>>,

    async_api: Option<Arc<SimulatorAsyncApi>>,
    async_api_server: Option<SimulatorAsyncApiServer>,

    node_apis: HashMap<String, NodeClient>,
}

impl Simulator {
    /// Create a new [`Simulator`] with no nodes, and empty config.
    pub fn new() -> Simulator {
        let rng = rand::random();
        let time_cv = Arc::new(TimeCv::new());
        Simulator {
            nodes: Vec::new(),
            config: SimulatorConfig::default(),
            network_manager: NetworkManager::new(time_cv.clone()),
            determinist_va_factory: DeterministRandomVariableFactory::new(rng),
            time_cv,
            async_api: None,
            async_api_server: None,
            common_time: Some(Arc::new(Mutex::new(f32::INFINITY))),
            node_apis: HashMap::new(),
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
        let mut simulator = Simulator::new();
        simulator.load_config(config, plugin_api);
        simulator
    }

    pub fn reset(&mut self, plugin_api: &Option<Box<&dyn PluginAPI>>) {
        self.nodes = Vec::new();
        self.time_cv = Arc::new(TimeCv::new());
        self.network_manager = NetworkManager::new(self.time_cv.clone());
        let config = self.config.clone();
        self.common_time = match &config.time_mode {
            TimeMode::Centralized => Some(Arc::new(Mutex::new(f32::INFINITY))),
            TimeMode::Decentralized => None,
        };
        let mut service_managers = HashMap::new();
        // Create robots
        for robot_config in &config.robots {
            self.add_robot(robot_config, plugin_api, &config);
            let node = self.nodes.last().unwrap();
            service_managers.insert(node.name(), node.service_manager());
        }
        // Create computation units
        for computation_unit_config in &config.computation_units {
            self.add_computation_unit(computation_unit_config, plugin_api, &config);
            let node = self.nodes.last().unwrap();
            service_managers.insert(node.name(), node.service_manager());
        }

        for node in self.nodes.iter_mut() {
            println!("Finishing initialization of {}", node.name());
            self.node_apis
                .insert(node.name(), node.post_creation_init(&service_managers));
        }
    }

    pub fn load_config_path(
        &mut self,
        config_path: &Path,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
    ) {
        println!("Load configuration from {:?}", config_path);
        let mut config: SimulatorConfig = match confy::load_path(config_path) {
            Ok(config) => config,
            Err(error) => {
                println!(
                    "ERROR: Error from Confy while loading the config file : {}",
                    error
                );
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
        println!("Checking configuration:");
        if config.check() {
            // Check network delay == 0
            let mut network_0 = false;
            for robot in &config.robots {
                if robot.network.reception_delay == 0. {
                    network_0 = true;
                    break;
                }
            }
            if !network_0 {
                for cu in &config.computation_units {
                    if cu.network.reception_delay == 0. {
                        network_0 = true;
                        break;
                    }
                }
            }
            if network_0 {
                println!("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\nWARNING: Network with 0 reception delay is not stable yet: messages can be treated later or deadlock can occur.\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            }
            println!("Config valid");
        } else {
            panic!("Error in config");
        }
        Self::init_log(&config.log);
        self.config = config.clone();
        if let Some(seed) = config.random_seed {
            self.determinist_va_factory.global_seed = seed;
        } else {
            self.config.random_seed = Some(self.determinist_va_factory.global_seed);
        }

        time_analysis::init_from_config(&self.config.time_analysis);

        self.reset(plugin_api);
    }

    pub fn config(&self) -> SimulatorConfig {
        self.config.clone()
    }

    /// Initialize the simulator environment.
    ///
    /// - start the logging environment.
    /// - Time analysis setup
    pub fn init_environment() {
        // env_logger::init();
        time_analysis::set_node_name("simulator".to_string());
    }

    fn init_log(log_config: &LoggerConfig) {
        init_log(log_config);
        THREAD_IDS.write().unwrap().push(thread::current().id());
        THREAD_NAMES.write().unwrap().push("simulator".to_string());
        THREAD_TIMES.write().unwrap().push(0.);
        EXCLUDE_NODES
            .write()
            .unwrap()
            .clone_from(&log_config.excluded_nodes);
        INCLUDE_NODES
            .write()
            .unwrap()
            .clone_from(&log_config.included_nodes);
        if log_config.included_nodes.len() > 0 {
            INCLUDE_NODES.write().unwrap().push("simulator".to_string());
        }
        if env_logger::builder()
            .target(env_logger::Target::Stdout)
            .format(|buf, record| {
                let thread_idx = THREAD_IDS
                    .read()
                    .unwrap()
                    .iter()
                    .position(|&x| x == thread::current().id())
                    .unwrap_or(0);
                let thread_name = THREAD_NAMES.read().unwrap()[thread_idx].clone();
                if EXCLUDE_NODES.read().unwrap().contains(&thread_name) {
                    return Ok(());
                }

                let included_nodes = INCLUDE_NODES.read().unwrap();
                if included_nodes.len() > 0 && !included_nodes.contains(&thread_name) {
                    return Ok(());
                }
                drop(included_nodes);
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
            .filter_level(log_config.log_level.clone().into())
            .try_init()
            .is_err()
        {
            println!("ERROR during log initialization!");
        } else {
            println!("Logging initialized at level: {}", log_config.log_level);
        }
    }

    /// Add a [`Node`] of type [`Robot`](NodeType::Robot) to the [`Simulator`].
    ///
    /// This function add the [`Node`] to the [`Simulator`] list and to the [`NetworkManager`].
    /// It also adds the [`NetworkManager`] to the new [`Node`].
    ///
    /// ## Argumants
    /// * `robot_config` - Configuration of the [`Node`].
    /// * `plugin_api` - Implementation of [`PluginAPI`] for the use of external modules.
    /// * `meta_config` - Configuration of the simulation run.
    fn add_robot(
        &mut self,
        robot_config: &RobotConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
    ) {
        self.nodes.push(NodeFactory::make_robot(
            robot_config,
            plugin_api,
            &global_config,
            &self.determinist_va_factory,
            self.time_cv.clone(),
        ));

        self.network_manager
            .register_node_network(self.nodes.last_mut().unwrap());
    }

    fn add_computation_unit(
        &mut self,
        computation_unit_config: &ComputationUnitConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
    ) {
        self.nodes.push(NodeFactory::make_computation_unit(
            computation_unit_config,
            plugin_api,
            &global_config,
            &self.determinist_va_factory,
            self.time_cv.clone(),
        ));

        self.network_manager
            .register_node_network(self.nodes.last_mut().unwrap());
    }

    /// Simply print the Simulator state, using the info channel and the debug print.
    pub fn show(&self) {
        println!("Simulator:");
        for node in self.nodes.iter() {
            println!("- {:?}", node);
        }
    }

    pub fn set_max_time(&mut self, max_time: f32) {
        self.config.max_time = max_time;
    }

    /// Run the scenario until the given time.
    ///
    /// This function starts one thread by [`Node`]. It waits that the thread finishes.
    ///
    /// After the scenario is done, the results are saved, and they are analysed, following
    /// the configuration give ([`SimulatorMetaConfig`]).
    pub fn run(&mut self) {
        let mut handles = vec![];
        let max_time = self.config.max_time;
        let nb_nodes = self.nodes.len();
        let barrier = Arc::new(Barrier::new(nb_nodes));
        let finishing_cv = Arc::new((Mutex::new(0usize), Condvar::new()));
        while let Some(node) = self.nodes.pop() {
            let i = self.nodes.len();
            let new_max_time = max_time.clone();
            let time_cv = self.time_cv.clone();
            let async_api_server = self.async_api_server.clone();
            let common_time = match &self.common_time {
                Some(ct) => Some(ct.clone()),
                None => None,
            };
            let finishing_cv_clone = finishing_cv.clone();
            let barrier_clone = barrier.clone();
            let handle = thread::spawn(move || -> SimbaResult<Node> {
                let ret = Self::run_one_node(
                    node,
                    new_max_time,
                    nb_nodes,
                    i,
                    time_cv,
                    async_api_server,
                    common_time,
                    barrier_clone,
                );
                *finishing_cv_clone.0.lock().unwrap() += 1;
                finishing_cv_clone.1.notify_all();
                ret
            });
            handles.push(handle);
        }

        self.simulator_spin(finishing_cv, nb_nodes);

        for handle in handles {
            self.nodes
                .push(handle.join().unwrap().expect("Node not returned"));
        }

        self.save_results();
    }

    /// Returns the list of all [`Record`]s produced by [`Simulator::run`].
    pub fn get_results(&self) -> Vec<Record> {
        let mut records = Vec::new();
        for node in self.nodes.iter() {
            let node_history = node.record_history();
            for (time, record) in node_history.iter() {
                records.push(Record {
                    time: time.clone(),
                    node: record.clone(),
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
                    node: row.node.clone(),
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

    /// Wait the end of the simulation. If other node send messages, the simulation
    /// will go back to the time of the last message received.
    fn wait_the_end(node: &Node, max_time: f32, time_cv: &TimeCv, nb_nodes: usize) -> bool {
        let mut lk = time_cv.finished_nodes.lock().unwrap();
        *lk += 1;
        if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
            debug!("Increase finished nodes: {}", *lk);
        }
        if *lk == nb_nodes {
            time_cv.condvar.notify_all();
            *lk = 0;
            return true;
        }
        loop {
            let buffered_msgs = node.process_messages();
            if buffered_msgs > 0 {
                *lk -= 1;
                if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
                    debug!("[wait_the_end] Messages to process: continue");
                }
                return false;
            }
            if is_enabled(crate::logger::InternalLog::NodeRunningDetailed) {
                debug!("[wait_the_end] Wait for others");
            }
            lk = time_cv.condvar.wait(lk).unwrap();
            let circulating_messages = time_cv.circulating_messages.lock().unwrap();
            if *lk == nb_nodes && *circulating_messages == 0 {
                return true;
            } else {
                *lk -= 1;
                node.process_messages();
                let next_time = node.next_time_step().0;
                if next_time < max_time {
                    return false;
                }
                *lk += 1;
            }
        }
    }
    /// Run the loop for the given `node` until reaching `max_time`.
    ///
    /// ## Arguments
    /// * `node` - Node to be run.
    /// * `max_time` - Time to stop the loop.
    fn run_one_node(
        mut node: Node,
        max_time: f32,
        nb_nodes: usize,
        node_idx: usize,
        time_cv: Arc<TimeCv>,
        async_api_server: Option<SimulatorAsyncApiServer>,
        common_time: Option<Arc<Mutex<f32>>>,
        barrier: Arc<Barrier>,
    ) -> SimbaResult<Node> {
        info!("Start thread of node {}", node.name());
        let mut thread_ids = THREAD_IDS.write().unwrap();
        thread_ids.push(thread::current().id());
        let thread_idx = thread_ids.len() - 1;
        THREAD_NAMES.write().unwrap().push(node.name());
        THREAD_TIMES.write().unwrap().push(0.);
        drop(thread_ids);
        time_analysis::set_node_name(node.name());

        let mut previous_time = 0.;
        loop {
            let (mut next_time, mut read_only) = node.next_time_step();
            if let Some(api) = &async_api_server {
                api.current_time
                    .lock()
                    .unwrap()
                    .insert(node.name(), next_time);
            }
            if let Some(common_time_arc) = &common_time {
                {
                    if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                        debug!("Get common time (next_time is {next_time})");
                    }
                    let mut unlocked_common_time = common_time_arc.lock().unwrap();
                    if *unlocked_common_time > next_time {
                        *unlocked_common_time = next_time;
                        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                            debug!("Set common time");
                        }
                    }
                }

                let mut lk = time_cv.finished_nodes.lock().unwrap();
                if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                    debug!("Got CV lock");
                }
                *lk += 1;
                time_cv.condvar.notify_all();
                if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                    debug!("Waiting for others... (next_time is {next_time})");
                }
                loop {
                    let buffered_msgs = node.process_messages();
                    if buffered_msgs > 0 {
                        *lk -= 1;
                        node.handle_messages(previous_time);
                        (next_time, read_only) = node.next_time_step();
                        {
                            let mut unlocked_common_time = common_time_arc.lock().unwrap();
                            if *unlocked_common_time > next_time {
                                *unlocked_common_time = next_time;
                                if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                                    debug!("Set common time at {next_time}");
                                }
                            }
                        }
                        *lk += 1;
                        time_cv.condvar.notify_all();
                    }
                    let circulating_messages = time_cv.circulating_messages.lock().unwrap();
                    if *lk == nb_nodes && *circulating_messages == 0 {
                        break;
                    } else if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                        debug!(
                            "Finished nodes = {}/{nb_nodes} and circulating messages = {}",
                            *lk, *circulating_messages
                        );
                    }
                    std::mem::drop(circulating_messages);
                    if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                        debug!("Wait CV");
                    }
                    lk = time_cv.condvar.wait(lk).unwrap();
                    if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                        debug!("End of CV wait");
                    }
                }
                if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                    debug!("Wait finished -- Release CV lock");
                }
                std::mem::drop(lk);

                next_time = *common_time_arc.lock().unwrap();
                if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                    debug!("Barrier... final next_time is {next_time}");
                }
                barrier.wait();
                *common_time_arc.lock().unwrap() = f32::INFINITY;
                *time_cv.finished_nodes.lock().unwrap() = 0;
                barrier.wait();
                if next_time > max_time {
                    break;
                }
            } else if next_time > max_time {
                if Self::wait_the_end(&node, max_time, &time_cv, nb_nodes) {
                    break;
                }
                (next_time, _) = node.next_time_step();
                info!("Return to time {next_time}");
            }

            let (own_next_time, own_read_only) = node.next_time_step();
            THREAD_TIMES.write().unwrap()[thread_idx] = next_time;
            if (own_next_time - next_time).abs() < TIME_ROUND {
                node.run_next_time_step(next_time, own_read_only);
                if own_read_only {
                    node.set_in_state(previous_time);
                } else {
                    if let Some(api) = &async_api_server {
                        let record = Record {
                            time: next_time,
                            node: node.record(),
                        };
                        api.records.send(record).unwrap();
                    }

                    previous_time = next_time;
                }
            } else {
                previous_time = next_time;
            }
        }
        Ok(node)
    }

    fn simulator_spin(&mut self, finishing_cv: Arc<(Mutex<usize>, Condvar)>, nb_nodes: usize) {
        // self.nodes is empty
        let mut node_states: HashMap<String, TimeOrderedData<State>> = HashMap::new();
        for (k, _) in self.node_apis.iter() {
            node_states.insert(k.clone(), TimeOrderedData::<State>::new());
        }
        loop {
            for (node_name, node_api) in self.node_apis.iter() {
                if let Some(state_update) = &node_api.state_update {
                    if let Ok((time, state)) = state_update.try_recv() {
                        node_states
                            .get_mut(node_name)
                            .expect(format!("Unknown node {node_name}").as_str())
                            .insert(time, state, true);
                    }
                }
            }
            self.network_manager.process_messages(&node_states);
            if *finishing_cv.0.lock().unwrap() == nb_nodes {
                return;
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
            let (records_tx, records_rx) = mpsc::channel();
            self.async_api_server = Some(SimulatorAsyncApiServer {
                current_time: Arc::clone(&map),
                records: records_tx,
            });
            self.async_api = Some(Arc::new(SimulatorAsyncApi {
                current_time: Arc::clone(&map),
                records: Arc::new(Mutex::new(records_rx)),
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

        let mut results: Vec<Vec<Record>> = Vec::new();

        for _i in 0..nb_replications {
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
