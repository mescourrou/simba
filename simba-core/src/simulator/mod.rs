/*!
Module serving the [`Simulator`] with the configuration and record structures.

The [`Simulator`] is the primary struct to be called to start the simulator,
the simulator can be used as follows:
```no_run
use std::path::Path;
use simba::simulator::Simulator;

// Initialize the environment
Simulator::init_environment();
println!("Load configuration...");
let mut simulator = Simulator::from_config_path(
    Path::new("config_example/config.yaml"),
    &None, //<- plugin API, to load external modules
).unwrap();

// Show the simulator loaded configuration
simulator.show();

// Run the simulator for the time given in the configuration
// It also save the results to json
simulator.run().unwrap();

simulator.compute_results().unwrap();

```


*/

mod results;
use results::ResultSavingData;
pub use results::{ResultConfig, ResultSaveMode, Results};

mod simulator_config;
pub use simulator_config::SimulatorConfig;

mod async_simulator;
use async_simulator::SimulatorAsyncApiServer;
pub use async_simulator::{AsyncSimulator, SimulatorAsyncApi};

extern crate confy;
use config_checker::ConfigCheckable;
use pyo3::{ffi::c_str, prelude::*};
use serde_derive::{Deserialize, Serialize};

use crate::{
    api::internal_api::NodeClient,
    constants::TIME_ROUND,
    errors::{SimbaError, SimbaErrorTypes, SimbaResult},
    logger::{init_log, is_enabled, LoggerConfig},
    networking::network_manager::NetworkManager,
    node::{
        node_factory::{ComputationUnitConfig, NodeFactory, NodeRecord, RobotConfig},
        Node,
    },
    plugin_api::PluginAPI,
    recordable::Recordable,
    state_estimators::State,
    time_analysis::{TimeAnalysisConfig, TimeAnalysisFactory},
    utils::{
        barrier::Barrier, determinist_random_variable::DeterministRandomVariableFactory,
        maths::round_precision, python::CONVERT_TO_DICT, time_ordered_data::TimeOrderedData,
    },
    VERSION,
};
use core::f32;
use std::{
    cmp::Ordering,
    path::{Path, PathBuf},
};
use std::{collections::BTreeMap, ffi::CString};

use colored::Colorize;
use serde_json;
use std::default::Default;
use std::fs::{self, File};
use std::io::prelude::*;
use std::sync::{Arc, Condvar, Mutex, RwLock};
use std::thread::{self, ThreadId};

use log::{debug, info, warn};

/// One time record of a node. The record is the state of the node with the
/// associated time.
///
/// This is a line for one node ([`NodeRecord`]) at a given time.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Record {
    /// Time of the record.
    pub time: f32,
    /// Record of a node.
    pub node: NodeRecord,
}

impl Ord for Record {
    fn cmp(&self, other: &Self) -> Ordering {
        if (self.time - other.time).abs() < TIME_ROUND {
            self.node.name().cmp(other.node.name())
        } else {
            self.time.total_cmp(&other.time)
        }
    }
}

impl PartialOrd for Record {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for Record {
    fn eq(&self, other: &Self) -> bool {
        if (self.time - other.time).abs() < TIME_ROUND {
            self.node.name().eq(other.node.name())
        } else {
            false
        }
    }
}

impl Eq for Record {}

static THREAD_IDS: RwLock<Vec<ThreadId>> = RwLock::new(Vec::new());
static THREAD_NAMES: RwLock<Vec<String>> = RwLock::new(Vec::new());
static TIME: RwLock<f32> = RwLock::new(0.);
static EXCLUDE_NODES: RwLock<Vec<String>> = RwLock::new(Vec::new());
static INCLUDE_NODES: RwLock<Vec<String>> = RwLock::new(Vec::new());

#[derive(Debug)]
pub struct TimeCv {
    pub waiting: Mutex<usize>,
    pub intermediate_parity: Mutex<u8>,
    pub circulating_messages: Mutex<usize>,
    pub force_finish: Mutex<bool>,
    pub condvar: Condvar,
}

impl TimeCv {
    pub fn new() -> Self {
        Self {
            waiting: Mutex::new(0),
            intermediate_parity: Mutex::new(0),
            circulating_messages: Mutex::new(0),
            force_finish: Mutex::new(false),
            condvar: Condvar::new(),
        }
    }
}

impl Default for TimeCv {
    fn default() -> Self {
        Self::new()
    }
}

/// This is the central structure which manages the run of the scenario.
///
/// To run the scenario, there are two mandatory steps:
/// * Load the config using [`Simulator::from_config_path`] (from a file), or using
///   [`Simulator::from_config`] with the [`SimulatorConfig`] structs directly.
/// * Run the scenario, once the config is loaded, the scenario can be run using
///   [`Simulator::run`].
///
/// Optionnal steps are the following:
/// * Initialize the environment with [`Simulator::init_environment`]. It initialize the logging environment.
/// * Use [`Simulator::show`] to print in the console the configuration loaded.
/// * Compute the results with [`Simulator::compute_results`].
/// * Get the records with [`Simulator::get_records`], providing the full list of all [`Record`]s.
///
/// ## Example
/// ```no_run
/// use log::info;
/// use simba::simulator::Simulator;
/// use std::path::Path;
///
/// // Initialize the environment, essentially the logging part
/// Simulator::init_environment();
/// info!("Load configuration...");
/// let mut simulator = Simulator::from_config_path(
///     Path::new("config_example/config.yaml"), //<- configuration path
///     &None,                                      //<- plugin API, to load external modules
/// ).unwrap();
///
/// // Show the simulator loaded configuration
/// simulator.show();
///
/// // It also save the results to "result.json",
/// simulator.run().unwrap();
///
/// // compute the results and show the figures.
/// simulator.compute_results().unwrap();
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
    determinist_va_factory: Arc<DeterministRandomVariableFactory>,

    time_cv: Arc<TimeCv>,
    common_time: Arc<RwLock<f32>>,

    async_api: Option<Arc<SimulatorAsyncApi>>,
    async_api_server: Option<SimulatorAsyncApiServer>,

    node_apis: BTreeMap<String, NodeClient>,

    result_saving_data: Option<ResultSavingData>,
    records: Vec<Record>,
    time_analysis_factory: TimeAnalysisFactory,
    force_send_results: bool,
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
            determinist_va_factory: Arc::new(DeterministRandomVariableFactory::new(rng)),
            time_cv,
            async_api: None,
            async_api_server: None,
            common_time: Arc::new(RwLock::new(f32::INFINITY)),
            node_apis: BTreeMap::new(),
            result_saving_data: Some(ResultSavingData::default()),
            records: Vec::new(),
            time_analysis_factory: TimeAnalysisFactory::init_from_config(
                &TimeAnalysisConfig::default(),
            )
            .unwrap(),
            force_send_results: false,
        }
    }

    /// Load the config from a file compatible with [`confy`]. Initialize the [`Simulator`].
    ///
    /// ## Arguments
    /// * `config_path` - `Path` to the config file (see example in `config_example/config.yaml`).
    /// * `plugin_api`  - Provide an implementation of [`PluginAPI`] if you want to use external modules.
    /// * `result_path` - Path to the file to save the results.
    /// * `compute_results` - Enable the computation of the results, using python script.
    /// * `no_gui` - Disable the GUI, and the opening of the figures.
    ///
    /// ## Return
    /// Returns a [`Simulator`] ready to be run.
    pub fn from_config_path(
        config_path: &Path,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
    ) -> SimbaResult<Simulator> {
        let mut sim = Simulator::new();
        sim.load_config_path(config_path, plugin_api)?;
        Ok(sim)
    }

    /// Load the config from structure instance.
    ///
    /// ## Arguments
    /// * `config` - Scenario configuration ([`SimulatorConfig`]).
    /// * `plugin_api`  - Provide an implementation of [`PluginAPI`] if you want to use external modules.
    ///
    /// ## Return
    /// Returns a [`Simulator`] ready to be run.
    pub fn from_config(
        config: &SimulatorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
    ) -> SimbaResult<Simulator> {
        let mut simulator = Simulator::new();
        simulator.load_config(config, plugin_api)?;
        Ok(simulator)
    }

    pub fn reset(&mut self, plugin_api: &Option<Arc<dyn PluginAPI>>) -> SimbaResult<()> {
        info!("Reset node");
        self.nodes = Vec::new();
        self.time_cv = Arc::new(TimeCv::new());
        self.network_manager = NetworkManager::new(self.time_cv.clone());
        let config = self.config.clone();
        self.common_time = Arc::new(RwLock::new(f32::INFINITY));

        self.time_analysis_factory = TimeAnalysisFactory::init_from_config(&config.time_analysis)?;

        if config.results.is_some() && self.async_api.is_none() {
            self.async_api = Some(self.get_async_api());
        }

        self.result_saving_data = self.config.results.as_ref().map(|cfg| ResultSavingData {
            save_mode: cfg.save_mode.clone(),
            ..Default::default()
        });

        let mut service_managers = BTreeMap::new();
        // Create robots
        for robot_config in &config.robots {
            self.add_robot(robot_config, plugin_api, &config, self.force_send_results);
            let node = self.nodes.last().unwrap();
            service_managers.insert(node.name(), node.service_manager());
        }
        // Create computation units
        for computation_unit_config in &config.computation_units {
            self.add_computation_unit(
                computation_unit_config,
                plugin_api,
                &config,
                self.force_send_results,
            );
            let node = self.nodes.last().unwrap();
            service_managers.insert(node.name(), node.service_manager());
        }

        for node in self.nodes.iter_mut() {
            info!("Finishing initialization of {}", node.name());
            self.node_apis
                .insert(node.name(), node.post_creation_init(&service_managers));
        }
        Ok(())
    }

    pub(crate) fn load_config_path(
        &mut self,
        config_path: &Path,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
    ) -> SimbaResult<()> {
        self.load_config_path_full(config_path, plugin_api, false)
    }

    pub(crate) fn load_config_path_full(
        &mut self,
        config_path: &Path,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        force_send_results: bool,
    ) -> SimbaResult<()> {
        println!("Load configuration from {:?}", config_path);
        let config = SimulatorConfig::load_from_path(config_path)?;
        self.load_config_full(&config, plugin_api, force_send_results)
    }

    pub fn load_config(
        &mut self,
        config: &SimulatorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
    ) -> SimbaResult<()> {
        self.load_config_full(config, plugin_api, false)
    }

    pub(crate) fn load_config_full(
        &mut self,
        config: &SimulatorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        force_send_results: bool,
    ) -> SimbaResult<()> {
        println!("Checking configuration...");
        Self::init_log(&config.log)?;
        match config.check() {
            Ok(_) => println!("Config valid"),
            Err(e) => {
                let e = SimbaError::new(
                    SimbaErrorTypes::ConfigError,
                    format!("Error in config:\n{e}"),
                );
                log::error!("{}", e.detailed_error());
                return Err(e);
            }
        };
        let config_version: Vec<usize> = config
            .version
            .split(".")
            .map(|s| s.parse().expect("Config version pattern not recognized"))
            .collect();
        if config_version.len() < 2 {
            return Err(SimbaError::new(
                SimbaErrorTypes::ConfigError,
                "Version is expected to be XX.YY at least".to_string(),
            ));
        }
        if config_version[0] != env!("CARGO_PKG_VERSION_MAJOR").parse::<usize>().unwrap()
            || config_version[1] != env!("CARGO_PKG_VERSION_MINOR").parse::<usize>().unwrap()
        {
            warn!(
                "Config major version ({}) differs from software version ({})",
                config.version, VERSION
            );
        }
        self.config = config.clone();
        if let Some(seed) = config.random_seed {
            self.determinist_va_factory.set_global_seed(seed);
        } else {
            self.config.random_seed = Some(self.determinist_va_factory.global_seed());
        }
        self.force_send_results = force_send_results;

        self.reset(plugin_api)
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

        Python::initialize();
    }

    fn init_log(log_config: &LoggerConfig) -> SimbaResult<()> {
        init_log(log_config);
        THREAD_IDS.write().unwrap().push(thread::current().id());
        THREAD_NAMES.write().unwrap().push("simulator".to_string());
        *TIME.write().unwrap() = 0.;
        EXCLUDE_NODES
            .write()
            .unwrap()
            .clone_from(&log_config.excluded_nodes);
        INCLUDE_NODES
            .write()
            .unwrap()
            .clone_from(&log_config.included_nodes);
        if !log_config.included_nodes.is_empty() {
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
                if !included_nodes.is_empty() && !included_nodes.contains(&thread_name) {
                    return Ok(());
                }
                drop(included_nodes);
                let time = TIME.read().unwrap();
                let time = format!("{:.4}", time) + ", ";
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
            warn!("Logger already initialized!");
        } else {
            println!("Logging initialized at level: {}", log_config.log_level);
        }
        Ok(())
    }

    /// Add a [`Node`] of type [`Robot`](crate::node_factory::NodeType::Robot) to the [`Simulator`].
    ///
    /// This function add the [`Node`] to the [`Simulator`] list and to the [`NetworkManager`].
    /// It also adds the [`NetworkManager`] to the new [`Node`].
    ///
    /// ## Argumants
    /// * `robot_config` - Configuration of the [`Robot`](crate::node_factory::NodeType::Robot).
    /// * `plugin_api` - Implementation of [`PluginAPI`] for the use of external modules.
    /// * `global_config` - Full configuration of the simulation.
    fn add_robot(
        &mut self,
        robot_config: &RobotConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        force_send_results: bool,
    ) {
        let mut new_node = NodeFactory::make_robot(
            robot_config,
            plugin_api,
            global_config,
            &self.determinist_va_factory,
            &mut self.time_analysis_factory,
            self.time_cv.clone(),
            force_send_results,
        );
        self.network_manager.register_node_network(&mut new_node);
        self.nodes.push(new_node);
    }

    fn add_computation_unit(
        &mut self,
        computation_unit_config: &ComputationUnitConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        force_send_results: bool,
    ) {
        let mut new_node = NodeFactory::make_computation_unit(
            computation_unit_config,
            plugin_api,
            global_config,
            &self.determinist_va_factory,
            &mut self.time_analysis_factory,
            self.time_cv.clone(),
            force_send_results,
        );
        self.network_manager.register_node_network(&mut new_node);
        self.nodes.push(new_node);
    }

    /// Simply print the Simulator state, using the info channel and the debug print.
    pub fn show(&self) {
        println!("Config:");
        println!("{:#?}", self.config);
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
    /// the configuration give ([`SimulatorConfig`]).
    pub fn run(&mut self) -> SimbaResult<()> {
        let mut handles = vec![];
        let max_time = self.config.max_time;
        let nb_nodes = Arc::new(RwLock::new(self.nodes.len()));
        let barrier = Arc::new(Barrier::new(self.nodes.len()));
        let finishing_cv = Arc::new((Mutex::new(0usize), Condvar::new()));

        if let Some(data) = &self.result_saving_data {
            match data.save_mode {
                ResultSaveMode::AtTheEnd => {}
                _ => self.prepare_save_results()?,
            }
        }
        while let Some(node) = self.nodes.pop() {
            let time_cv = self.time_cv.clone();
            let async_api_server = self.async_api_server.clone();
            let common_time_clone = self.common_time.clone();
            let finishing_cv_clone = finishing_cv.clone();
            let barrier_clone = barrier.clone();
            let nb_nodes = nb_nodes.clone();
            let handle = thread::spawn(move || -> SimbaResult<Option<Node>> {
                let ret = Self::run_one_node(
                    node,
                    max_time,
                    nb_nodes,
                    time_cv.clone(),
                    async_api_server,
                    common_time_clone,
                    barrier_clone,
                );
                match &ret {
                    Err(_) => *time_cv.force_finish.lock().unwrap() = true,
                    Ok(Some(_)) => {
                        // Increase finishing nodes only if the node is still existing
                        // as in case of zombie, the total number of node has been decreased.
                        *finishing_cv_clone.0.lock().unwrap() += 1;
                        finishing_cv_clone.1.notify_all();
                    }
                    _ => {}
                };

                ret
            });
            handles.push(handle);
        }

        let mut error = None;
        if let Err(e) = self.simulator_spin(finishing_cv, nb_nodes) {
            error = Some(e);
        }

        for handle in handles {
            match handle.join().unwrap() {
                Err(e) => error = Some(e),
                Ok(node) => {
                    if let Some(n) = node {
                        self.nodes.push(n)
                    }
                }
            };
        }

        if let Some(e) = error {
            self.process_records(None).map_err(|e2| {
                SimbaError::new(e2.error_type(), format!("Error while processing previous error.\nPrevious error: {}\nLast error: {}", e.detailed_error(), e2.detailed_error()))
            })?;
            return Err(e);
        }

        self.process_records(None)
    }

    /// Returns the list of all [`Record`]s produced by [`Simulator::run`].
    pub fn get_records(&self, sorted: bool) -> Vec<Record> {
        let mut records = self.records.clone();
        if sorted {
            records.sort();
        }
        records
    }

    fn prepare_save_results(&self) -> SimbaResult<()> {
        if self.config.results.is_none() {
            return Ok(());
        }
        let result_config = self.config.results.clone().unwrap();
        let filename = result_config.result_path;
        if filename.is_none() {
            return Ok(());
        }
        let filename = self.config.base_path.as_ref().join(filename.unwrap());

        info!(
            "Saving results to {}",
            filename.to_str().unwrap_or_default()
        );
        let mut recording_file = match File::create(filename.clone()) {
            Err(e) => {
                return Err(SimbaError::new(
                    SimbaErrorTypes::ConfigError,
                    format!(
                        "Impossible to create result file '{}': {}",
                        filename.to_str().unwrap(),
                        e
                    ),
                ));
            }
            Ok(f) => f,
        };

        recording_file.write_all(b"{\"config\": ").unwrap();
        if let Err(e) = serde_json::to_writer(&recording_file, &self.config) {
            return Err(SimbaError::new(
                SimbaErrorTypes::ImplementationError,
                format!("Error during json serialization of config: {e}"),
            ));
        }
        recording_file.write_all(b",\n\"records\": [\n").unwrap();
        Ok(())
    }

    /// Save the results to the file given during the configuration.
    ///
    /// If the configuration of the [`Simulator`] do not contain a result path, no results are saved.
    fn process_records(&mut self, time: Option<f32>) -> SimbaResult<()> {
        if self.config.results.is_none() {
            return Ok(());
        }
        let result_saving_data = self.result_saving_data.as_ref().unwrap().clone();
        if let ResultSaveMode::AtTheEnd = result_saving_data.save_mode {
            if time.is_some() {
                return Ok(());
            }
            self.prepare_save_results()?;
        }
        let result_saving_data = self.result_saving_data.as_mut().unwrap();
        match &mut result_saving_data.save_mode {
            ResultSaveMode::Batch(remaining_size) => {
                if time.is_some() {
                    if *remaining_size <= 1 {
                        *remaining_size = match self.config.results.as_ref().unwrap().save_mode {
                            ResultSaveMode::Batch(s) => s,
                            _ => {
                                return Err(SimbaError::new(
                        SimbaErrorTypes::ImplementationError,
                                    "Incoherence between configuration and loaded configuration for Result save_mode".to_string(),
                                ));
                            }
                        }
                    } else {
                        *remaining_size -= 1;
                        return Ok(());
                    }
                }
                // If no time is given, force save (for end)
            }
            ResultSaveMode::Periodic(next_save) => {
                if let Some(time) = time {
                    if *next_save <= time {
                        *next_save = match self.config.results.as_ref().unwrap().save_mode {
                            ResultSaveMode::Periodic(t) => {
                                round_precision(*next_save + t, TIME_ROUND).unwrap()
                            }
                            _ => {
                                return Err(SimbaError::new(
                        SimbaErrorTypes::ImplementationError,
                                    "Incoherence between configuration and loaded configuration for Result save_mode".to_string(),
                                ));
                            }
                        }
                    } else {
                        return Ok(());
                    }
                }
                // If no time is given, force save (for end)
            }
            ResultSaveMode::Continuous => (),
            ResultSaveMode::AtTheEnd => (),
        }

        if time.is_none() {
            // Only at the end
            self.time_analysis_factory.save_results();
        }

        let mut new_records = Vec::new();
        if let Some(async_api) = &self.async_api {
            while let Ok(record) = async_api.records.lock().unwrap().try_recv() {
                new_records.push(record)
            }
        }

        let result_config = self.config.results.clone().unwrap();
        let filename = result_config.result_path;
        if let Some(filename) = filename {
            let filename = self.config.base_path.as_ref().join(filename);

            info!(
                "Saving results to {}",
                filename.to_str().unwrap_or_default()
            );
            let mut recording_file = match File::options().append(true).open(filename.clone()) {
                Err(e) => {
                    return Err(SimbaError::new(
                        SimbaErrorTypes::ConfigError,
                        format!(
                            "Impossible to open result file '{}': {}",
                            filename.to_str().unwrap(),
                            e
                        ),
                    ));
                }
                Ok(f) => f,
            };

            for record in &new_records {
                if result_saving_data.first_row {
                    result_saving_data.first_row = false;
                } else {
                    recording_file.write_all(b",\n").unwrap();
                }
                if let Err(e) = serde_json::to_writer(&recording_file, &record) {
                    return Err(SimbaError::new(
                        SimbaErrorTypes::ImplementationError,
                        format!(
                            "Error during json serialization of record {:?}: {}",
                            &record, e
                        ),
                    ));
                }
            }
            if time.is_none() {
                // Only at the end. If crashes in between, the user need to close the json array+object manually
                recording_file.write_all(b"\n]}").unwrap();
            }
        }
        self.records.extend(new_records);
        Ok(())
    }

    pub fn load_results(&mut self) -> SimbaResult<f32> {
        if self.config.results.is_none() {
            return Err(SimbaError::new(
                SimbaErrorTypes::ConfigError,
                "Request for loading results but no result configuration".to_string(),
            ));
        }
        let result_config = self.config.results.clone().unwrap();
        let filename = result_config.result_path;
        if filename.is_none() {
            return Err(SimbaError::new(
                SimbaErrorTypes::ConfigError,
                "Request for loading results but no result path in configuration".to_string(),
            ));
        }
        let filename = self.config.base_path.as_ref().join(filename.unwrap());
        let mut recording_file = File::open(filename).expect("Impossible to open record file");
        let mut content = String::new();
        recording_file
            .read_to_string(&mut content)
            .expect("Impossible to read record file");

        let results: Results = serde_json::from_str(&content).expect("Error during json parsing");

        self.records = results.records;
        let mut max_time = self.common_time.write().unwrap();
        for record in &self.records {
            *max_time = max_time.max(record.time);
            self.async_api_server.as_ref().unwrap().send_record(record);
        }
        self.async_api_server
            .as_ref()
            .unwrap()
            .update_time(*max_time);
        Ok(*max_time)
    }

    /// Run the loop for the given `node` until reaching `max_time`.
    ///
    /// ## Arguments
    /// * `node` - Node to be run.
    /// * `max_time` - Time to stop the loop.
    fn run_one_node(
        mut node: Node,
        max_time: f32,
        nb_nodes: Arc<RwLock<usize>>,
        time_cv: Arc<TimeCv>,
        async_api_server: Option<SimulatorAsyncApiServer>,
        common_time: Arc<RwLock<f32>>,
        barrier: Arc<Barrier>,
    ) -> SimbaResult<Option<Node>> {
        info!("Start thread of node {}", node.name());
        let mut thread_ids = THREAD_IDS.write().unwrap();
        thread_ids.push(thread::current().id());
        THREAD_NAMES.write().unwrap().push(node.name());
        drop(thread_ids);
        let mut next_time = -1.;
        loop {
            if *time_cv.force_finish.lock().unwrap() {
                break;
            }
            next_time = node.next_time_step(next_time)?;
            if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                debug!("Got next_time: {next_time}");
            }

            {
                if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                    debug!("Get common time (next_time is {next_time})");
                }
                let mut unlocked_common_time = common_time.write().unwrap();
                if *unlocked_common_time > next_time {
                    *unlocked_common_time = next_time;
                    if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                        debug!("Set common time");
                    }
                }
            }
            barrier.wait();

            next_time = *common_time.read().unwrap();
            if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                debug!("Barrier... final next_time is {next_time}");
            }
            barrier.wait();
            *common_time.write().unwrap() = f32::INFINITY;
            barrier.wait();
            if let Some(async_api_server) = &async_api_server {
                async_api_server.update_time(next_time);
            }
            *TIME.write().unwrap() = next_time;
            if next_time > max_time {
                break;
            }

            let nb_nodes_unlocked = *nb_nodes.read().unwrap();
            node.run_next_time_step(next_time, &time_cv, nb_nodes_unlocked)?;
            node.sync_with_others(&time_cv, nb_nodes_unlocked, next_time);
            if node.send_records() {
                if let Some(async_api_server) = &async_api_server {
                    async_api_server.send_record(&Record {
                        time: next_time,
                        node: node.record(),
                    });
                }
            }
            if node.zombie() {
                // Only place where nb_node should change.
                if is_enabled(crate::logger::InternalLog::NodeRunning) {
                    debug!("Killing node {}", node.name());
                }
                if node.process_messages() > 0 {
                    node.handle_messages(next_time);
                }
                *nb_nodes.write().unwrap() -= 1;
                time_cv.condvar.notify_all();
                node.kill();
                barrier.remove_one();
                return Ok(None);
            }

            barrier.wait();
        }

        Ok(Some(node))
    }

    fn simulator_spin(
        &mut self,
        finishing_cv: Arc<(Mutex<usize>, Condvar)>,
        nb_nodes: Arc<RwLock<usize>>,
    ) -> SimbaResult<()> {
        // self.nodes is empty
        let mut node_states: BTreeMap<String, TimeOrderedData<State>> = BTreeMap::new();
        for (k, _) in self.node_apis.iter() {
            node_states.insert(k.clone(), TimeOrderedData::<State>::new());
        }
        let mut previous_time = *self.common_time.read().unwrap();
        loop {
            for (node_name, node_api) in self.node_apis.iter() {
                if let Some(state_update) = &node_api.state_update {
                    if let Ok((time, state)) = state_update.try_recv() {
                        node_states
                            .get_mut(node_name)
                            .unwrap_or_else(|| panic!("Unknown node {node_name}"))
                            .insert(time, state, true);
                    }
                }
            }
            if let Some(async_api) = &self.async_api {
                let current_time = *async_api.current_time.read().unwrap();
                if (current_time - previous_time).abs() >= TIME_ROUND {
                    previous_time = current_time;
                    self.process_records(Some(current_time))?;
                }
            }
            self.network_manager.process_messages(&node_states)?;
            if *finishing_cv.0.lock().unwrap() >= *nb_nodes.read().unwrap() {
                return Ok(());
            }
        }
    }

    pub fn compute_results(&self) -> SimbaResult<()> {
        let results = self.get_records(false);
        self._compute_results(results, &self.config)
    }

    /// Compute the results from the file where it was saved before.
    ///
    /// If the [`Simulator`] config disabled the computation of the results, this function
    /// does nothing.
    fn _compute_results(&self, results: Vec<Record>, config: &SimulatorConfig) -> SimbaResult<()> {
        if self.config.results.is_none()
            || self
                .config
                .results
                .as_ref()
                .unwrap()
                .analyse_script
                .is_none()
        {
            return Ok(());
        }
        let result_config = self.config.results.clone().unwrap();

        info!("Starting result analyse...");
        let show_figures = result_config.show_figures;

        let json_results =
            serde_json::to_string(&results).expect("Error during converting results to json");
        let json_config =
            serde_json::to_string(&config).expect("Error during converting results to json");

        let show_figure_py = cr#"
import matplotlib.pyplot as plt

def show():
    plt.show()
"#;

        let script_path = self
            .config
            .base_path
            .as_ref()
            .join(result_config.analyse_script.unwrap());
        let python_script = match fs::read_to_string(script_path.clone()) {
            Err(e) => {
                return Err(SimbaError::new(
                    SimbaErrorTypes::ConfigError,
                    format!(
                        "Result analyser script not found ({}): {}",
                        script_path.to_str().unwrap(),
                        e
                    ),
                ))
            }
            Ok(s) => CString::new(s).unwrap(),
        };
        let res = Python::attach(|py| -> PyResult<()> {
            let script = PyModule::from_code(py, CONVERT_TO_DICT, c_str!(""), c_str!(""))?;
            let convert_fn: Py<PyAny> = script.getattr("convert")?.into();
            let result_dict = convert_fn.call(py, (json_results,), None)?;
            let config_dict = convert_fn.call(py, (json_config,), None)?;
            let param_dict =
                convert_fn.call(py, (&result_config.python_params.to_string(),), None)?;

            let script = PyModule::from_code(py, &python_script, c_str!(""), c_str!(""))?;
            let analyse_fn: Py<PyAny> = script.getattr("analyse")?.into();
            info!("Analyse the results...");
            let figure_path;
            if let Some(p) = &result_config.figures_path {
                figure_path = self.config.base_path.as_ref().join(p);
                fs::create_dir_all(&figure_path).unwrap_or_else(|_| {
                    panic!(
                        "Impossible to create figure directory ({:#?})",
                        &figure_path
                    )
                });
            } else {
                figure_path = PathBuf::new();
            }
            let res = analyse_fn.call(
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
                let show_script = PyModule::from_code(py, show_figure_py, c_str!(""), c_str!(""))?;
                let show_fn: Py<PyAny> = show_script.getattr("show")?.into();
                show_fn.call(py, (), None)?;
            }
            Ok(())
        });
        if let Some(err) = res.err() {
            Err(SimbaError::new(
                SimbaErrorTypes::PythonError,
                err.to_string(),
            ))
        } else {
            Ok(())
        }
    }

    pub fn get_async_api(&mut self) -> Arc<SimulatorAsyncApi> {
        if self.async_api_server.is_none() {
            self.async_api_server = Some(SimulatorAsyncApiServer::new(0.));
        }
        Arc::new(self.async_api_server.as_mut().unwrap().new_client())
    }
}

impl Default for Simulator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use core::f32;
    use std::{path::Path, sync::mpsc::Sender};

    use serde_json::Value;

    use crate::{
        constants::TIME_ROUND,
        logger::LogLevel,
        networking::{
            message_handler::MessageHandler,
            network::{Envelope, MessageFlag},
        },
        sensors::Observation,
        state_estimators::{
            BenchStateEstimatorConfig, StateEstimator, StateEstimatorConfig, StateEstimatorRecord, WorldState, external_estimator::{ExternalEstimatorConfig, ExternalEstimatorRecord}, perfect_estimator::PerfectEstimatorConfig
        },
    };

    use super::*;

    #[test]
    fn replication_test() {
        let nb_replications = 10;

        let mut results: Vec<Vec<Record>> = Vec::new();

        for i in 0..nb_replications {
            print!("Run {}/{nb_replications} ... ", i + 1);
            let mut simulator =
                Simulator::from_config_path(Path::new("test_config/config.yaml"), &None).unwrap();

            simulator.run().unwrap();

            results.push(simulator.get_records(true));
            println!("OK");
        }

        let reference_result = &results[0];
        assert!(!reference_result.is_empty());
        for result in results.iter().skip(1) {
            assert_eq!(result.len(), reference_result.len());
            for (j, ref_result) in reference_result.iter().enumerate() {
                let result_as_str = format!("{:?}", result[j]);
                let reference_result_as_str = format!("{:?}", ref_result);
                assert_eq!(
                    result_as_str, reference_result_as_str,
                    "{result_as_str} != {reference_result_as_str}"
                );
            }
        }
    }

    #[derive(Debug)]
    struct StateEstimatorTest {
        pub last_time: f32,
        pub kill_time: f32,
    }

    impl StateEstimator for StateEstimatorTest {
        fn correction_step(
            &mut self,
            _node: &mut crate::node::Node,
            _observations: &[Observation],
            _time: f32,
        ) {
        }

        fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
            if time >= self.kill_time {
                node.network()
                    .as_ref()
                    .unwrap()
                    .write()
                    .unwrap()
                    .send_to(
                        "node2".to_string(),
                        Value::Null,
                        time,
                        vec![MessageFlag::Kill],
                    )
                    .unwrap();
                self.kill_time = f32::INFINITY;
            }
        }

        fn prediction_step(&mut self, _node: &mut crate::node::Node, time: f32) {
            self.last_time = time;
        }

        fn next_time_step(&self) -> f32 {
            round_precision(self.last_time + 0.1, TIME_ROUND).unwrap()
        }
        fn world_state(&self) -> WorldState {
            WorldState::new()
        }
    }

    impl Recordable<StateEstimatorRecord> for StateEstimatorTest {
        fn record(&self) -> StateEstimatorRecord {
            StateEstimatorRecord::External(ExternalEstimatorRecord {
                record: serde_json::Value::Null,
            })
        }
    }

    impl MessageHandler for StateEstimatorTest {
        fn get_letter_box(&self) -> Option<Sender<Envelope>> {
            None
        }
    }

    struct PluginAPITest {}

    impl PluginAPI for PluginAPITest {
        fn get_state_estimator(
            &self,
            config: &serde_json::Value,
            _global_config: &SimulatorConfig,
            _va_factory: &Arc<DeterministRandomVariableFactory>,
        ) -> Box<dyn StateEstimator> {
            Box::new(StateEstimatorTest {
                last_time: 0.,
                kill_time: config.as_number().unwrap().as_f64().unwrap() as f32,
            }) as Box<dyn StateEstimator>
        }
    }

    #[test]
    fn kill_node() {
        let kill_time = 5.;
        let mut config = SimulatorConfig::default();
        config.log.log_level = LogLevel::Off;
        // config.log.log_level = LogLevel::Internal(vec![crate::logger::InternalLog::All]);
        config.max_time = 10.;
        config.results = Some(ResultConfig::default());
        config.robots.push(RobotConfig {
            name: "node1".to_string(),
            state_estimator: StateEstimatorConfig::Perfect(PerfectEstimatorConfig {
                targets: vec!["self".to_string(), "node2".to_string(), "node3".to_string()],
                ..Default::default()
            }),
            state_estimator_bench: vec![BenchStateEstimatorConfig {
                name: "own".to_string(),
                config: StateEstimatorConfig::External(ExternalEstimatorConfig {
                    config: serde_json::to_value(kill_time).unwrap(),
                }),
            }],
            ..Default::default()
        });
        config.robots.push(RobotConfig {
            name: "node2".to_string(),
            state_estimator: StateEstimatorConfig::Perfect(PerfectEstimatorConfig {
                targets: vec!["self".to_string(), "node1".to_string(), "node3".to_string()],
                ..Default::default()
            }),
            ..Default::default()
        });
        config.robots.push(RobotConfig {
            name: "node3".to_string(),
            state_estimator: StateEstimatorConfig::Perfect(PerfectEstimatorConfig {
                targets: vec!["self".to_string(), "node1".to_string(), "node2".to_string()],
                ..Default::default()
            }),
            ..Default::default()
        });

        let plugin_api = PluginAPITest {};

        let mut simulator = Simulator::from_config(&config, &Some(Arc::new(plugin_api))).unwrap();

        simulator.run().unwrap();

        let records = simulator.get_records(false);
        let mut last_node2_time: f32 = 0.;
        for record in records {
            let t = record.time;
            if let NodeRecord::Robot(r) = record.node {
                if r.name.as_str() == "node2" {
                    last_node2_time = last_node2_time.max(t);
                }
            }
        }
        assert!(
            kill_time == last_node2_time,
            "Node not killed at right time (last time is {})",
            last_node2_time
        );
    }
}
