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
    None, //<- plugin API, to load external modules
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
    VERSION,
    api::internal_api::NodeClient,
    constants::TIME_ROUND,
    errors::{SimbaError, SimbaErrorTypes, SimbaResult},
    logger::{LoggerConfig, init_log, is_enabled},
    networking::{network_manager::NetworkManager, service_manager::ServiceManager},
    node::{
        Node, NodeMetaData, NodeState,
        node_factory::{
            ComputationUnitConfig, MakeNodeParams, NodeFactory, NodeRecord, RobotConfig,
        },
    },
    plugin_api::PluginAPI,
    recordable::Recordable,
    scenario::{Scenario, config::ScenarioConfig},
    state_estimators::State,
    time_analysis::{TimeAnalysisConfig, TimeAnalysisFactory},
    utils::{
        SharedMutex, SharedRoLock, SharedRwLock,
        barrier::Barrier,
        determinist_random_variable::DeterministRandomVariableFactory,
        maths::round_precision,
        python::CONVERT_TO_DICT,
        rfc::{self, RemoteFunctionCall, RemoteFunctionCallHost},
        time_ordered_data::TimeOrderedData,
    },
};
use core::f32;
use std::{
    cmp::Ordering, fs::OpenOptions, io::SeekFrom, path::{Path, PathBuf}, thread::JoinHandle
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

pub(crate) struct RunningParameters {
    max_time: f32,
    nb_nodes: SharedRwLock<usize>,
    finishing_cv: Arc<(Mutex<usize>, Condvar)>,
    barrier: Arc<Barrier>,
    handles: Vec<JoinHandle<SimbaResult<Option<Node>>>>,
    end_time_step_sync_hosts: Vec<RemoteFunctionCallHost<(), ()>>,
    running_nodes_names: Vec<String>,
}

struct NodeSyncParams {
    nb_nodes: SharedRwLock<usize>,
    time_cv: Arc<TimeCv>,
    common_time: SharedRwLock<f32>,
    barrier: Arc<Barrier>,
    end_time_step_sync: RemoteFunctionCall<(), ()>,
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
///     None,                                      //<- plugin API, to load external modules
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
    common_time: SharedRwLock<f32>,

    async_api: Option<Arc<SimulatorAsyncApi>>,
    async_api_server: Option<SimulatorAsyncApiServer>,

    node_apis: BTreeMap<String, NodeClient>,

    result_saving_data: Option<ResultSavingData>,
    records: Vec<Record>,
    time_analysis_factory: Option<TimeAnalysisFactory>,
    force_send_results: bool,
    scenario: SharedMutex<Scenario>,
    plugin_api: Option<Arc<dyn PluginAPI>>,
    service_managers: BTreeMap<String, SharedRwLock<ServiceManager>>,
    meta_data_list: SharedRwLock<BTreeMap<String, SharedRoLock<NodeMetaData>>>,
}

impl Simulator {
    /// Create a new [`Simulator`] with no nodes, and empty config.
    pub fn new() -> Simulator {
        let rng = rand::random();
        let time_cv = Arc::new(TimeCv::new());
        let va_factory = Arc::new(DeterministRandomVariableFactory::new(rng));
        Simulator {
            nodes: Vec::new(),
            config: SimulatorConfig::default(),
            network_manager: NetworkManager::new(time_cv.clone()),
            determinist_va_factory: va_factory.clone(),
            time_cv,
            async_api: None,
            async_api_server: None,
            common_time: Arc::new(RwLock::new(f32::INFINITY)),
            node_apis: BTreeMap::new(),
            result_saving_data: Some(ResultSavingData::default()),
            records: Vec::new(),
            time_analysis_factory: Some(TimeAnalysisFactory::init_from_config(
                &TimeAnalysisConfig::default(),
            )
            .unwrap()),
            force_send_results: false,
            scenario: Arc::new(Mutex::new(Scenario::from_config(
                &ScenarioConfig::default(),
                &va_factory,
            ))),
            plugin_api: None,
            service_managers: BTreeMap::new(),
            meta_data_list: Arc::new(RwLock::new(BTreeMap::new())),
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
        plugin_api: Option<Arc<dyn PluginAPI>>,
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
        plugin_api: Option<Arc<dyn PluginAPI>>,
    ) -> SimbaResult<Simulator> {
        let mut simulator = Simulator::new();
        simulator.load_config(config, plugin_api)?;
        Ok(simulator)
    }

    pub fn reset(&mut self, plugin_api: Option<Arc<dyn PluginAPI>>) -> SimbaResult<()> {
        info!("Reset node");
        self.meta_data_list.write().unwrap().clear();
        self.nodes = Vec::new();
        self.time_cv = Arc::new(TimeCv::new());
        self.network_manager = NetworkManager::new(self.time_cv.clone());
        let config = self.config.clone();
        self.common_time = Arc::new(RwLock::new(f32::INFINITY));

        self.time_analysis_factory = match &config.time_analysis {
            Some(time_analysis) => Some(TimeAnalysisFactory::init_from_config(time_analysis)?),
            None => None,
        };

        if config.results.is_some() && self.async_api.is_none() {
            self.async_api = Some(self.get_async_api());
        }

        self.result_saving_data = self.config.results.as_ref().map(|cfg| ResultSavingData {
            save_mode: cfg.save_mode.clone(),
            ..Default::default()
        });

        self.plugin_api = plugin_api.clone();

        self.service_managers = BTreeMap::new();
        // Create robots
        for robot_config in &config.robots {
            self.add_robot(robot_config, &config, self.force_send_results, 0.)?;
            let node = self.nodes.last().unwrap();
            self.service_managers
                .insert(node.name(), node.service_manager());
        }
        // Create computation units
        for computation_unit_config in &config.computation_units {
            self.add_computation_unit(
                computation_unit_config,
                &config,
                self.force_send_results,
                0.,
            )?;
            let node = self.nodes.last().unwrap();
            self.service_managers
                .insert(node.name(), node.service_manager());
        }

        for node in self.nodes.iter_mut() {
            info!("Finishing initialization of {}", node.name());
            self.node_apis.insert(
                node.name(),
                node.post_creation_init(&self.service_managers, self.meta_data_list.clone()),
            );
        }

        self.scenario = Arc::new(Mutex::new(Scenario::from_config(
            &config.scenario,
            &self.determinist_va_factory,
        )));
        Ok(())
    }

    pub(crate) fn load_config_path(
        &mut self,
        config_path: &Path,
        plugin_api: Option<Arc<dyn PluginAPI>>,
    ) -> SimbaResult<()> {
        self.load_config_path_full(config_path, plugin_api, false)
    }

    pub(crate) fn load_config_path_full(
        &mut self,
        config_path: &Path,
        plugin_api: Option<Arc<dyn PluginAPI>>,
        force_send_results: bool,
    ) -> SimbaResult<()> {
        println!("Load configuration from {:?}", config_path);
        let config = SimulatorConfig::load_from_path(config_path)?;
        self.load_config_full(&config, plugin_api, force_send_results)
    }

    pub fn load_config(
        &mut self,
        config: &SimulatorConfig,
        plugin_api: Option<Arc<dyn PluginAPI>>,
    ) -> SimbaResult<()> {
        self.load_config_full(config, plugin_api, false)
    }

    pub(crate) fn load_config_full(
        &mut self,
        config: &SimulatorConfig,
        plugin_api: Option<Arc<dyn PluginAPI>>,
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
            .filter_module("tracing::span", log::LevelFilter::Off)
            .filter_module("winit", log::LevelFilter::Off)
            .filter_module("eframe", log::LevelFilter::Off)
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
        global_config: &SimulatorConfig,
        force_send_results: bool,
        initial_time: f32,
    ) -> SimbaResult<()> {
        let mut new_node = NodeFactory::make_robot(
            robot_config,
            &mut MakeNodeParams {
                plugin_api: &self.plugin_api,
                global_config,
                va_factory: &self.determinist_va_factory,
                time_analysis_factory: self.time_analysis_factory.as_mut(),
                time_cv: self.time_cv.clone(),
                force_send_results,
                new_name: None,
                initial_time,
            },
        )?;
        let meta_data = new_node.meta_data();
        let name = meta_data.read().unwrap().name.clone();
        self.meta_data_list.write().unwrap().insert(name, meta_data);
        if new_node.state() != NodeState::Running {
            return Ok(());
        }
        self.network_manager.register_node_network(&mut new_node);
        self.nodes.push(new_node);
        Ok(())
    }

    fn add_computation_unit(
        &mut self,
        computation_unit_config: &ComputationUnitConfig,
        global_config: &SimulatorConfig,
        force_send_results: bool,
        initial_time: f32,
    ) -> SimbaResult<()> {
        let mut new_node = NodeFactory::make_computation_unit(
            computation_unit_config,
            &mut MakeNodeParams {
                plugin_api: &self.plugin_api,
                global_config,
                va_factory: &self.determinist_va_factory,
                time_analysis_factory: self.time_analysis_factory.as_mut(),
                time_cv: self.time_cv.clone(),
                force_send_results,
                new_name: None,
                initial_time,
            },
        )?;
        let meta_data = new_node.meta_data();
        let name = meta_data.read().unwrap().name.clone();
        self.meta_data_list.write().unwrap().insert(name, meta_data);
        if new_node.state() != NodeState::Running {
            return Ok(());
        }
        self.network_manager.register_node_network(&mut new_node);
        self.nodes.push(new_node);
        Ok(())
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
        let mut running_parameters = RunningParameters {
            max_time: self.config.max_time,
            nb_nodes: Arc::new(RwLock::new(0)),
            finishing_cv: Arc::new((Mutex::new(0usize), Condvar::new())),
            barrier: Arc::new(Barrier::new(1)),
            handles: vec![],
            end_time_step_sync_hosts: Vec::new(),
            running_nodes_names: Vec::new(),
        };

        if let Some(data) = &self.result_saving_data {
            match data.save_mode {
                ResultSaveMode::AtTheEnd => {}
                _ => self.prepare_save_results()?,
            }
        }
        while let Some(node) = self.nodes.pop() {
            self.spawn_node(node, &mut running_parameters)?;
        }

        let mut error = None;
        running_parameters.barrier.wait();
        running_parameters.barrier.remove_one();
        if let Err(e) = self.simulator_spin(&mut running_parameters) {
            log::error!("Error in simulator spin: {}", e.detailed_error());
            error = Some(e);
            *self.time_cv.force_finish.lock().unwrap() = true;
        }

        for handle in running_parameters.handles.drain(0..) {
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

    #[cfg(not(feature = "force_hard_determinism"))]
    pub(crate) fn spawn_node_from_name(
        &mut self,
        node_name: &str,
        new_node_name: &str,
        running_parameters: &mut RunningParameters,
        time: f32,
    ) -> SimbaResult<()> {
        let mut node = NodeFactory::make_node_from_name(
            node_name,
            &mut MakeNodeParams {
                plugin_api: &self.plugin_api,
                global_config: &self.config,
                va_factory: &self.determinist_va_factory,
                time_analysis_factory: self.time_analysis_factory.as_mut(),
                time_cv: self.time_cv.clone(),
                force_send_results: self.force_send_results,
                new_name: Some(new_node_name),
                initial_time: time,
            },
        )?;
        node.set_state(NodeState::Running);
        self.network_manager.register_node_network(&mut node);
        self.service_managers
            .insert(node.name(), node.service_manager());
        self.node_apis.insert(
            node.name(),
            node.post_creation_init(&self.service_managers, self.meta_data_list.clone()),
        );
        self.spawn_node(node, running_parameters)
    }

    pub(crate) fn spawn_node(
        &mut self,
        node: Node,
        running_parameters: &mut RunningParameters,
    ) -> SimbaResult<()> {
        if running_parameters
            .running_nodes_names
            .contains(&node.name())
        {
            return Err(SimbaError::new(
                SimbaErrorTypes::ImplementationError,
                format!(
                    "Node with name '{}' is already running, cannot spawn another node with the same name",
                    node.name()
                ),
            ));
        }

        let max_time = running_parameters.max_time;
        let time_cv = self.time_cv.clone();
        let async_api_server = self.async_api_server.clone();
        let common_time_clone = self.common_time.clone();
        let finishing_cv_clone = running_parameters.finishing_cv.clone();
        let barrier_clone = running_parameters.barrier.clone();
        barrier_clone.add_one();
        let nb_nodes = running_parameters.nb_nodes.clone();
        *nb_nodes.write().unwrap() += 1;
        let (end_time_step_sync_tx, end_time_step_sync_rx) = rfc::make_pair();
        running_parameters
            .end_time_step_sync_hosts
            .push(end_time_step_sync_rx);
        running_parameters.running_nodes_names.push(node.name());
        let handle = thread::spawn(move || -> SimbaResult<Option<Node>> {
            let ret = Self::run_one_node(
                node,
                max_time,
                async_api_server,
                NodeSyncParams {
                    nb_nodes,
                    time_cv: time_cv.clone(),
                    common_time: common_time_clone,
                    barrier: barrier_clone,
                    end_time_step_sync: end_time_step_sync_tx,
                },
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
        running_parameters.handles.push(handle);
        Ok(())
    }

    /// Returns the list of all [`Record`]s produced by [`Simulator::run`].
    pub fn get_records(&self, sorted: bool) -> Vec<Record> {
        let mut records = self.records.clone();
        if sorted {
            records.sort();
        }
        records
    }

    fn prepare_save_results(&mut self) -> SimbaResult<()> {
        if self.config.results.is_none() {
            return Ok(());
        }
        let result_config = self.config.results.clone().unwrap();
        let filename = result_config.result_path;
        if filename.is_none() {
            return Ok(());
        }
        let filename = self.config.base_path.as_ref().join(filename.unwrap());

        if !self.records.is_empty() {
            // Results already started, just need to remove last line
            let mut file = match OpenOptions::new()
                .read(true)
                .write(true)
                .open(&filename) {
                    Err(e) => {
                        return Err(SimbaError::new(
                            SimbaErrorTypes::ConfigError,
                            format!(
                                "Impossible to open result file '{}': {}",
                                filename.to_str().unwrap(),
                                e
                            ),
                        ));
                    },
                    Ok(f) => f,
                };            
            let file_size = file.metadata().unwrap().len();
            
            let mut buffer = [0u8; 1];
            let mut pos = file_size.saturating_sub(1);
            
            while pos > 0 {
                file.seek(SeekFrom::Start(pos)).unwrap();
                file.read_exact(&mut buffer).unwrap();
                
                // Remove last newline
                // WARNING: assumes that the file does not end with newline, as generated by process_records
                if buffer[0] == b'\n' {
                    file.set_len(pos).unwrap();
                    return Ok(());
                }
                pos -= 1;
            }
            unreachable!("Result file should not be empty if records exist");
        }
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

        if time.is_none() && let Some(taf) = &mut self.time_analysis_factory {
            // Only at the end
            taf.save_results();
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

    #[deprecated(note = "Will be removed in future release. Use load_results_full instead")]
    pub fn load_results(&mut self) -> SimbaResult<f32> {
        self.load_results_full(None)
    }

    pub fn load_results_full(&mut self, filename: Option<String>) -> SimbaResult<f32> {
        if self.config.results.is_none() {
            return Err(SimbaError::new(
                SimbaErrorTypes::ConfigError,
                "Request for loading results but no result configuration".to_string(),
            ));
        }
        let result_config = self.config.results.clone().unwrap();
        let filename = filename.or(result_config.result_path);
        if filename.is_none() {
            return Err(SimbaError::new(
                SimbaErrorTypes::ConfigError,
                "Request for loading results but no result path in configuration".to_string(),
            ));
        }
        let filename = self.config.base_path.as_ref().join(filename.unwrap());
        let results = Self::deserialize_results_from_file(&filename)?;

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

    pub fn deserialize_results_from_file(filename: &Path) -> SimbaResult<Results> {
        info!("Loading results from file `{}`", filename.to_str().unwrap());
        let mut recording_file = File::open(filename).expect("Impossible to open record file");
        let mut content = String::new();
        recording_file
            .read_to_string(&mut content)
            .expect("Impossible to read record file");

        info!("Deserialize results...");
        Ok(serde_json::from_str(&content).expect("Error during json parsing"))
    }

    /// Run the loop for the given `node` until reaching `max_time`.
    ///
    /// ## Arguments
    /// * `node` - Node to be run.
    /// * `max_time` - Time to stop the loop.
    fn run_one_node(
        mut node: Node,
        max_time: f32,
        async_api_server: Option<SimulatorAsyncApiServer>,
        node_sync_params: NodeSyncParams,
    ) -> SimbaResult<Option<Node>> {
        if node.state() != NodeState::Running {
            return Err(SimbaError::new(
                SimbaErrorTypes::ImplementationError,
                format!(
                    "Node {} not in Running state at start of run_one_node",
                    node.name()
                ),
            ));
        }
        info!("Start thread of node {}", node.name());
        let mut thread_ids = THREAD_IDS.write().unwrap();
        thread_ids.push(thread::current().id());
        THREAD_NAMES.write().unwrap().push(node.name());
        drop(thread_ids);
        let mut next_time = -1.;
        node_sync_params.barrier.wait();
        node_sync_params.barrier.wait();
        loop {
            if *node_sync_params.time_cv.force_finish.lock().unwrap() {
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
                let mut unlocked_common_time = node_sync_params.common_time.write().unwrap();
                if *unlocked_common_time > next_time {
                    *unlocked_common_time = next_time;
                    if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                        debug!("Set common time");
                    }
                }
            }
            node_sync_params.barrier.wait();

            next_time = *node_sync_params.common_time.read().unwrap();
            if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                debug!("Barrier... final next_time is {next_time}");
            }
            node_sync_params.barrier.wait();
            *node_sync_params.common_time.write().unwrap() = f32::INFINITY;
            node_sync_params.barrier.wait();
            if let Some(async_api_server) = &async_api_server {
                async_api_server.update_time(next_time);
            }
            *TIME.write().unwrap() = next_time;
            if next_time > max_time {
                break;
            }

            let nb_nodes_unlocked = *node_sync_params.nb_nodes.read().unwrap();
            node.run_next_time_step(next_time, &node_sync_params.time_cv, nb_nodes_unlocked)?;
            node.sync_with_others(&node_sync_params.time_cv, nb_nodes_unlocked, next_time);
            if node.send_records()
                && let Some(async_api_server) = &async_api_server
            {
                async_api_server.send_record(&Record {
                    time: next_time,
                    node: node.record(),
                });
            }
            node_sync_params.end_time_step_sync.call(());
            node_sync_params.barrier.wait();
            if node.process_messages() > 0 {
                node.handle_messages(next_time);
            }
            if node.state() == NodeState::Zombie {
                info!("Killing node {}", node.name());
                if node.process_messages() > 0 {
                    node.handle_messages(next_time);
                }
                *node_sync_params.nb_nodes.write().unwrap() -= 1;
                node_sync_params.time_cv.condvar.notify_all();
                node.kill(next_time);
                node_sync_params.barrier.remove_one();
                return Ok(None);
            }

            node_sync_params.barrier.wait();
        }

        Ok(Some(node))
    }

    fn simulator_spin(&mut self, running_parameters: &mut RunningParameters) -> SimbaResult<()> {
        // self.nodes is empty
        let mut node_states: BTreeMap<String, TimeOrderedData<(State, NodeState)>> =
            BTreeMap::new();
        for (k, _) in self.node_apis.iter() {
            node_states.insert(k.clone(), TimeOrderedData::<(State, NodeState)>::new());
        }
        let mut waiting_nodes = 0;
        loop {
            for (node_name, node_api) in self.node_apis.iter() {
                if let Some(state_update) = &node_api.state_update
                    && let Ok((time, state)) = state_update.try_recv()
                {
                    if !node_states.contains_key(node_name) {
                        node_states.insert(
                            node_name.clone(),
                            TimeOrderedData::<(State, NodeState)>::new(),
                        );
                    }
                    if let Some(node_state) = node_states.get_mut(node_name) {
                        node_state.insert(time, state, true);
                    }
                }
            }
            let mut time_end_procedure = false;
            for end_time_step_sync in running_parameters.end_time_step_sync_hosts.iter() {
                end_time_step_sync.try_recv_closure_mut(|()| {
                    waiting_nodes += 1;
                    if waiting_nodes >= *running_parameters.nb_nodes.read().unwrap() {
                        running_parameters.barrier.add_one();
                        time_end_procedure = true;
                        waiting_nodes = 0;
                    }
                });
            }

            if time_end_procedure {
                let current_time = *TIME.read().unwrap();
                if let Err(e) = self.process_records(Some(current_time)) {
                    log::error!(
                        "Error in processing records at time {}: {}",
                        current_time,
                        e.detailed_error()
                    );
                    return Err(e);
                }
                let scenario = self.scenario.clone();
                scenario
                    .lock()
                    .unwrap()
                    .execute_scenario(current_time, self, &node_states, running_parameters)
                    .unwrap();
                if let Err(e) = self.network_manager.process_messages(&node_states) {
                    if let SimbaErrorTypes::NetworkError(_) = e.error_type() {
                        // Network errors are expected during the simulation, as nodes can go
                        // offline at any time.
                        warn!(
                            "Network error in processing network messages at time {}: {}",
                            current_time,
                            e.detailed_error()
                        );
                    } else {
                        log::error!(
                            "Error in processing network messages at time {}: {}",
                            current_time,
                            e.detailed_error()
                        );
                        return Err(e);
                    }
                }
                running_parameters.barrier.remove_one();
            }

            if let Err(e) = self.network_manager.process_messages(&node_states) {
                if let SimbaErrorTypes::NetworkError(_) = e.error_type() {
                    // Network errors are expected during the simulation, as nodes can go
                    // offline at any time.
                    warn!(
                        "Network error in processing network messages: {}",
                        e.detailed_error()
                    );
                } else {
                    return Err(e);
                }
            }
            if *running_parameters.finishing_cv.0.lock().unwrap()
                >= *running_parameters.nb_nodes.read().unwrap()
            {
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
                ));
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

    #[cfg(not(feature = "force_hard_determinism"))]
    pub(crate) fn get_network_manager_mut(&mut self) -> &mut NetworkManager {
        &mut self.network_manager
    }
}

impl Default for Simulator {
    fn default() -> Self {
        Self::new()
    }
}
