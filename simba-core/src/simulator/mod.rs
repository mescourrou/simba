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
pub use simulator_config::OptimizationConfig;
pub use simulator_config::SimulatorConfig;

mod async_simulator;
use async_simulator::SimulatorAsyncApiServer;
pub use async_simulator::{AsyncSimulator, SimulatorAsyncApi};

extern crate confy;
use config_checker::ConfigCheckable;
use pyo3::{ffi::c_str, prelude::*};
use serde_derive::{Deserialize, Serialize};

use simba_com::pub_sub::{PathBroker, PathMultiClient};

use crate::context::Context;
use crate::error;
use crate::info;
use crate::internal;
use crate::physics::Physics;
use crate::utils::SharedRoLock;
use crate::warning;
use crate::{
    VERSION,
    api::internal_api::NodeClient,
    constants::TIME_ROUND,
    environment::Environment,
    errors::{SimbaError, SimbaErrorTypes, SimbaResult},
    logger::LoggerConfig,
    networking::{network::Envelope, network_manager::NetworkManager},
    node::{
        Node, NodeState,
        node_factory::{
            ComputationUnitConfig, MakeNodeParams, NodeFactory, NodeRecord, RobotConfig,
        },
    },
    plugin_api::PluginAPI,
    recordable::Recordable,
    scenario::{Scenario, config::ScenarioConfig},
    time_analysis::{TimeAnalysisConfig, TimeAnalysisFactory},
    utils::{
        SharedMutex, SharedRwLock, barrier::Barrier,
        determinist_random_variable::DeterministRandomVariableFactory, maths::round_precision,
        python::CONVERT_TO_DICT,
    },
};
use core::f32;
use std::{
    cmp::Ordering,
    collections::HashMap,
    fs::OpenOptions,
    io::SeekFrom,
    path::{Path, PathBuf},
    thread::JoinHandle,
};
use std::{collections::BTreeMap, ffi::CString};

use serde_json;
use std::default::Default;
use std::fs::{self, File};
use std::io::prelude::*;
use std::sync::{Arc, Condvar, Mutex, RwLock};
use std::thread;

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

#[derive(Debug)]
/// Synchronization state shared across simulator and node threads.
pub struct TimeCv {
    /// Number of nodes currently waiting on synchronization.
    pub waiting: Mutex<usize>,
    /// Toggle used to separate successive waiting phases.
    pub intermediate_parity: Mutex<u8>,
    /// Number of messages still circulating in the current step.
    pub circulating_messages: Mutex<usize>,
    /// Flag forcing all threads to stop as soon as possible.
    pub force_finish: Mutex<bool>,
    /// Condition variable used to wake waiting threads.
    pub condvar: Condvar,
}

impl TimeCv {
    /// Create a fresh synchronization state.
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
    end_time_step_syncs: Vec<Arc<Mutex<bool>>>,
    running_nodes_names: Vec<String>,
    nb_threads: usize,
    last_sim_time: f32,
}

struct NodeSyncParams {
    nb_nodes: SharedRwLock<usize>,
    time_cv: Arc<TimeCv>,
    common_time: SharedRwLock<f32>,
    barrier: Arc<Barrier>,
    end_time_step_sync: Arc<Mutex<bool>>,
    physics_list: SharedRwLock<BTreeMap<String, SharedRoLock<Box<dyn Physics>>>>,
}

/// Broker type used by the simulator network.
pub type SimbaBroker = PathBroker<Envelope, String, Option<[f32; 2]>>;
/// Multi-client handle type associated with [`SimbaBroker`].
pub type SimbaBrokerMultiClient = PathMultiClient<Envelope, String>;

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
    max_threads: usize,

    async_api: Option<Arc<SimulatorAsyncApi>>,
    async_api_server: Option<SimulatorAsyncApiServer>,

    node_apis: BTreeMap<String, NodeClient>,
    physics_list: SharedRwLock<BTreeMap<String, SharedRoLock<Box<dyn Physics>>>>,

    result_saving_data: Option<ResultSavingData>,
    records: Vec<Record>,
    time_analysis_factory: Option<TimeAnalysisFactory>,
    force_send_results: bool,
    scenario: SharedMutex<Scenario>,
    plugin_api: Option<Arc<dyn PluginAPI>>,
    environment: Arc<Environment>,
    context: Context,
    last_sim_time: f32,
}

impl Simulator {
    /// Create a new [`Simulator`] with no nodes, and empty config.
    pub fn new() -> Simulator {
        let rng = rand::random();
        let time_cv = Arc::new(TimeCv::new());
        let va_factory = Arc::new(DeterministRandomVariableFactory::new(rng));
        let network_manager = NetworkManager::new();
        let broker = network_manager.broker().clone();
        #[cfg(feature = "monothreaded")]
        let max_threads = 1;
        #[cfg(not(feature = "monothreaded"))]
        let max_threads = 0;
        Simulator {
            nodes: Vec::new(),
            config: SimulatorConfig::default(),
            network_manager,
            determinist_va_factory: va_factory.clone(),
            time_cv,
            async_api: None,
            async_api_server: None,
            common_time: Arc::new(RwLock::new(f32::INFINITY)),
            node_apis: BTreeMap::new(),
            result_saving_data: Some(ResultSavingData::default()),
            physics_list: Arc::new(RwLock::new(BTreeMap::new())),
            records: Vec::new(),
            time_analysis_factory: Some(
                TimeAnalysisFactory::init_from_config(&TimeAnalysisConfig::default()).unwrap(),
            ),
            force_send_results: false,
            scenario: Arc::new(Mutex::new(Scenario::from_config(
                &ScenarioConfig::default(),
                &SimulatorConfig::default(),
                &va_factory,
                &broker,
            ))),
            plugin_api: None,
            environment: Arc::new(Environment::default()),
            max_threads,
            context: Context::new(&LoggerConfig::default(), Some(0.)),
            last_sim_time: -1.,
        }
    }

    /// Load the config from a file compatible with [`confy`], see [`SimulatorConfig`]. Initialize the [`Simulator`].
    ///
    /// ## Arguments
    /// * `config_path` - `Path` to the config file (see example in `config_example/config.yaml`).
    /// * `plugin_api`  - Provide an implementation of [`PluginAPI`] if you want to use external modules.
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

    /// Reset the simulator state from the currently loaded configuration.
    pub fn reset(&mut self, plugin_api: Option<Arc<dyn PluginAPI>>) -> SimbaResult<()> {
        info!(self.context, "Reset node");
        self.network_manager.reset();
        self.environment.clear_meta_data();
        self.nodes = Vec::new();
        self.time_cv = Arc::new(TimeCv::new());
        self.last_sim_time = -1.;
        let config = self.config.clone();
        self.common_time = Arc::new(RwLock::new(f32::INFINITY));
        self.max_threads = config.optimization.threads;

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
        self.records.clear();

        self.plugin_api = plugin_api.clone();

        self.environment = Arc::new(Environment::from_config(&config.environment, &config, &self.determinist_va_factory)?);

        // Create robots
        for robot_config in &config.robots {
            self.add_robot(
                robot_config,
                &config,
                self.force_send_results,
                0.,
                &self.context.clone(),
            )?;
            let node = self.nodes.last().unwrap();
            if let Some(physics) = node.physics() {
                self.physics_list
                    .write()
                    .unwrap()
                    .insert(node.name().clone(), physics);
            }
        }
        // Create computation units
        for computation_unit_config in &config.computation_units {
            self.add_computation_unit(
                computation_unit_config,
                &config,
                self.force_send_results,
                0.,
                &self.context.clone(),
            )?;
            let node = self.nodes.last().unwrap();
            if let Some(physics) = node.physics() {
                self.physics_list
                    .write()
                    .unwrap()
                    .insert(node.name().clone(), physics);
            }
        }

        self.scenario = Arc::new(Mutex::new(Scenario::from_config(
            &config.scenario,
            &config,
            &self.determinist_va_factory,
            &self.network_manager.broker(),
        )));

        for node in self.nodes.iter_mut() {
            info!(self.context, "Finishing initialization of {}", node.name());
            self.node_apis.insert(
                node.name(),
                node.post_creation_init(
                    self.physics_list.clone(),
                    self.environment.get_meta_data().clone(),
                    0.,
                    &self.context.clone(),
                ),
            );
        }

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

    /// Load a simulator configuration into an existing simulator instance.
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
        self.init_log(&config.log)?;
        match config.check() {
            Ok(_) => println!("Config valid"),
            Err(e) => {
                let e = SimbaError::new(
                    SimbaErrorTypes::ConfigError,
                    format!("Error in config:\n{e}"),
                );
                error!(Context::default(), "{}", e.detailed_error());
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
            warning!(
                Context::default(),
                "Config major version ({}) differs from software version ({})",
                config.version,
                VERSION
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

    /// Return a clone of the currently loaded configuration.
    pub fn config(&self) -> SimulatorConfig {
        self.config.clone()
    }

    /// Initialize the simulator environment.
    /// - initialize Python interpreter, to be able to run Python scripts in the simulator (for results analysis, or for Python nodes).
    pub fn init_environment() {
        // env_logger::init();

        Python::initialize();
    }

    fn init_log(&mut self, log_config: &LoggerConfig) -> SimbaResult<()> {
        self.context.update_config(log_config);
        self.context.update_time(0.);
        Ok(())
    }

    /// Add a [`Node`] of type [`Robot`](crate::node::node_factory::NodeType::Robot) to the [`Simulator`].
    ///
    /// This function add the [`Node`] to the [`Simulator`] list and to the [`NetworkManager`].
    /// It also adds the [`NetworkManager`] to the new [`Node`].
    ///
    /// ## Arguments
    /// * `robot_config` - Configuration of the [`Robot`](crate::node::node_factory::NodeType::Robot).
    /// * `global_config` - Full configuration of the simulation.
    /// * `force_send_results` - Whether the node should send results to the simulator even if the results are not used for the scenario or the analysis. This can be used to force the sending of results for a node, even if it is not required, for example to be able to show its state in the GUI.
    /// * `initial_time` - Initial time for the node, can be not null if the node is added during the simulation (for example in a scenario spawn event).
    fn add_robot(
        &mut self,
        robot_config: &RobotConfig,
        global_config: &SimulatorConfig,
        force_send_results: bool,
        initial_time: f32,
        context: &Context,
    ) -> SimbaResult<()> {
        let context = context.new_callstack_level(&format!("add_robot({})", robot_config.name));
        let new_node = NodeFactory::make_robot(
            robot_config,
            &mut MakeNodeParams {
                plugin_api: &self.plugin_api,
                global_config,
                va_factory: &self.determinist_va_factory,
                time_analysis_factory: self.time_analysis_factory.as_mut(),
                time_cv: self.time_cv.clone(),
                force_send_results,
                new_name: None,
                broker: &self.network_manager.broker(),
                initial_time,
                environment: self.environment.clone(),
                context,
            },
        )?;
        let meta_data = new_node.meta_data();
        let name = meta_data.read().unwrap().name.clone();
        self.environment.insert_meta_data(name, meta_data);
        if new_node.state() != NodeState::Running {
            return Ok(());
        }
        self.nodes.push(new_node);
        Ok(())
    }

    /// Add a [`Node`] of type [`ComputationUnit`](crate::node::node_factory::NodeType::ComputationUnit) to the [`Simulator`].
    ///
    /// This function add the [`Node`] to the [`Simulator`] list and to the [`NetworkManager`].
    /// It also adds the [`NetworkManager`] to the new [`Node`].
    ///
    /// ## Arguments
    /// * `computation_unit_config` - Configuration of the [`ComputationUnit`](crate::node::node_factory::NodeType::ComputationUnit).
    /// * `global_config` - Full configuration of the simulation.
    /// * `force_send_results` - Whether the node should send results to the simulator even if the results are not used for the scenario or the analysis. This can be used to force the sending of results for a node, even if it is not required, for example to be able to show its state in the GUI.
    /// * `initial_time` - Initial time for the node, can be not null if the node is added during the simulation (for example in a scenario spawn event).
    fn add_computation_unit(
        &mut self,
        computation_unit_config: &ComputationUnitConfig,
        global_config: &SimulatorConfig,
        force_send_results: bool,
        initial_time: f32,
        context: &Context,
    ) -> SimbaResult<()> {
        let context = context.new_callstack_level(&format!(
            "add_computation_unit({})",
            computation_unit_config.name
        ));
        let new_node = NodeFactory::make_computation_unit(
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
                broker: &self.network_manager.broker(),
                environment: self.environment.clone(),
                context,
            },
        )?;
        let meta_data = new_node.meta_data();
        let name = meta_data.read().unwrap().name.clone();
        self.environment.insert_meta_data(name, meta_data);
        if new_node.state() != NodeState::Running {
            return Ok(());
        }
        self.nodes.push(new_node);
        Ok(())
    }

    /// Simply print the Simulator state, using stdout.
    pub fn show(&self) {
        println!("Config:");
        self.show_config();
        self.show_state(None);
    }

    /// Show the configuration loaded
    pub fn show_config(&self) {
        println!("{:#?}", self.config);
    }

    /// Show simulator state
    pub fn show_state(&self, node_name: Option<&str>) {
        if let Some(node_name) = node_name {
            if let Some(node) = self.nodes.iter().find(|n| n.name() == node_name) {
                println!("State of node '{}':\n{:#?}", node_name, node);
            } else {
                println!("Node '{}' not found", node_name);
            }
        } else {
            println!(
                "Simulator at time {:.4}:",
                *self.common_time.read().unwrap()
            );
            for node in self.nodes.iter() {
                println!("- {:#?}", node);
            }
        }
    }

    /// Override the maximum simulated time for subsequent runs.
    pub fn set_max_time(&mut self, max_time: f32) {
        self.config.max_time = max_time;
    }

    /// Run the scenario until the given time.
    ///
    /// This function starts one thread by [`Node`]. It waits that the thread finishes.
    ///
    /// After the scenario is done, the results are not processed. Use [`Simulator::compute_results`] to process the results and compute the analysis.
    pub fn run(&mut self) -> SimbaResult<()> {
        let context = self.context.new_callstack_level("run");
        let mut running_parameters = RunningParameters {
            max_time: self.config.max_time,
            last_sim_time: self.last_sim_time,
            nb_nodes: Arc::new(RwLock::new(0)),
            finishing_cv: Arc::new((Mutex::new(0usize), Condvar::new())),
            barrier: Arc::new(Barrier::new(1)),
            handles: vec![],
            end_time_step_syncs: Vec::new(),
            running_nodes_names: Vec::new(),
            nb_threads: self.max_threads,
        };

        if let Some(data) = &self.result_saving_data {
            match data.save_mode {
                ResultSaveMode::AtTheEnd => {}
                _ => self.prepare_save_results()?,
            }
        }

        let mut error = None;
        if cfg!(feature = "monothreaded") || self.max_threads == 1 {
            let mut node_contexts = self
                .nodes
                .iter()
                .map(|node| (node.name(), context.new_node_context(node.name())))
                .collect::<HashMap<_, _>>();
            for node in self.nodes.iter_mut() {
                if node.state() != NodeState::Running {
                    return Err(SimbaError::new(
                        SimbaErrorTypes::ImplementationError,
                        format!(
                            "Node {} not in Running state at start of run_one_node",
                            node.name()
                        ),
                    ));
                }
                info!(node_contexts[&node.name()], "Start node {}", node.name());
            }
            let mut previous_time = self.last_sim_time;
            loop {
                let mut time = f32::INFINITY;
                // if *node_sync_params.time_cv.force_finish.lock().unwrap() {
                //     break;
                // }
                for node in self.nodes.iter_mut() {
                    match node.next_time_step(
                        previous_time + TIME_ROUND / 2.,
                        &node_contexts[&node.name()],
                    ) {
                        Ok(t) => {
                            if t < time {
                                time = t;
                                internal!(
                                    node_contexts[&node.name()],
                                    crate::logger::InternalLog::NodeSyncDetailed,
                                    "Node {} set next_time to {time}",
                                    node.name()
                                );
                            }
                        }
                        Err(e) => {
                            error = Some(e);
                            break;
                        }
                    }
                }
                if error.is_some() {
                    break;
                }
                internal!(
                    context,
                    crate::logger::InternalLog::NodeSyncDetailed,
                    "Got next_time: {time}"
                );

                if let Some(async_api_server) = &self.async_api_server {
                    async_api_server.update_time(time);
                }
                self.context.update_time(time);
                for context in node_contexts.values_mut() {
                    context.update_time(time);
                }
                if time > running_parameters.max_time {
                    break;
                }

                info!(context, "Run time {}", time);
                for node in self.nodes.iter_mut() {
                    node.process_messages();
                    node.physics_update(time, &node_contexts[&node.name()]);
                }
                let node_states = self.node_states();
                if let Err(e) = self
                    .network_manager
                    .process_messages(&node_states, &context)
                {
                    error = Some(e);
                    break;
                }
                for node in self.nodes.iter_mut() {
                    if node.process_messages() {
                        node.handle_messages(time, &node_contexts[&node.name()]);
                    }
                    node.pre_loop_hooks(time, &node_contexts[&node.name()]);
                }
                if let Err(e) = self
                    .network_manager
                    .process_messages(&node_states, &context)
                {
                    error = Some(e);
                    break;
                }
                let mut do_control_loops = Vec::new();
                for node in self.nodes.iter_mut() {
                    if node.process_messages() {
                        node.handle_messages(time, &node_contexts[&node.name()]);
                    }
                    do_control_loops.push(node.prediction_step(time, &node_contexts[&node.name()]));
                }
                if let Err(e) = self
                    .network_manager
                    .process_messages(&node_states, &context)
                {
                    error = Some(e);
                    break;
                }
                for node in self.nodes.iter_mut() {
                    if node.process_messages() {
                        node.handle_messages(time, &node_contexts[&node.name()]);
                    }
                    node.make_observations(time, &node_contexts[&node.name()]);
                }
                if let Err(e) = self
                    .network_manager
                    .process_messages(&node_states, &context)
                {
                    error = Some(e);
                    break;
                }
                for node in self.nodes.iter_mut() {
                    if node.process_messages() {
                        node.handle_messages(time, &node_contexts[&node.name()]);
                    }
                    node.correction_step(time, &node_contexts[&node.name()]);
                }

                if let Err(e) = self
                    .network_manager
                    .process_messages(&node_states, &context)
                {
                    error = Some(e);
                    break;
                }
                for (node, do_control_loop) in
                    self.nodes.iter_mut().zip(do_control_loops.into_iter())
                {
                    if node.process_messages() {
                        node.handle_messages(time, &node_contexts[&node.name()]);
                    }
                    node.nav_and_control_step(time, do_control_loop, &node_contexts[&node.name()]);
                    if node.process_messages() {
                        node.handle_messages(time, &node_contexts[&node.name()]);
                    }
                }

                internal!(
                    context,
                    crate::logger::InternalLog::NodeSyncDetailed,
                    "End of time step"
                );

                let node_states = self.node_states();
                if let Err(e) =
                    self.end_of_time_step_procedure(&node_states, &mut running_parameters, &context)
                {
                    error = Some(e);
                    break;
                }

                let mut to_remove = Vec::new();
                for (i, node) in self.nodes.iter_mut().enumerate() {
                    if node.send_records()
                        && let Some(async_api_server) = &self.async_api_server
                    {
                        async_api_server.send_record(&Record {
                            time,
                            node: node.record(&node_contexts[&node.name()]),
                        });
                    }

                    if node.process_messages() {
                        node.handle_messages(time, &node_contexts[&node.name()]);
                    }
                    if node.state() == NodeState::Zombie {
                        info!(context, "Killing node {}", node.name());
                        if node.process_messages() {
                            node.handle_messages(time, &node_contexts[&node.name()]);
                        }
                        node.kill(time);
                        self.physics_list.write().unwrap().remove(&node.name());
                        to_remove.push(i);
                    }
                }
                for i in to_remove.into_iter().rev() {
                    self.nodes.remove(i);
                }
                previous_time = time;
            }
        } else {
            #[cfg(not(feature = "monothreaded"))]
            {
                if self.max_threads == 0 {
                    while let Some(node) = self.nodes.pop() {
                        let node_context = context.new_node_context(node.name());
                        self.spawn_node(node, &mut running_parameters, &node_context)?;
                    }

                    running_parameters.barrier.wait();
                    running_parameters.barrier.remove_one();
                    if let Err(e) = self.simulator_spin(&mut running_parameters) {
                        error!(context, "Error in simulator spin: {}", e.detailed_error());
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
                } else {
                    use scoped_threadpool::Pool;

                    let mut pool = Pool::new(self.max_threads as u32);
                    let mut node_contexts = self
                        .nodes
                        .iter()
                        .map(|node| (node.name(), context.new_node_context(node.name())))
                        .collect::<HashMap<_, _>>();
                    for node in self.nodes.iter_mut() {
                        if node.state() != NodeState::Running {
                            return Err(SimbaError::new(
                                SimbaErrorTypes::ImplementationError,
                                format!(
                                    "Node {} not in Running state at start of run_one_node",
                                    node.name()
                                ),
                            ));
                        }
                        info!(node_contexts[&node.name()], "Start node {}", node.name());
                    }
                    let mut previous_time = self.last_sim_time;
                    loop {
                        let mut time = f32::INFINITY;
                        // if *node_sync_params.time_cv.force_finish.lock().unwrap() {
                        //     break;
                        // }
                        for node in self.nodes.iter_mut() {
                            match node.next_time_step(
                                previous_time + TIME_ROUND / 2.,
                                &node_contexts[&node.name()],
                            ) {
                                Ok(t) => {
                                    if t < time {
                                        time = t;
                                        internal!(
                                            node_contexts[&node.name()],
                                            crate::logger::InternalLog::NodeSyncDetailed,
                                            "Node {} set next_time to {time}",
                                            node.name()
                                        );
                                    }
                                }
                                Err(e) => {
                                    error = Some(e);
                                    break;
                                }
                            }
                        }
                        if error.is_some() {
                            break;
                        }
                        internal!(
                            context,
                            crate::logger::InternalLog::NodeSyncDetailed,
                            "Got next_time: {time}"
                        );

                        if let Some(async_api_server) = &self.async_api_server {
                            async_api_server.update_time(time);
                        }
                        self.context.update_time(time);
                        for context in node_contexts.values_mut() {
                            context.update_time(time);
                        }
                        if time > running_parameters.max_time {
                            break;
                        }

                        info!(context, "Run time {}", time);
                        pool.scoped(|scope| {
                            for node in self.nodes.iter_mut() {
                                let node_context = node_contexts[&node.name()].clone();
                                scope.execute(move || {
                                    node.process_messages();
                                    node.physics_update(time, &node_context);
                                });
                            }
                        });
                        let node_states = self.node_states();
                        if let Err(e) = self
                            .network_manager
                            .process_messages(&node_states, &context)
                        {
                            error = Some(e);
                            break;
                        }
                        pool.scoped(|scope| {
                            for node in self.nodes.iter_mut() {
                                let node_context = node_contexts[&node.name()].clone();
                                scope.execute(move || {
                                    if node.process_messages() {
                                        node.handle_messages(time, &node_context);
                                    }
                                    node.pre_loop_hooks(time, &node_context);
                                });
                            }
                        });
                        if let Err(e) = self
                            .network_manager
                            .process_messages(&node_states, &context)
                        {
                            error = Some(e);
                            break;
                        }
                        let mut do_control_loops = Vec::new();
                        pool.scoped(|scope| {
                            let (tx, rx) = std::sync::mpsc::channel();
                            let node_len = self.nodes.len();
                            for (i, node) in self.nodes.iter_mut().enumerate() {
                                do_control_loops.push(false);
                                let node_context = node_contexts[&node.name()].clone();
                                let tx = tx.clone();
                                scope.execute(move || {
                                    if node.process_messages() {
                                        node.handle_messages(time, &node_context);
                                    }
                                    tx.send((i, node.prediction_step(time, &node_context)))
                                        .unwrap();
                                });
                            }
                            for (i, do_or_do_not) in rx.iter().take(node_len) {
                                do_control_loops[i] = do_or_do_not;
                            }
                        });
                        if let Err(e) = self
                            .network_manager
                            .process_messages(&node_states, &context)
                        {
                            error = Some(e);
                            break;
                        }
                        pool.scoped(|scope| {
                            for node in self.nodes.iter_mut() {
                                let node_context = node_contexts[&node.name()].clone();
                                scope.execute(move || {
                                    if node.process_messages() {
                                        node.handle_messages(time, &node_context);
                                    }
                                    node.make_observations(time, &node_context);
                                });
                            }
                        });
                        if let Err(e) = self
                            .network_manager
                            .process_messages(&node_states, &context)
                        {
                            error = Some(e);
                            break;
                        }
                        pool.scoped(|scope| {
                            for node in self.nodes.iter_mut() {
                                let node_context = node_contexts[&node.name()].clone();
                                scope.execute(move || {
                                    if node.process_messages() {
                                        node.handle_messages(time, &node_context);
                                    }
                                    node.correction_step(time, &node_context);
                                });
                            }
                        });

                        if let Err(e) = self
                            .network_manager
                            .process_messages(&node_states, &context)
                        {
                            error = Some(e);
                            break;
                        }

                        pool.scoped(|scope| {
                            for (node, do_control_loop) in
                                self.nodes.iter_mut().zip(do_control_loops.into_iter())
                            {
                                let node_context = node_contexts[&node.name()].clone();
                                scope.execute(move || {
                                    if node.process_messages() {
                                        node.handle_messages(time, &node_context);
                                    }
                                    node.nav_and_control_step(time, do_control_loop, &node_context);
                                    if node.process_messages() {
                                        node.handle_messages(time, &node_context);
                                    }
                                });
                            }
                        });

                        internal!(
                            context,
                            crate::logger::InternalLog::NodeSyncDetailed,
                            "End of time step"
                        );

                        let node_states = self.node_states();
                        if let Err(e) = self.end_of_time_step_procedure(
                            &node_states,
                            &mut running_parameters,
                            &context,
                        ) {
                            error = Some(e);
                            break;
                        }

                        let mut to_remove = Vec::new();
                        for (i, node) in self.nodes.iter_mut().enumerate() {
                            if node.send_records()
                                && let Some(async_api_server) = &self.async_api_server
                            {
                                async_api_server.send_record(&Record {
                                    time,
                                    node: node.record(&node_contexts[&node.name()]),
                                });
                            }

                            if node.process_messages() {
                                node.handle_messages(time, &node_contexts[&node.name()]);
                            }
                            if node.state() == NodeState::Zombie {
                                info!(context, "Killing node {}", node.name());
                                if node.process_messages() {
                                    node.handle_messages(time, &node_contexts[&node.name()]);
                                }
                                node.kill(time);
                                self.physics_list.write().unwrap().remove(&node.name());
                                to_remove.push(i);
                            }
                        }
                        for i in to_remove.into_iter().rev() {
                            self.nodes.remove(i);
                        }
                        previous_time = time;
                    }
                }
            }
        }

        if let Some(e) = error {
            self.process_records(None, &context).map_err(|e2| {
                SimbaError::new(e2.error_type(), format!("Error while processing previous error.\nPrevious error: {}\nLast error: {}", e.detailed_error(), e2.detailed_error()))
            })?;
            return Err(e);
        }

        self.process_records(None, &context)
    }

    pub(crate) fn spawn_node_from_name(
        &mut self,
        node_name: &str,
        new_node_name: &str,
        running_parameters: &mut RunningParameters,
        time: f32,
        context: &Context,
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
                broker: &self.network_manager.broker(),
                environment: self.environment.clone(),
                context: context
                    .new_callstack_level(&format!("spawn_node_from_name({})", node_name)),
            },
        )?;
        let meta_data = node.meta_data();
        let name = meta_data.read().unwrap().name.clone();
        self.environment.insert_meta_data(name.clone(), meta_data);

        node.set_state(NodeState::Running);

        self.physics_list
            .write()
            .unwrap()
            .insert(node.name().clone(), node.physics().unwrap());

        self.node_apis.insert(
            name.clone(),
            node.post_creation_init(
                self.physics_list.clone(),
                self.environment.get_meta_data().clone(),
                time,
                context,
            ),
        );
        if cfg!(feature = "monothreaded") || running_parameters.nb_threads == 1 {
            self.nodes.push(node);
            Ok(())
        } else {
            #[cfg(feature = "monothreaded")]
            panic!("spawn_node_from_name should not be called in monothreaded mode");
            #[cfg(not(feature = "monothreaded"))]
            self.spawn_node(node, running_parameters, context)
        }
    }

    #[cfg(not(feature = "monothreaded"))]
    pub(crate) fn spawn_node(
        &mut self,
        node: Node,
        running_parameters: &mut RunningParameters,
        context: &Context,
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
        internal!(
            context,
            crate::logger::InternalLog::NodeSyncDetailed,
            "Spawning node {}",
            node.name()
        );

        let max_time = running_parameters.max_time;
        let last_sim_time = running_parameters.last_sim_time;
        let time_cv = self.time_cv.clone();
        let async_api_server = self.async_api_server.clone();
        let common_time_clone = self.common_time.clone();
        let finishing_cv_clone = running_parameters.finishing_cv.clone();
        let barrier_clone = running_parameters.barrier.clone();
        barrier_clone.add_one();
        let nb_nodes = running_parameters.nb_nodes.clone();
        *nb_nodes.write().unwrap() += 1;
        let end_time_step_sync = Arc::new(Mutex::new(false));
        let physics_list = self.physics_list.clone();
        running_parameters
            .end_time_step_syncs
            .push(end_time_step_sync.clone());
        running_parameters.running_nodes_names.push(node.name());
        let context = self.context.new_node_context(node.name());
        let handle = thread::spawn(move || -> SimbaResult<Option<Node>> {
            let ret = Self::run_one_node(
                node,
                max_time,
                last_sim_time,
                async_api_server,
                NodeSyncParams {
                    nb_nodes,
                    time_cv: time_cv.clone(),
                    common_time: common_time_clone,
                    barrier: barrier_clone,
                    end_time_step_sync,
                    physics_list,
                },
                context.clone(),
            );
            let _lk = time_cv.waiting.lock().unwrap();
            match &ret {
                Err(_) => *time_cv.force_finish.lock().unwrap() = true,
                Ok(Some(node)) => {
                    // Increase finishing nodes only if the node is still existing
                    // as in case of zombie, the total number of node has been decreased.
                    *finishing_cv_clone.0.lock().unwrap() += 1;
                    internal!(
                        context,
                        crate::logger::InternalLog::NodeSyncDetailed,
                        "Node {} finished: {} nodes finished",
                        node.name(),
                        *finishing_cv_clone.0.lock().unwrap(),
                    );
                    // finishing_cv_clone.1.notify_all();
                }
                _ => {}
            };
            time_cv.condvar.notify_all();

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
            let mut file = match OpenOptions::new().read(true).write(true).open(&filename) {
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
            self.context,
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
    fn process_records(&mut self, time: Option<f32>, context: &Context) -> SimbaResult<()> {
        let context = context.new_callstack_level("process_records");
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

        if time.is_none()
            && let Some(taf) = &mut self.time_analysis_factory
        {
            // Only at the end
            taf.save_results(&context);
        }

        internal!(
            context,
            crate::logger::InternalLog::NodeRunning,
            "Collecting results"
        );
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
                context,
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

    /// Load results from the given result `filename` or from the named specified in the loaded configuration.
    ///
    /// Returns the maximum time found in loaded records.
    pub fn load_results(&mut self, filename: Option<String>) -> SimbaResult<f32> {
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
        let results = Self::deserialize_results_from_file(&filename, &self.context)?;

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

    #[deprecated(note = "Will be removed in future release. Use load_results instead")]
    /// Deprecated alias for [`Simulator::load_results`].
    pub fn load_results_full(&mut self, filename: Option<String>) -> SimbaResult<f32> {
        self.load_results(filename)
    }

    /// Deserialize persisted simulator results from a JSON file.
    ///
    /// ## Arguments
    /// * `filename` - Path to the JSON result file.
    /// * `context` - Shared simulation context used for load/deserialization logging.
    pub fn deserialize_results_from_file(
        filename: &Path,
        context: &Context,
    ) -> SimbaResult<Results> {
        info!(
            context,
            "Loading results from file `{}`",
            filename.to_str().unwrap()
        );
        let mut recording_file = match File::open(filename) {
            Ok(file) => file,
            Err(e) => {
                return Err(SimbaError::new(
                    SimbaErrorTypes::ConfigError,
                    format!("Impossible to open record file: {}", e),
                ));
            }
        };
        let mut content = String::new();
        recording_file
            .read_to_string(&mut content)
            .expect("Impossible to read record file");

        info!(context, "Deserialize results...");
        Ok(serde_json::from_str(&content).expect("Error during json parsing"))
    }

    /// Run the loop for the given `node` until reaching `max_time`.
    ///
    /// ## Arguments
    /// * `node` - Node to be run.
    /// * `max_time` - Time to stop the loop.
    /// * `async_api_server` - If the async API is enabled, the node will send its records to the async API server, which will be able to send them to the GUI in real time.
    /// * `node_sync_params` - Parameters to synchronize the node with the other nodes of the simulation.
    #[cfg(not(feature = "monothreaded"))]
    fn run_one_node(
        mut node: Node,
        max_time: f32,
        last_sim_time: f32,
        async_api_server: Option<SimulatorAsyncApiServer>,
        node_sync_params: NodeSyncParams,
        context: Context,
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
        info!(context, "Start thread of node {}", node.name());
        let mut next_time = last_sim_time;
        node_sync_params.barrier.wait();
        node_sync_params.barrier.wait();
        loop {
            if *node_sync_params.time_cv.force_finish.lock().unwrap() {
                break;
            }
            next_time = node.next_time_step(next_time + TIME_ROUND / 2., &context)?;
            internal!(
                context,
                crate::logger::InternalLog::NodeSyncDetailed,
                "Got next_time: {next_time}"
            );
            internal!(
                context,
                crate::logger::InternalLog::NodeSyncDetailed,
                "Get common time (next_time is {next_time})"
            );
            {
                let mut unlocked_common_time = node_sync_params.common_time.write().unwrap();
                if *unlocked_common_time > next_time {
                    *unlocked_common_time = next_time;
                    internal!(
                        context,
                        crate::logger::InternalLog::NodeSyncDetailed,
                        "Set common time"
                    );
                }
            }
            node_sync_params.barrier.wait();

            next_time = *node_sync_params.common_time.read().unwrap();
            internal!(
                context,
                crate::logger::InternalLog::NodeSyncDetailed,
                "Barrier... final next_time is {next_time}"
            );
            node_sync_params.barrier.wait();
            *node_sync_params.common_time.write().unwrap() = f32::INFINITY;
            node_sync_params.barrier.wait();
            if let Some(async_api_server) = &async_api_server {
                async_api_server.update_time(next_time);
            }
            context.update_time(next_time);
            if next_time > max_time {
                break;
            }

            node.run_next_time_step(next_time, &node_sync_params.time_cv, &context)?;
            internal!(
                context,
                crate::logger::InternalLog::NodeSyncDetailed,
                "End of time step wait"
            );
            if node.send_records()
                && let Some(async_api_server) = &async_api_server
            {
                async_api_server.send_record(&Record {
                    time: next_time,
                    node: node.record(&context),
                });
            }
            internal!(
                context,
                crate::logger::InternalLog::NodeSyncDetailed,
                "End of time step sync"
            );

            node_sync_params
                .end_time_step_sync
                .lock()
                .unwrap()
                .clone_from(&true);
            node.sync_with_others(&node_sync_params.time_cv, next_time, &context);
            // node_sync_params.time_cv.condvar.notify_all();
            // while !*node_sync_params.end_time_step_sync.lock().unwrap() {
            //     lk = node_sync_params.time_cv.condvar.wait(lk).unwrap();
            // }
            // std::mem::drop(lk);
            internal!(
                context,
                crate::logger::InternalLog::NodeSyncDetailed,
                "Wait at final barrier"
            );
            node_sync_params.barrier.wait();
            if node.process_messages() {
                node.handle_messages(next_time, &context);
            }
            if node.state() == NodeState::Zombie {
                info!(context, "Killing node {}", node.name());
                if node.process_messages() {
                    node.handle_messages(next_time, &context);
                }
                *node_sync_params.nb_nodes.write().unwrap() -= 1;
                node_sync_params.time_cv.condvar.notify_all();
                node.kill(next_time);
                node_sync_params
                    .physics_list
                    .write()
                    .unwrap()
                    .remove(&node.name());
                node_sync_params.barrier.remove_one();
                return Ok(None);
            }

            node_sync_params.barrier.wait();
        }

        Ok(Some(node))
    }

    /// Main loop for the simulator main thread. This loop is responsible for synchronizing the nodes at each time step, executing the scenario, and processing the messages between nodes.
    fn simulator_spin(&mut self, running_parameters: &mut RunningParameters) -> SimbaResult<()> {
        let time_cv = self.time_cv.clone();
        let context = self.context.new_callstack_level("spin");
        loop {
            let mut lk = time_cv.waiting.lock().unwrap();
            let mut waiting_nodes = 0;
            while *lk < *running_parameters.nb_nodes.read().unwrap()
                && *running_parameters.finishing_cv.0.lock().unwrap()
                    < *running_parameters.nb_nodes.read().unwrap()
                && !*time_cv.force_finish.lock().unwrap()
            {
                internal!(
                    context,
                    crate::logger::InternalLog::NodeSyncDetailed,
                    "Simulator spin waiting... (waiting_nodes: {}, nb_nodes: {}, finishing_nodes: {}, end_procedure_waiting: {})",
                    *lk,
                    *running_parameters.nb_nodes.read().unwrap(),
                    *running_parameters.finishing_cv.0.lock().unwrap(),
                    waiting_nodes,
                );
                lk = time_cv.condvar.wait(lk).unwrap();
            }
            if *time_cv.force_finish.lock().unwrap() {
                return Ok(());
            }

            internal!(
                context,
                crate::logger::InternalLog::NodeSyncDetailed,
                "Simulator spin continue... (waiting_nodes: {}, nb_nodes: {}, finishing_nodes: {}, end_procedure_waiting: {})",
                *lk,
                *running_parameters.nb_nodes.read().unwrap(),
                *running_parameters.finishing_cv.0.lock().unwrap(),
                waiting_nodes
            );
            let node_states = self.node_states();

            let mut time_end_procedure = false;
            for end_time_step_sync in running_parameters.end_time_step_syncs.iter() {
                let mut lock = end_time_step_sync.lock().unwrap();
                if *lock {
                    waiting_nodes += 1;
                    if waiting_nodes >= *running_parameters.nb_nodes.read().unwrap() {
                        running_parameters.barrier.add_one();
                        time_end_procedure = true;
                        waiting_nodes = 0;
                    }
                    *lock = false;
                };
            }

            if time_end_procedure {
                internal!(
                    context,
                    crate::logger::InternalLog::NodeSyncDetailed,
                    "Time step end procedure... (nb_nodes: {}, finishing_nodes: {})",
                    *running_parameters.nb_nodes.read().unwrap(),
                    *running_parameters.finishing_cv.0.lock().unwrap()
                );
                self.end_of_time_step_procedure(&node_states, running_parameters, &context)?;
                for end_time_step_sync in running_parameters.end_time_step_syncs.iter() {
                    end_time_step_sync.lock().unwrap().clone_from(&false);
                }
                running_parameters.barrier.remove_one();
            } else {
                self.network_manager
                    .process_messages(&node_states, &context)
                    .unwrap();
            }
            if *running_parameters.finishing_cv.0.lock().unwrap()
                >= *running_parameters.nb_nodes.read().unwrap()
            {
                internal!(
                    context,
                    crate::logger::InternalLog::NodeSyncDetailed,
                    "All nodes finishing... (nb_nodes: {}, finishing_nodes: {})",
                    *running_parameters.nb_nodes.read().unwrap(),
                    *running_parameters.finishing_cv.0.lock().unwrap()
                );
                return Ok(());
            }
            internal!(
                context,
                crate::logger::InternalLog::NodeSyncDetailed,
                "End of simulator spin loop... (nb_nodes: {}, finishing_nodes: {})",
                *running_parameters.nb_nodes.read().unwrap(),
                *running_parameters.finishing_cv.0.lock().unwrap()
            );
            // Finishing time step procedure
            *lk = 0;
            let mut waiting_parity = time_cv.intermediate_parity.lock().unwrap();
            *waiting_parity = 1 - *waiting_parity;
            time_cv.condvar.notify_all();
        }
    }

    fn end_of_time_step_procedure(
        &mut self,
        node_states: &HashMap<String, Option<[f32; 2]>>,
        running_parameters: &mut RunningParameters,
        context: &Context,
    ) -> SimbaResult<()> {
        let current_time = context.get_time().expect("Simulator context has no time!!");
        running_parameters.last_sim_time = current_time;
        self.last_sim_time = current_time;
        if let Err(e) = self.process_records(Some(current_time), context) {
            error!(
                context,
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
            .execute_scenario(self, node_states, running_parameters, context)
            .unwrap();
        self.network_manager
            .process_messages(node_states, context)
            .unwrap();
        Ok(())
    }

    fn node_states(&self) -> HashMap<String, Option<[f32; 2]>> {
        HashMap::from_iter(
            self.environment
                .get_meta_data()
                .read()
                .unwrap()
                .iter()
                .filter_map(|(node_name, meta_data)| {
                    let meta_data = meta_data.read().unwrap();
                    if meta_data.state == NodeState::Running {
                        Some((node_name.clone(), meta_data.position))
                    } else {
                        None
                    }
                }),
        )
    }

    /// Compute configured post-processing results from in-memory records.
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

        info!(self.context, "Starting result analyse...");
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
            info!(self.context, "Analyse the results...");
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
                info!(self.context, "Showing figures...");
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

    /// Create and return a new async API client for the simulator.
    pub fn get_async_api(&mut self) -> Arc<SimulatorAsyncApi> {
        if self.async_api_server.is_none() {
            self.async_api_server = Some(SimulatorAsyncApiServer::new(0.));
        }
        Arc::new(self.async_api_server.as_mut().unwrap().new_client())
    }

    /// Get the shared message broker used by the simulator network manager.
    pub fn get_broker(&self) -> SharedRwLock<SimbaBroker> {
        self.network_manager.broker()
    }

    /// Get a context for the simulator
    pub fn get_context(&self) -> Context {
        self.context.clone()
    }

    /// Get the environment of the simulator, containing the map and the meta-data of the nodes.
    pub fn get_environment(&self) -> Arc<Environment> {
        self.environment.clone()
    }
}

impl Default for Simulator {
    fn default() -> Self {
        Self::new()
    }
}
