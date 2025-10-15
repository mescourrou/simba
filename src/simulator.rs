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
    ).unwrap();

    // Show the simulator loaded configuration
    simulator.show();

    // Run the simulator for the time given in the configuration
    // It also save the results to json
    simulator.run().unwrap();

    simulator.compute_results().unwrap();
}

```


*/

// Configuration for Simulator
extern crate confy;
#[cfg(feature = "gui")]
use crate::{
    constants::TIME_ROUND,
    gui::{
        utils::{enum_combobox, json_config, path_finder},
        UIComponent,
    },
    utils::determinist_random_variable::seed_generation_component,
};
use config_checker::macros::Check;
use config_checker::ConfigCheckable;
#[cfg(feature = "gui")]
use egui::CollapsingHeader;
use pyo3::prelude::*;
use serde_derive::{Deserialize, Serialize};
#[cfg(feature = "gui")]
use simba_macros::{EnumToString, ToVec};

use crate::{
    api::internal_api::NodeClient,
    errors::{SimbaError, SimbaErrorTypes, SimbaResult},
    logger::{init_log, is_enabled, LoggerConfig},
    networking::network_manager::NetworkManager,
    node::Node,
    node_factory::{ComputationUnitConfig, NodeFactory, NodeRecord, RobotConfig},
    plugin_api::PluginAPI,
    recordable::Recordable,
    state_estimators::state_estimator::State,
    time_analysis::{self, TimeAnalysisConfig},
    utils::determinist_random_variable::DeterministRandomVariableFactory,
    utils::{format_option_f32, time_ordered_data::TimeOrderedData},
};
use core::f32;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use colored::Colorize;
use serde_json::{self, Value};
use std::default::Default;
use std::fs::{self, File};
use std::io::prelude::*;
use std::sync::{mpsc, Arc, Barrier, Condvar, Mutex, RwLock};
use std::thread::{self, ThreadId};

use log::{debug, info, warn};

use pyo3::prepare_freethreaded_python;

#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct ResultConfig {
    /// Filename to save the results, in JSON format. The directory of this
    /// file is used to save the figures if results are computed.
    pub result_path: String,
    /// Show the matplotlib figures
    pub show_figures: bool,
    /// Path to the python analyse scrit.
    /// This script should have the following entry point:
    /// ```def analyse(result_data: Record, figure_path: str, figure_type: str)```
    /// If the option is none, the script is not run
    pub analyse_script: Option<String>,
    pub figures_path: Option<String>,
    pub python_params: Value,
}

impl Default for ResultConfig {
    /// Default scenario configuration: no nodes.
    fn default() -> Self {
        Self {
            result_path: String::from("../results.json"),
            show_figures: true,
            analyse_script: None,
            figures_path: None,
            python_params: Value::Null,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ResultConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        buffer_stack: &mut BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        let python_param_key = format!("result-config-python-params-{}", unique_id);
        let python_param_error_key = format!("result-config-python-params-error-{}", unique_id);
        CollapsingHeader::new("Results").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Result Path:");
                path_finder(ui, &mut self.result_path, &global_config.base_path);
            });

            ui.horizontal(|ui| {
                ui.label("Show figures:");
                ui.checkbox(&mut self.show_figures, "");
            });

            ui.horizontal(|ui| {
                ui.label("Result Path:");
                if let Some(script) = &mut self.analyse_script {
                    path_finder(ui, script, &global_config.base_path);
                    if ui.button("X").clicked() {
                        self.analyse_script = None;
                    }
                } else if ui.button("+").clicked() {
                    self.analyse_script = Some(String::from(""));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Figure Path:");
                if let Some(fig) = &mut self.figures_path {
                    path_finder(ui, fig, &global_config.base_path);
                    if ui.button("X").clicked() {
                        self.figures_path = None;
                    }
                } else if ui.button("+").clicked() {
                    self.figures_path = Some(String::from(""));
                }
            });
            ui.vertical(|ui| {
                ui.label("Python params (JSON format, null for nothing):");
                json_config(
                    ui,
                    &python_param_key,
                    &python_param_error_key,
                    buffer_stack,
                    &mut self.python_params,
                );
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &String) {
        CollapsingHeader::new("Results").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!("Result Path: {}", self.result_path));
            });

            ui.horizontal(|ui| {
                ui.label("Show figures: ");
                if self.show_figures {
                    ui.label("Yes");
                } else {
                    ui.label("No");
                }
            });

            ui.horizontal(|ui| {
                ui.label("Result Path: ");
                if let Some(script) = &self.analyse_script {
                    ui.label(script);
                } else {
                    ui.label("None");
                }
            });
            ui.horizontal(|ui| {
                ui.label("Figure Path: ");
                if let Some(path) = &self.figures_path {
                    ui.label(path);
                } else {
                    ui.label("None");
                }
            });
            ui.vertical(|ui| {
                ui.label("Python params: ");
                ui.label(self.python_params.to_string());
            });
        });
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "gui", derive(ToVec, EnumToString))]
pub enum TimeMode {
    Centralized,
    Decentralized,
}

/// Scenario configuration for the simulator.
/// The Simulator configuration is the root of the scenario configuration.
///
/// This config contains an item, `robots`, which list the robot nodes [`RobotConfig`]
/// and a list of `computation_units`: [`ComputationUnitConfig`].
///
/// ## Example in yaml:
/// ```ignore
/// robots:
///     - RobotConfig 1
///     - RobotConfig 2
/// computation_units:
///     - ComputationUnitConfig 1
///     - ComputationUnitConfig 2
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

    #[serde(skip_deserializing)]
    pub base_path: Box<Path>,

    pub max_time: f32,
    pub time_mode: TimeMode,

    #[check]
    pub time_analysis: TimeAnalysisConfig,
    #[serde(serialize_with = "format_option_f32")]
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

#[cfg(feature = "gui")]
impl crate::gui::UIComponent for SimulatorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        CollapsingHeader::new("Simulator").show(ui, |ui| {
            ui.horizontal(|ui| {
                self.log.show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );
            });

            ui.horizontal_top(|ui| {
                if let Some(result_cfg) = &mut self.results {
                    result_cfg.show_mut(
                        ui,
                        ctx,
                        buffer_stack,
                        global_config,
                        current_node_name,
                        unique_id,
                    );
                    if ui.button("X").clicked() {
                        self.results = None;
                    }
                } else {
                    ui.label("Results: ");
                    if ui.button("+").clicked() {
                        self.results = Some(ResultConfig::default());
                    }
                }
            });

            ui.horizontal(|ui| {
                ui.label("Random seed: ");
                if let Some(seed) = &mut self.random_seed {
                    seed_generation_component(seed, ui, buffer_stack, unique_id);
                    if ui.button("X").clicked() {
                        self.random_seed = None;
                    }
                } else {
                    if ui.button("+").clicked() {
                        self.random_seed = Some(rand::random());
                    }
                }
            });

            ui.horizontal(|ui| {
                ui.label("Max time: ");
                ui.add(
                    egui::DragValue::new(&mut self.max_time)
                        .max_decimals((1. / TIME_ROUND) as usize),
                );
            });

            ui.horizontal(|ui| {
                ui.label("Time Mode: ");
                enum_combobox(ui, &mut self.time_mode, "time-mode");
            });

            ui.horizontal(|ui| {
                self.time_analysis.show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );
            });

            ui.vertical(|ui| {
                ui.label("Robots:");
                let mut remove = None;
                for (i, r) in self.robots.iter_mut().enumerate() {
                    let robot_unique_id = format!("{}-{}", unique_id, &r.name);
                    ui.horizontal_top(|ui| {
                        r.show_mut(ui, ctx, buffer_stack, global_config, None, &robot_unique_id);
                        if ui.button("X").clicked() {
                            remove = Some(i);
                        }
                    });
                }
                if let Some(i) = remove {
                    self.robots.remove(i);
                }
                if ui.button("Add").clicked() {
                    self.robots.push(RobotConfig::default());
                }
            });

            ui.vertical(|ui| {
                ui.label("Computation Units:");
                let mut remove = None;
                for (i, cu) in self.computation_units.iter_mut().enumerate() {
                    let cu_unique_id = format!("{}-{}", unique_id, &cu.name);
                    ui.horizontal_top(|ui| {
                        cu.show_mut(ui, ctx, buffer_stack, global_config, None, &cu_unique_id);
                        if ui.button("X").clicked() {
                            remove = Some(i);
                        }
                    });
                }
                if let Some(i) = remove {
                    self.computation_units.remove(i);
                }
                if ui.button("Add").clicked() {
                    self.computation_units
                        .push(ComputationUnitConfig::default());
                }
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &String) {
        CollapsingHeader::new("Simulator").show(ui, |ui| {
            ui.horizontal(|ui| {
                self.log.show(ui, ctx, unique_id);
            });

            ui.horizontal_top(|ui| {
                if let Some(result_cfg) = &self.results {
                    result_cfg.show(ui, ctx, unique_id);
                } else {
                    ui.label("Results disabled");
                }
            });

            ui.horizontal(|ui| {
                ui.label("Random seed: ");
                if let Some(seed) = &self.random_seed {
                    ui.label(format!("{}", seed));
                } else {
                    ui.label("Disabled");
                }
            });

            ui.horizontal(|ui| {
                ui.label("Max time: ");
                ui.label(format!("{}", self.max_time));
            });

            ui.horizontal(|ui| {
                ui.label("Time Mode: ");
                ui.label(format!("{}", self.time_mode.to_string()));
            });

            ui.horizontal(|ui| {
                self.time_analysis.show(ui, ctx, unique_id);
            });

            ui.vertical(|ui| {
                ui.label("Robots:");
                for r in &self.robots {
                    let robot_unique_id = format!("{}-{}", unique_id, &r.name);
                    r.show(ui, ctx, &robot_unique_id);
                }
            });

            ui.vertical(|ui| {
                ui.label("Computation Units:");
                for cu in &self.computation_units {
                    let cu_unique_id = format!("{}-{}", unique_id, &cu.name);
                    cu.show(ui, ctx, &cu_unique_id);
                }
            });
        });
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
static TIME: RwLock<f32> = RwLock::new(0.);
static EXCLUDE_NODES: RwLock<Vec<String>> = RwLock::new(Vec::new());
static INCLUDE_NODES: RwLock<Vec<String>> = RwLock::new(Vec::new());

pub struct SimulatorAsyncApi {
    pub current_time: Arc<Mutex<f32>>,
    pub records: Arc<Mutex<mpsc::Receiver<Record>>>,
}

#[derive(Clone)]
struct SimulatorAsyncApiServer {
    pub current_time: Arc<Mutex<f32>>,
    pub records: mpsc::Sender<Record>,
}

#[derive(Debug)]
pub struct TimeCv {
    pub finished_nodes: Mutex<usize>,
    pub intermediate_waiting: Mutex<usize>,
    pub intermediate_parity: Mutex<u8>,
    pub circulating_messages: Mutex<usize>,
    pub force_finish: Mutex<bool>,
    pub condvar: Condvar,
}

impl TimeCv {
    pub fn new() -> Self {
        Self {
            finished_nodes: Mutex::new(0),
            intermediate_waiting: Mutex::new(0),
            intermediate_parity: Mutex::new(0),
            circulating_messages: Mutex::new(0),
            force_finish: Mutex::new(false),
            condvar: Condvar::new(),
        }
    }
}

/// This is the central structure which manages the run of the scenario.
///
/// To run the scenario, there are two mandatory steps:
/// * Load the config using [`Simulator::from_config_path`] (from a file), or using
/// [`Simulator::from_config`] with the [`SimulatorConfig`] structs directly.
/// * Run the scenario, once the config is loaded, the scenario can be run using
/// [`Simulator::run`].
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
/// fn main() {
///
///     // Initialize the environment, essentially the logging part
///     Simulator::init_environment();
///     info!("Load configuration...");
///     let mut simulator = Simulator::from_config_path(
///         Path::new("config_example/config.yaml"), //<- configuration path
///         &None,                                      //<- plugin API, to load external modules
///     ).unwrap();
///
///     // Show the simulator loaded configuration
///     simulator.show();
///
///     // It also save the results to "result.json",
///     simulator.run().unwrap();
///
///     // compute the results and show the figures.
///     simulator.compute_results().unwrap();
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
    common_time: Arc<Mutex<f32>>,

    async_api: Option<Arc<SimulatorAsyncApi>>,
    async_api_server: Option<SimulatorAsyncApiServer>,

    node_apis: BTreeMap<String, NodeClient>,
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
            common_time: Arc::new(Mutex::new(f32::INFINITY)),
            node_apis: BTreeMap::new(),
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
        plugin_api: &Option<Box<&dyn PluginAPI>>,
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
        plugin_api: &Option<Box<&dyn PluginAPI>>,
    ) -> SimbaResult<Simulator> {
        let mut simulator = Simulator::new();
        simulator.load_config(config, plugin_api)?;
        Ok(simulator)
    }

    pub fn reset(&mut self, plugin_api: &Option<Box<&dyn PluginAPI>>) -> SimbaResult<()> {
        self.nodes = Vec::new();
        self.time_cv = Arc::new(TimeCv::new());
        self.network_manager = NetworkManager::new(self.time_cv.clone());
        let config = self.config.clone();
        self.common_time = Arc::new(Mutex::new(f32::INFINITY));
        let mut service_managers = BTreeMap::new();
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
            info!("Finishing initialization of {}", node.name());
            self.node_apis
                .insert(node.name(), node.post_creation_init(&service_managers));
        }
        Ok(())
    }

    pub fn load_config_path(
        &mut self,
        config_path: &Path,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
    ) -> SimbaResult<()> {
        println!("Load configuration from {:?}", config_path);
        let mut config: SimulatorConfig = match confy::load_path(config_path) {
            Ok(config) => config,
            Err(error) => {
                let what = format!("Error from Confy while loading the config file : {}", error);
                println!("ERROR: {what}");
                return Err(SimbaError::new(SimbaErrorTypes::ConfigError, what));
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
        self.load_config(&config, plugin_api)
    }

    pub fn load_config(
        &mut self,
        config: &SimulatorConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
    ) -> SimbaResult<()> {
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
            return Err(SimbaError::new(
                SimbaErrorTypes::ConfigError,
                "Error in config".to_string(),
            ));
        }
        Self::init_log(&config.log)?;
        self.config = config.clone();
        if let Some(seed) = config.random_seed {
            self.determinist_va_factory.global_seed = seed;
        } else {
            self.config.random_seed = Some(self.determinist_va_factory.global_seed);
        }

        time_analysis::init_from_config(&self.config.time_analysis)?;

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
        time_analysis::set_node_name("simulator".to_string());

        prepare_freethreaded_python();
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
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
    ) {
        let mut new_node = NodeFactory::make_robot(
            robot_config,
            plugin_api,
            &global_config,
            &self.determinist_va_factory,
            self.time_cv.clone(),
        );
        self.network_manager.register_node_network(&mut new_node);
        self.nodes.push(new_node);
    }

    fn add_computation_unit(
        &mut self,
        computation_unit_config: &ComputationUnitConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
    ) {
        let mut new_node = NodeFactory::make_computation_unit(
            computation_unit_config,
            plugin_api,
            &global_config,
            &self.determinist_va_factory,
            self.time_cv.clone(),
        );
        self.network_manager.register_node_network(&mut new_node);
        self.nodes.push(new_node);
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
    /// the configuration give ([`SimulatorConfig`]).
    pub fn run(&mut self) -> SimbaResult<()> {
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
            let common_time_clone = self.common_time.clone();
            let finishing_cv_clone = finishing_cv.clone();
            let barrier_clone = barrier.clone();
            let handle = thread::spawn(move || -> SimbaResult<Node> {
                let ret = Self::run_one_node(
                    node,
                    new_max_time,
                    nb_nodes,
                    i,
                    time_cv.clone(),
                    async_api_server,
                    common_time_clone,
                    barrier_clone,
                );
                if ret.is_err() {
                    *time_cv.force_finish.lock().unwrap() = true;
                }
                *finishing_cv_clone.0.lock().unwrap() += 1;
                finishing_cv_clone.1.notify_all();
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
                Ok(n) => self.nodes.push(n),
            };
        }

        if let Some(e) = error {
            return Err(e);
        }

        self.save_results()
    }

    /// Returns the list of all [`Record`]s produced by [`Simulator::run`].
    pub fn get_records(&self) -> Vec<Record> {
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
    fn save_results(&mut self) -> SimbaResult<()> {
        if self.config.results.is_none() {
            return Ok(());
        }
        let result_config = self.config.results.clone().unwrap();
        let filename = result_config.result_path;
        let filename = self.config.base_path.as_ref().join(filename);

        time_analysis::save_results();
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

        recording_file.write(b"{\"config\": ").unwrap();
        if let Err(e) = serde_json::to_writer(&recording_file, &self.config) {
            return Err(SimbaError::new(
                SimbaErrorTypes::ImplementationError,
                format!("Error during json serialization of config: {e}"),
            ));
        }
        recording_file.write(b",\n\"records\": [\n").unwrap();

        let results = self.get_records();

        let mut first_row = true;
        for row in &results {
            if first_row {
                first_row = false;
            } else {
                recording_file.write(b",\n").unwrap();
            }
            let record = Record {
                time: row.time.clone(),
                node: row.node.clone(),
            };
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
        recording_file.write(b"\n]}").unwrap();
        Ok(())
    }

    pub fn load_results_and_analyse(&mut self) -> SimbaResult<()> {
        if self.config.results.is_none() {
            return Ok(());
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

        self._compute_results(results.records, &results.config)
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
        _node_idx: usize,
        time_cv: Arc<TimeCv>,
        async_api_server: Option<SimulatorAsyncApiServer>,
        common_time: Arc<Mutex<f32>>,
        barrier: Arc<Barrier>,
    ) -> SimbaResult<Node> {
        info!("Start thread of node {}", node.name());
        let mut thread_ids = THREAD_IDS.write().unwrap();
        thread_ids.push(thread::current().id());
        THREAD_NAMES.write().unwrap().push(node.name());
        drop(thread_ids);
        time_analysis::set_node_name(node.name());

        let mut previous_time = 0.;
        loop {
            if *time_cv.force_finish.lock().unwrap() {
                break;
            }
            let mut next_time = node.next_time_step()?;
            if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                debug!("Got next_time: {next_time}");
            }

            {
                if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                    debug!("Get common time (next_time is {next_time})");
                }
                let mut unlocked_common_time = common_time.lock().unwrap();
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
                    next_time = node.next_time_step()?;
                    {
                        let mut unlocked_common_time = common_time.lock().unwrap();
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

            next_time = *common_time.lock().unwrap();
            if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                debug!("Barrier... final next_time is {next_time}");
            }
            barrier.wait();
            *common_time.lock().unwrap() = f32::INFINITY;
            *time_cv.finished_nodes.lock().unwrap() = 0;
            barrier.wait();
            if let Some(api) = &async_api_server {
                *api.current_time.lock().unwrap() = next_time;
            }
            *TIME.write().unwrap() = next_time;
            if next_time > max_time {
                break;
            }

            node.run_next_time_step(next_time, &time_cv, nb_nodes)?;
            if let Some(api) = &async_api_server {
                let record = Record {
                    time: next_time,
                    node: node.record(),
                };
                api.records.send(record).unwrap();
            }
            previous_time = next_time;
        }
        Ok(node)
    }

    fn simulator_spin(
        &mut self,
        finishing_cv: Arc<(Mutex<usize>, Condvar)>,
        nb_nodes: usize,
    ) -> SimbaResult<()> {
        // self.nodes is empty
        let mut node_states: BTreeMap<String, TimeOrderedData<State>> = BTreeMap::new();
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
            self.network_manager.process_messages(&node_states)?;
            if *finishing_cv.0.lock().unwrap() == nb_nodes {
                return Ok(());
            }
        }
    }

    pub fn compute_results(&self) -> SimbaResult<()> {
        let results = self.get_records();
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

        // prepare_freethreaded_python();

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
            Ok(s) => s,
        };
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
            let time = Arc::new(Mutex::new(0.));
            let (records_tx, records_rx) = mpsc::channel();
            self.async_api_server = Some(SimulatorAsyncApiServer {
                current_time: Arc::clone(&time),
                records: records_tx,
            });
            self.async_api = Some(Arc::new(SimulatorAsyncApi {
                current_time: Arc::clone(&time),
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

        for i in 0..nb_replications {
            print!("Run {}/{nb_replications} ... ", i + 1);
            let mut simulator =
                Simulator::from_config_path(Path::new("test_config/config.yaml"), &None).unwrap();

            simulator.run().unwrap();

            results.push(simulator.get_records());
            println!("OK");
        }

        let reference_result = &results[0];
        for i in 1..nb_replications {
            assert_eq!(results[i].len(), reference_result.len());
            for j in 0..reference_result.len() {
                let result_as_str = format!("{:?}", results[i][j]);
                let reference_result_as_str = format!("{:?}", reference_result[j]);
                assert_eq!(
                    result_as_str, reference_result_as_str,
                    "{result_as_str} != {reference_result_as_str}"
                );
            }
        }
    }
}
