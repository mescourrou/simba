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

// Configuration for Simulator
extern crate confy;
#[cfg(feature = "gui")]
use crate::{
    constants::TIME_ROUND_DECIMALS,
    gui::{
        utils::{json_config, path_finder},
        UIComponent,
    },
    utils::determinist_random_variable::seed_generation_component,
};
use config_checker::macros::Check;
use config_checker::ConfigCheckable;
#[cfg(feature = "gui")]
use egui::CollapsingHeader;
use pyo3::{ffi::c_str, prelude::*};
use serde_derive::{Deserialize, Serialize};
#[cfg(feature = "gui")]
use simba_macros::EnumToString;

use crate::{
    api::{
        async_api::{
            AsyncApi, AsyncApiLoadConfigRequest, AsyncApiRunRequest, AsyncApiRunner, PluginAsyncAPI,
        },
        internal_api::NodeClient,
    },
    constants::TIME_ROUND,
    errors::{SimbaError, SimbaErrorTypes, SimbaResult},
    logger::{init_log, is_enabled, LoggerConfig},
    networking::network_manager::NetworkManager,
    node::Node,
    node_factory::{ComputationUnitConfig, NodeFactory, NodeRecord, RobotConfig},
    plugin_api::PluginAPI,
    recordable::Recordable,
    state_estimators::state_estimator::State,
    time_analysis::{TimeAnalysisConfig, TimeAnalysisFactory},
    utils::{
        self, barrier::Barrier, determinist_random_variable::DeterministRandomVariableFactory,
        enum_tools::ToVec, format_option_f32, maths::round_precision,
        time_ordered_data::TimeOrderedData,
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
use serde_json::{self, Value};
use std::default::Default;
use std::fs::{self, File};
use std::io::prelude::*;
use std::sync::{mpsc, Arc, Condvar, Mutex, RwLock};
use std::thread::{self, ThreadId};

use log::{debug, info, warn};

#[derive(Serialize, Deserialize, Debug, Clone, Default)]
#[cfg_attr(feature = "gui", derive(EnumToString))]
#[serde(deny_unknown_fields)]
pub enum ResultSaveMode {
    #[default]
    AtTheEnd,
    Continuous,
    Batch(usize),
    Periodic(f32),
}

impl ToVec<&'static str> for ResultSaveMode {
    fn to_vec() -> Vec<&'static str> {
        vec!["AtTheEnd", "Continuous", "Batch", "Periodic"]
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct ResultConfig {
    /// Filename to save the results, in JSON format. The directory of this
    /// file is used to save the figures if results are computed.
    pub result_path: Option<String>,
    /// Show the matplotlib figures
    pub show_figures: bool,
    /// Path to the python analyse scrit.
    /// This script should have the following entry point:
    /// ```def analyse(result_data: Record, figure_path: str, figure_type: str)```
    /// If the option is none, the script is not run
    pub analyse_script: Option<String>,
    pub figures_path: Option<String>,
    pub python_params: Value,
    pub save_mode: ResultSaveMode,
}

impl Default for ResultConfig {
    /// Default scenario configuration: no nodes.
    fn default() -> Self {
        Self {
            result_path: None,
            show_figures: false,
            analyse_script: None,
            figures_path: None,
            python_params: Value::Null,
            save_mode: ResultSaveMode::default(),
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
        unique_id: &str,
    ) {
        let python_param_key = format!("result-config-python-params-{}", unique_id);
        let python_param_error_key = format!("result-config-python-params-error-{}", unique_id);
        CollapsingHeader::new("Results").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Result Path:");
                if let Some(path) = &mut self.result_path {
                    path_finder(ui, path, &global_config.base_path);
                    if ui.button("X").clicked() {
                        self.result_path = None;
                    }
                } else if ui.button("+").clicked() {
                    self.result_path = Some(String::from(""));
                }
            });

            let mut current_str = self.save_mode.to_string();
            ui.horizontal(|ui| {
                use crate::gui::utils::string_combobox;

                ui.label("Save mode:");
                string_combobox(
                    ui,
                    &ResultSaveMode::to_vec()
                        .iter()
                        .map(|x| String::from(*x))
                        .collect(),
                    &mut current_str,
                    format!("result-save-mode-choice-{}", unique_id),
                );
                if let ResultSaveMode::Batch(size) = &mut self.save_mode {
                    use egui::DragValue;

                    ui.add(DragValue::new(size).max_decimals(0));
                    if *size < 1 {
                        *size = 1;
                    }
                } else if let ResultSaveMode::Periodic(period) = &mut self.save_mode {
                    use egui::DragValue;

                    ui.add(DragValue::new(period));
                    if *period <= TIME_ROUND {
                        *period = TIME_ROUND;
                    }
                }
            });
            if current_str != self.save_mode.to_string() {
                match current_str.as_str() {
                    "AtTheEnd" => self.save_mode = ResultSaveMode::AtTheEnd,
                    "Continuous" => self.save_mode = ResultSaveMode::Continuous,
                    "Batch" => self.save_mode = ResultSaveMode::Batch(10),
                    "Periodic" => self.save_mode = ResultSaveMode::Periodic(5.),
                    _ => panic!("Where did you find this value?"),
                };
            }

            ui.horizontal(|ui| {
                ui.label("Show figures:");
                ui.checkbox(&mut self.show_figures, "");
            });

            ui.horizontal(|ui| {
                ui.label("Analyse script path:");
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

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        CollapsingHeader::new("Results").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Result Path: ");
                if let Some(path) = &self.figures_path {
                    ui.label(path);
                } else {
                    ui.label("None");
                }
            });

            ui.horizontal(|ui| {
                let mut as_str = self.save_mode.to_string();
                if let ResultSaveMode::Batch(s) = &self.save_mode {
                    as_str = format!("{} ({})", as_str, s);
                } else if let ResultSaveMode::Periodic(p) = &self.save_mode {
                    as_str = format!("{} ({} s)", as_str, p);
                }
                ui.label(format!("Save mode: {}", as_str));
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
    pub version: String,
    #[check]
    pub log: LoggerConfig,
    #[check]
    pub results: Option<ResultConfig>,

    pub base_path: Box<Path>,

    pub max_time: f32,

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
            version: VERSION.to_string(),
            log: LoggerConfig::default(),
            base_path: Box::from(Path::new(".")),
            results: None,
            time_analysis: TimeAnalysisConfig::default(),
            random_seed: None,
            robots: Vec::new(),
            computation_units: Vec::new(),
            max_time: 60.,
        }
    }
}

impl SimulatorConfig {
    pub fn load_from_path(path: &Path) -> SimbaResult<Self> {
        let mut config: serde_yaml::Value = match confy::load_path(path) {
            Ok(config) => config,
            Err(error) => {
                let what = format!(
                    "Error from Confy while loading the config file : {}",
                    utils::confy::detailed_error(&error)
                );
                println!("ERROR: {what}");
                return Err(SimbaError::new(SimbaErrorTypes::ConfigError, what));
            }
        };
        config.apply_merge().map_err(|e| {
            let what = format!("Error from SerdeYAML while merging YAML tags: {}", e);
            println!("ERROR: {what}");
            SimbaError::new(SimbaErrorTypes::ConfigError, what)
        })?;
        let mut config: SimulatorConfig = match serde_yaml::from_value(config) {
            Ok(c) => c,
            Err(e) => {
                let what = format!("Error from SerdeYAML while loading SimulatorConfig : {}", e);
                println!("ERROR: {what}");
                return Err(SimbaError::new(SimbaErrorTypes::ConfigError, what));
            }
        };

        config.base_path = Box::from(path.parent().unwrap());
        config.time_analysis.output_path = config
            .base_path
            .as_ref()
            .join(&config.time_analysis.output_path)
            .to_str()
            .unwrap()
            .to_string();

        Ok(config)
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
        unique_id: &str,
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
                } else if ui.button("+").clicked() {
                    self.random_seed = Some(rand::random());
                }
            });

            ui.horizontal(|ui| {
                ui.label("Max time: ");
                ui.add(egui::DragValue::new(&mut self.max_time).max_decimals(TIME_ROUND_DECIMALS));
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

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
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
    pub current_time: Arc<RwLock<f32>>,
    pub records: Arc<Mutex<mpsc::Receiver<Record>>>,
}

#[derive(Clone)]
struct SimulatorAsyncApiServer {
    current_time: Arc<RwLock<f32>>,
    records: Vec<mpsc::Sender<Record>>,
}

impl SimulatorAsyncApiServer {
    pub fn new(time: f32) -> Self {
        Self {
            current_time: Arc::new(RwLock::new(time)),
            records: Vec::new(),
        }
    }

    pub fn new_client(&mut self) -> SimulatorAsyncApi {
        let (tx, rx) = mpsc::channel();
        self.records.push(tx);
        SimulatorAsyncApi {
            current_time: self.current_time.clone(),
            records: Arc::new(Mutex::new(rx)),
        }
    }

    pub fn update_time(&self, new_time: f32) {
        *self.current_time.write().unwrap() = new_time;
    }

    pub fn send_record(&self, record: &Record) {
        for tx in &self.records {
            tx.send(record.clone()).unwrap();
        }
    }
}

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

#[derive(Clone)]
struct ResultSavingData {
    save_mode: ResultSaveMode,
    first_row: bool,
}

impl Default for ResultSavingData {
    fn default() -> Self {
        Self {
            save_mode: ResultSaveMode::default(),
            first_row: true,
        }
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
            if node.send_records {
                if let Some(async_api_server) = &async_api_server {
                    async_api_server.send_record(&Record {
                        time: next_time,
                        node: node.record(),
                    });
                }
            }
            if node.zombie {
                // Only place where nb_node should change.
                if is_enabled(crate::logger::InternalLog::NodeRunning) {
                    debug!("Killing node {}", node.name);
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

        let convert_to_dict = cr#"
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
            let script = PyModule::from_code(py, convert_to_dict, c_str!(""), c_str!(""))?;
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

pub struct AsyncSimulator {
    server: Arc<Mutex<AsyncApiRunner>>,
    api: AsyncApi,
    async_plugin_api: Option<Arc<PluginAsyncAPI>>,
    // python_api: Option<PythonAPI>,
}

impl AsyncSimulator {
    pub fn from_config(
        config_path: String,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
    ) -> SimbaResult<AsyncSimulator> {
        Simulator::init_environment();

        let server = Arc::new(Mutex::new(AsyncApiRunner::new()));
        let api = server.lock().unwrap().get_api();
        let sim = Self {
            server,
            api,
            async_plugin_api: plugin_api.as_ref().map(|_| Arc::new(PluginAsyncAPI::new())),
        };

        sim.server.lock().unwrap().run(
            sim.async_plugin_api
                .clone()
                .map(|api| api as Arc<dyn PluginAPI>),
        );
        sim.api.load_config.async_call(AsyncApiLoadConfigRequest {
            config_path,
            force_send_results: false,
        });

        if let Some(unwrapped_async_api) = &sim.async_plugin_api {
            let api_client = &unwrapped_async_api.get_client();
            let plugin_api_unwrapped = plugin_api.as_ref().unwrap();
            let mut res = sim.api.load_config.try_get_result();
            while res.is_none() {
                api_client.get_state_estimator.try_recv_closure(|request| {
                    plugin_api_unwrapped.get_state_estimator(
                        &request.config,
                        &request.global_config,
                        &request.va_factory,
                    )
                });
                api_client.get_controller.try_recv_closure(|request| {
                    plugin_api_unwrapped.get_controller(
                        &request.config,
                        &request.global_config,
                        &request.va_factory,
                    )
                });
                api_client.get_navigator.try_recv_closure(|request| {
                    plugin_api_unwrapped.get_navigator(
                        &request.config,
                        &request.global_config,
                        &request.va_factory,
                    )
                });
                api_client.get_physics.try_recv_closure(|request| {
                    plugin_api_unwrapped.get_physics(
                        &request.config,
                        &request.global_config,
                        &request.va_factory,
                    )
                });
                plugin_api_unwrapped.check_requests();
                res = sim.api.load_config.try_get_result();
            }
            res.unwrap()?;
        } else {
            sim.api.load_config.wait_result().unwrap()?;
        }

        Ok(sim)
    }

    pub fn run(
        &mut self,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        max_time: Option<f32>,
        reset: bool,
    ) {
        self.api
            .run
            .async_call(AsyncApiRunRequest { max_time, reset });
        if let Some(plugin_api) = plugin_api {
            while self.api.run.try_get_result().is_none() {
                plugin_api.check_requests();
                if Python::attach(|py| py.check_signals()).is_err() {
                    break;
                }
            }
        } else {
            while self.api.run.try_get_result().is_none() {
                if Python::attach(|py| py.check_signals()).is_err() {
                    break;
                }
            }
        }
    }

    pub fn get_records(&self, sorted: bool) -> SimbaResult<Vec<Record>> {
        self.api.get_records.call(sorted).unwrap()
    }

    pub fn compute_results(&self) {
        // Calling directly the simulator to keep python in one thread
        self.server
            .lock()
            .unwrap()
            .get_simulator()
            .lock()
            .unwrap()
            .compute_results()
            .unwrap();
    }

    pub fn stop(&self) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Stop server");
        }
        // Stop server thread
        self.server.lock().unwrap().stop();
    }
}

#[cfg(test)]
mod tests {
    use core::f32;
    use std::{path::Path, sync::mpsc::Sender};

    use crate::{
        constants::TIME_ROUND,
        logger::LogLevel,
        networking::{
            message_handler::MessageHandler,
            network::{Envelope, MessageFlag},
        },
        sensors::sensor::Observation,
        state_estimators::{
            external_estimator::{ExternalEstimatorConfig, ExternalEstimatorRecord},
            perfect_estimator::PerfectEstimatorConfig,
            state_estimator::{
                BenchStateEstimatorConfig, StateEstimator, StateEstimatorConfig,
                StateEstimatorRecord, WorldState,
            },
        },
    };

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

            results.push(simulator.get_records(true));
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
                node.network
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
                record: Value::Null,
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
