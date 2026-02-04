#[cfg(feature = "gui")]
use egui::CollapsingHeader;
use simba_macros::config_derives;
#[cfg(feature = "gui")]
use std::collections::BTreeMap;
use std::path::Path;

use crate::{
    VERSION,
    errors::{SimbaError, SimbaErrorTypes, SimbaResult},
    logger::LoggerConfig,
    node::node_factory::{ComputationUnitConfig, RobotConfig},
    scenario::config::ScenarioConfig,
    simulator::ResultConfig,
    time_analysis::TimeAnalysisConfig,
    utils::{self, format_option_f32},
};

#[cfg(feature = "gui")]
use crate::{
    constants::TIME_ROUND_DECIMALS, utils::determinist_random_variable::seed_generation_component,
};

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
#[config_derives]
pub struct SimulatorConfig {
    pub version: String,
    #[check]
    pub log: LoggerConfig,
    #[check]
    pub results: Option<ResultConfig>,

    pub base_path: Box<Path>,

    pub max_time: f32,

    #[check]
    pub time_analysis: Option<TimeAnalysisConfig>,
    #[serde(serialize_with = "format_option_f32")]
    pub random_seed: Option<f32>,
    /// List of the robots to run, with their specific configuration.
    #[check]
    pub robots: Vec<RobotConfig>,
    #[check]
    pub computation_units: Vec<ComputationUnitConfig>,
    #[check]
    pub scenario: ScenarioConfig,
}

impl Default for SimulatorConfig {
    /// Default scenario configuration: no nodes.
    fn default() -> Self {
        Self {
            version: VERSION.to_string(),
            log: LoggerConfig::default(),
            base_path: Box::from(Path::new(".")),
            results: None,
            time_analysis: Some(TimeAnalysisConfig::default()),
            random_seed: None,
            robots: Vec::new(),
            computation_units: Vec::new(),
            max_time: 60.,
            scenario: ScenarioConfig::default(),
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
        if let Some(time_analysis) = &mut config.time_analysis {
            time_analysis.output_path = config
                .base_path
                .as_ref()
                .join(&time_analysis.output_path)
                .to_str()
                .unwrap()
                .to_string();
        }

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

            ui.horizontal_top(|ui| {
                if let Some(time_analysis) = &mut self.time_analysis {
                    time_analysis.show_mut(
                        ui,
                        ctx,
                        buffer_stack,
                        global_config,
                        current_node_name,
                        unique_id,
                    );
                    if ui.button("X").clicked() {
                        self.time_analysis = None;
                    }
                } else {
                    ui.label("Time Analysis: ");
                    if ui.button("+").clicked() {
                        self.time_analysis = Some(TimeAnalysisConfig::default());
                    }
                }
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
                if let Some(time_analysis) = &self.time_analysis {
                    time_analysis.show(ui, ctx, unique_id);
                } else {
                    ui.label("Time Analysis disabled");
                }
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
