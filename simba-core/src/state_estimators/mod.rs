/*!
Module providing different strategies for the state estimation.

To use an external state estimator (in Rust), use [`external_estimator`]
and implement a specification for [`PluginAPI`](crate::plugin_api::PluginAPI).

## How to create a new (internal) state estimation strategy
To create a new state estimation strategy, here are the required steps.

1) **Make a configuration**
   Your new strategy should define a Config struct, such as
   [`PerfectEstimator`](crate::state_estimators::perfect_estimator::PerfectEstimator),
   with Serialize, Deserialize, Debug, Clone and Default traits.  Add this new Config to
   [`StateEstimatorConfig`](state_estimator::StateEstimatorConfig) enumeration, and to the match
   patterns in [`make_state_estimator_from_config`](state_estimator::make_state_estimator_from_config)
   so that the right strategy is created.

2) **Make a new Record**
   Your new strategy should define a Record struct, such as
   [`PerfectEstimator`](crate::state_estimators::perfect_estimator::PerfectEstimator),
   with Serialize, Deserialize, Debug and Clone traits.

3) **Implement the Trait**
   Implement the trait methods.

4) **Implement the Stateful trait**
   Don't forget to implement the [`Stateful`](crate::stateful::Stateful) trait, with the newly created
   Record struct as generic type.
*/

pub mod external_estimator;
pub mod perfect_estimator;
pub mod pybinds;
pub mod python_estimator;

extern crate nalgebra as na;
use na::SVector;

extern crate confy;
use serde_derive::{Deserialize, Serialize};
use simba_macros::config_derives;

/// Configuration for [`State`] in order to load a state from the configuration.
///
/// The pose should contain 3 elements.
/// TODO: Make a config validation scheme.
#[config_derives]
pub struct StateConfig {
    /// Position and orientation of the robot
    #[check(le(self.velocity.len(), 3))]
    pub pose: Vec<f32>,
    /// Linear velocity
    #[check(le(self.velocity.len(), 2))]
    pub velocity: Vec<f32>,
    pub random: Vec<RandomVariableTypeConfig>,
    pub variable_order: Vec<String>,
}

impl Default for StateConfig {
    fn default() -> Self {
        Self {
            pose: vec![0., 0., 0.],
            velocity: vec![0., 0.],
            random: Vec::new(),
            variable_order: Vec::new(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for StateConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        ui.horizontal(|ui| {
            ui.label("x: ");
            ui.add(egui::DragValue::new(self.pose.get_mut(0).unwrap()).max_decimals(10));
        });
        ui.horizontal(|ui| {
            ui.label("y: ");
            ui.add(egui::DragValue::new(self.pose.get_mut(1).unwrap()).max_decimals(10));
        });
        ui.horizontal(|ui| {
            ui.label("θ: ");
            ui.add(egui::DragValue::new(self.pose.get_mut(2).unwrap()).max_decimals(10));
        });
        ui.horizontal(|ui| {
            ui.label("vx: ");
            ui.add(egui::DragValue::new(self.velocity.get_mut(0).unwrap()).max_decimals(10));
        });
        ui.horizontal(|ui| {
            ui.label("vy: ");
            ui.add(egui::DragValue::new(self.velocity.get_mut(1).unwrap()).max_decimals(10));
        });
        egui::CollapsingHeader::new("Random State")
            .id_salt("state-config-random-variable")
            .show(ui, |ui| {
                RandomVariableTypeConfig::show_vector_mut(
                    &mut self.random,
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );
                let possible_variables = [
                    "x",
                    "y",
                    "orientation",
                    "velocity",
                    "velocity_x",
                    "velocity_y",
                    "vx",
                    "vy",
                    "r",
                    "theta",
                ]
                .iter()
                .map(|x| String::from(*x))
                .collect();
                ui.horizontal(|ui| {
                    ui.label("Variable order:");
                    for (i, var) in self.variable_order.iter_mut().enumerate() {
                        let unique_var_id = format!("variable-{i}-{unique_id}");
                        string_combobox(ui, &possible_variables, var, unique_var_id);
                    }
                    if !self.variable_order.is_empty() && ui.button("-").clicked() {
                        self.variable_order.pop();
                    }
                    if ui.button("+").clicked() {
                        self.variable_order.push(
                            possible_variables
                                .get(self.variable_order.len().min(possible_variables.len()))
                                .unwrap()
                                .clone(),
                        );
                    }
                });
            });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.horizontal(|ui| {
            ui.label(format!("x: {}", self.pose.first().unwrap()));
        });
        ui.horizontal(|ui| {
            ui.label(format!("y: {}", self.pose.get(1).unwrap()));
        });
        ui.horizontal(|ui| {
            ui.label(format!("θ: {}", self.pose.get(2).unwrap()));
        });
        ui.horizontal(|ui| {
            ui.label(format!("vx: {}", self.velocity.first().unwrap()));
        });
        ui.horizontal(|ui| {
            ui.label(format!("vy: {}", self.velocity.get(1).unwrap()));
        });
        RandomVariableTypeConfig::show_vector(&self.random, ui, ctx, unique_id);
        ui.horizontal(|ui| {
            ui.label("Variable order:");
            for var in self.variable_order.iter() {
                ui.label(format!("{}, ", var));
            }
        });
    }
}

/// Record for [`State`] in order to record a state.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct StateRecord {
    /// Position and orientation of the robot
    pub pose: [f32; 3],
    /// Linear velocity.
    pub velocity: [f32; 2],
}

impl Default for StateRecord {
    fn default() -> Self {
        Self {
            pose: [0., 0., 0.],
            velocity: [0., 0.],
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for StateRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!(
                "pose: ({}, {}, {})",
                self.pose[0], self.pose[1], self.pose[2]
            ));
            ui.label(format!(
                "velocity: ({}, {})",
                self.velocity[0], self.velocity[1]
            ));
        });
    }
}

/// State to be estimated.
#[derive(Debug, Clone)]
pub struct State {
    /// Pose of the robot [x, y, orientation]
    pub pose: SVector<f32, 3>,
    /// Linear velocity of the robot (in the longitudinal direction).
    pub velocity: SVector<f32, 2>,
}

impl State {
    /// Creates a new [`State`], with all values to 0.
    pub fn new() -> Self {
        Self {
            pose: SVector::<f32, 3>::new(0., 0., 0.),
            velocity: SVector::<f32, 2>::new(0., 0.),
        }
    }

    pub fn from_vector(vec: Vec<f32>) -> Self {
        let mut state = State::new();
        if !vec.is_empty() {
            state.pose.x = vec[0];
        }
        if vec.len() >= 2 {
            state.pose.y = vec[1];
        }
        if vec.len() >= 3 {
            state.pose.z = vec[2];
        }
        if vec.len() >= 4 {
            state.velocity.x = vec[3];
        }
        if vec.len() >= 5 {
            state.velocity.y = vec[4];
        }
        state
    }

    /// Load a [`State`] from the `config` ([`StateConfig`]).
    pub fn from_config(
        config: &StateConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Self {
        let mut state = Self::new();
        for (i, coord) in config.pose.iter().enumerate() {
            if i >= 3 {
                break;
            }
            state.pose[i] = *coord;
        }
        for (i, vel) in config.velocity.iter().enumerate() {
            if i >= 2 {
                break;
            }
            state.velocity[i] = *vel;
        }
        let mut i = 0;
        let mut add_r = 0.;
        let mut add_theta = 0.;
        for rv_config in config.random.iter() {
            let rv = va_factory.make_variable(rv_config.clone());
            let sampled_value = rv.generate(0.);
            for sample in sampled_value {
                assert!(
                    i < config.variable_order.len(),
                    "The variable_order length ({}) is smaller than the number of random variables sampled ({}).",
                    config.variable_order.len(),
                    i + 1
                );
                match config.variable_order[i].as_str() {
                    "x" => state.pose[0] += sample,
                    "y" => state.pose[1] += sample,
                    "orientation" => state.pose[2] += sample,
                    "velocity" | "velocity_x" | "vx" => state.velocity[0] += sample,
                    "velocity_y" | "vy" => state.velocity[1] += sample,
                    "r" => add_r += sample,
                    "theta" => add_theta += sample,
                    _ => panic!(
                        "Unknown variable name in variable_order ({}), should be in [x, y, orientation, velocity | velocity_x | vx, velocity_y | vy, r, theta]",
                        config.variable_order[i]
                    ),
                }
                i += 1;
            }
        }

        if i != 0 && (add_r != 0. || add_theta != 0.) {
            state.pose.z += add_theta;
            state.pose.z = mod2pi(state.pose.z);
            state.pose.x += add_r * state.pose.z.cos();
            state.pose.y += add_r * state.pose.z.sin();
        }
        state
    }

    pub fn theta_modulo(mut self) -> Self {
        self.pose.z = mod2pi(self.pose.z);
        self
    }
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

impl Recordable<StateRecord> for State {
    fn record(&self) -> StateRecord {
        StateRecord {
            pose: {
                let mut ve = [0., 0., 0.];
                for (i, coord) in self.pose.iter().enumerate() {
                    if i >= ve.len() {
                        break;
                    }
                    ve[i] = *coord;
                }
                ve
            },
            velocity: {
                let mut ve = [0., 0.];
                for (i, coord) in self.velocity.iter().enumerate() {
                    if i >= ve.len() {
                        break;
                    }
                    ve[i] = *coord;
                }
                ve
            },
        }
    }
}

use std::collections::BTreeMap;
use std::fmt;

impl fmt::Display for State {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            formatter,
            "pose: [{}, {}, {}], v: [{}, {}]",
            self.pose.x, self.pose.y, self.pose.z, self.velocity[0], self.velocity[1]
        )?;
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct WorldStateRecord {
    pub ego: Option<StateRecord>,
    pub objects: BTreeMap<String, StateRecord>,
    pub landmarks: BTreeMap<i32, StateRecord>,
    pub occupancy_grid: Option<OccupancyGrid>,
}

#[cfg(feature = "gui")]
impl UIComponent for WorldStateRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| {
            if let Some(s) = &self.ego {
                egui::CollapsingHeader::new("Ego").show(ui, |ui| {
                    s.show(ui, ctx, unique_id);
                });
            } else {
                ui.label("ego: None");
            }

            egui::CollapsingHeader::new("Landmarks").show(ui, |ui| {
                for (i, l) in &self.landmarks {
                    egui::CollapsingHeader::new(format!("Landmark {i}")).show(ui, |ui| {
                        l.show(ui, ctx, unique_id);
                    });
                }
            });

            egui::CollapsingHeader::new("Objects").show(ui, |ui| {
                for (n, l) in &self.objects {
                    egui::CollapsingHeader::new(format!("Object {n}")).show(ui, |ui| {
                        l.show(ui, ctx, unique_id);
                    });
                }
            });

            ui.label("Occupancy Grid: Not viewable");
        });
    }
}

/// Full State to be estimated.
#[derive(Debug, Clone, Default)]
pub struct WorldState {
    pub ego: Option<State>,
    pub objects: BTreeMap<String, State>,
    pub landmarks: BTreeMap<i32, State>,
    pub occupancy_grid: Option<OccupancyGrid>,
}

impl WorldState {
    pub fn new() -> Self {
        Self {
            ego: None,
            objects: BTreeMap::new(),
            landmarks: BTreeMap::new(),
            occupancy_grid: None,
        }
    }
}

impl Recordable<WorldStateRecord> for WorldState {
    fn record(&self) -> WorldStateRecord {
        WorldStateRecord {
            ego: self.ego.as_ref().map(|s| s.record()),
            landmarks: BTreeMap::from_iter(self.landmarks.iter().map(|(id, s)| (*id, s.record()))),
            objects: BTreeMap::from_iter(
                self.objects.iter().map(|(id, s)| (id.clone(), s.record())),
            ),
            occupancy_grid: self.occupancy_grid.clone(),
        }
    }
}

#[cfg(feature = "gui")]
use crate::gui::{
    UIComponent,
    utils::{string_combobox, text_singleline_with_apply},
};
use crate::{errors::SimbaResult, node::Node};
use crate::recordable::Recordable;
use crate::simulator::SimulatorConfig;
#[cfg(feature = "gui")]
use crate::utils::enum_tools::ToVec;
use crate::utils::geometry::mod2pi;
use crate::utils::occupancy_grid::OccupancyGrid;
use crate::{
    networking::message_handler::MessageHandler,
    utils::determinist_random_variable::RandomVariableTypeConfig,
};
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use std::sync::{Arc, RwLock};

/// List the possible configs, to be selected in the global config.
///
/// To select the [`StateEstimatorConfig::Perfect`], the yaml config should be:
/// ```YAML
/// state_estimator:
///     Perfect:
///         prediction_period: 0.01
/// ```
#[config_derives]
pub enum StateEstimatorConfig {
    Perfect(perfect_estimator::PerfectEstimatorConfig),
    External(external_estimator::ExternalEstimatorConfig),
    Python(python_estimator::PythonEstimatorConfig),
}

#[cfg(feature = "gui")]
impl UIComponent for StateEstimatorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        let mut current_str = self.to_string();
        ui.horizontal(|ui| {
            ui.label("State Estimator:");
            string_combobox(
                ui,
                &StateEstimatorConfig::to_vec()
                    .iter()
                    .map(|x: &&str| String::from(*x))
                    .collect(),
                &mut current_str,
                format!("state-estimator-choice-{}", unique_id),
            );
        });
        if current_str != self.to_string() {
            match current_str.as_str() {
                "Perfect" => {
                    *self = StateEstimatorConfig::Perfect(
                        perfect_estimator::PerfectEstimatorConfig::default(),
                    )
                }
                "External" => {
                    *self = StateEstimatorConfig::External(
                        external_estimator::ExternalEstimatorConfig::default(),
                    )
                }
                "Python" => {
                    *self = StateEstimatorConfig::Python(
                        python_estimator::PythonEstimatorConfig::default(),
                    )
                }
                _ => panic!("Where did you find this value?"),
            };
        }
        match self {
            StateEstimatorConfig::Perfect(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            StateEstimatorConfig::External(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            StateEstimatorConfig::Python(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
        }
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.horizontal(|ui| {
            ui.label(format!("State Estimator: {}", self));
        });
        match self {
            StateEstimatorConfig::Perfect(c) => c.show(ui, ctx, unique_id),
            StateEstimatorConfig::External(c) => c.show(ui, ctx, unique_id),
            StateEstimatorConfig::Python(c) => c.show(ui, ctx, unique_id),
        }
    }
}

/// List the possible records.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum StateEstimatorRecord {
    Perfect(perfect_estimator::PerfectEstimatorRecord),
    External(external_estimator::ExternalEstimatorRecord),
    Python(python_estimator::PythonEstimatorRecord),
}

#[cfg(feature = "gui")]
impl UIComponent for StateEstimatorRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| match self {
            Self::Perfect(r) => {
                egui::CollapsingHeader::new("Perfect").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
            Self::External(r) => {
                egui::CollapsingHeader::new("ExternalStateEstimator").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
            Self::Python(r) => {
                egui::CollapsingHeader::new("PythonExternalStateEstimator").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
        });
    }
}

/**
 * Make the right [`StateEstimator`] from the configuration given.
 */
pub fn make_state_estimator_from_config(
    config: &StateEstimatorConfig,
    plugin_api: &Option<Arc<dyn PluginAPI>>,
    global_config: &SimulatorConfig,
    va_factory: &Arc<DeterministRandomVariableFactory>,
    initial_time: f32,
) -> SimbaResult<Box<dyn StateEstimator>> {
    Ok(match config {
        StateEstimatorConfig::Perfect(c) => Box::new(
            perfect_estimator::PerfectEstimator::from_config(c, global_config, initial_time),
        ) as Box<dyn StateEstimator>,
        StateEstimatorConfig::External(c) => {
            Box::new(external_estimator::ExternalEstimator::from_config(
                c,
                plugin_api,
                global_config,
                va_factory,
                initial_time,
            )?) as Box<dyn StateEstimator>
        }
        StateEstimatorConfig::Python(c) => Box::new(
            python_estimator::PythonEstimator::from_config(c, global_config, initial_time).unwrap(),
        ) as Box<dyn StateEstimator>,
    })
}

use crate::sensors::Observation;

pub trait StateEstimator:
    std::fmt::Debug
    + std::marker::Send
    + std::marker::Sync
    + Recordable<StateEstimatorRecord>
    + MessageHandler
{
    /// Prediction step of the state estimator.
    ///
    /// The prediction step should be able to compute the state of the node at the given time.
    ///
    /// ## Arguments
    /// * `node` -- mutable reference on the current [`Node`] to be able to interact with
    ///   other modules.
    /// * `time` -- Time to reach.
    fn prediction_step(&mut self, node: &mut Node, time: f32);

    /// Correction step of the state estimator.
    ///
    /// The correction step processes the observations. The received observations can be of every
    /// types, the implementation should not assert a specific type.
    ///
    /// ## Arguments
    /// * `node` -- mutable reference on the current [`Node`] to be able to interact with
    ///   other modules.
    /// * `observations` -- Observation vector.
    /// * `time` -- Current time.
    fn correction_step(&mut self, node: &mut Node, observations: &[Observation], time: f32);

    /// Return the current estimated state.
    fn world_state(&self) -> WorldState;

    /// Return the next prediction step time. The correction step
    /// is called for each observation.
    fn next_time_step(&self) -> f32;

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32);
}

#[config_derives]
pub struct BenchStateEstimatorConfig {
    pub name: String,
    #[check]
    pub config: StateEstimatorConfig,
}

impl Default for BenchStateEstimatorConfig {
    fn default() -> Self {
        Self {
            name: String::from("bench_state_estimator"),
            config: StateEstimatorConfig::Perfect(
                perfect_estimator::PerfectEstimatorConfig::default(),
            ),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for BenchStateEstimatorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new(&self.name).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Name: ");
                text_singleline_with_apply(
                    ui,
                    format!("bench-name-key-{}", unique_id).as_str(),
                    buffer_stack,
                    &mut self.name,
                );
            });

            self.config.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
        });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new(&self.name).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!("Name: {}", self.name));
            });

            self.config.show(ui, ctx, unique_id);
        });
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BenchStateEstimatorRecord {
    pub name: String,
    pub record: StateEstimatorRecord,
}

#[derive(Debug)]
pub struct BenchStateEstimator {
    pub name: String,
    pub state_estimator: Arc<RwLock<Box<dyn StateEstimator>>>,
}
