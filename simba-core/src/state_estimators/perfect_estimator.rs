/*!
Module providing the [`PerfectEstimator`] strategy. This strategy uses directly
the groundtruth to provide the estimation. It can be used when the state used
by the controller should be perfect.
*/

use std::path::Path;

use super::{State, WorldState, WorldStateRecord};
use crate::{
    constants::TIME_ROUND, errors::SimbaErrorTypes, networking::service_manager::ServiceError, physics::robot_models::Command,
};

#[cfg(feature = "gui")]
use crate::gui::{
    UIComponent,
    utils::{path_finder, string_checkbox},
};
use crate::recordable::Recordable;
use crate::sensors::Observation;
use crate::sensors::oriented_landmark_sensor::OrientedLandmarkSensor;
use crate::simulator::SimulatorConfig;
use crate::utils::maths::round_precision;
use log::{error, info, warn};
use serde_derive::{Deserialize, Serialize};
use simba_macros::config_derives;

/// Configuration for [`PerfectEstimator`].
#[config_derives]
pub struct PerfectEstimatorConfig {
    /// Prediction period.
    #[check(ge(0.))]
    pub prediction_period: f32,
    pub targets: Vec<String>,
    pub map_path: Option<String>,
}

impl Default for PerfectEstimatorConfig {
    fn default() -> Self {
        Self {
            prediction_period: 0.1,
            targets: vec!["self".to_string()],
            map_path: None,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PerfectEstimatorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Perfect Estimator")
            .id_salt(format!("perfect-estimator-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Prediction period:");
                    if self.prediction_period < TIME_ROUND {
                        self.prediction_period = TIME_ROUND;
                    }
                    ui.add(
                        egui::DragValue::new(&mut self.prediction_period)
                            .max_decimals((1. / TIME_ROUND) as usize),
                    );
                });

                let mut possible_targets =
                    Vec::from_iter(global_config.robots.iter().map(|x| x.name.clone()));
                possible_targets.insert(0, "self".to_string());
                if let Some(idx) = possible_targets
                    .iter()
                    .position(|x| x == current_node_name.unwrap())
                {
                    possible_targets.remove(idx);
                }

                ui.horizontal_wrapped(|ui| {
                    ui.label("Targets:");
                    string_checkbox(ui, &possible_targets, &mut self.targets);
                });

                ui.horizontal_wrapped(|ui| {
                    ui.label("Map path:");
                    let mut enabled = self.map_path.is_some();
                    ui.checkbox(&mut enabled, "Enable");
                    if enabled {
                        path_finder(
                            ui,
                            self.map_path.as_mut().unwrap(),
                            &global_config.base_path,
                        );
                    } else {
                        self.map_path = None;
                    }
                });
            });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new("Perfect Estimator")
            .id_salt(format!("perfect-estimator-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(format!("Prediction period: {}", self.prediction_period));
                });

                ui.horizontal_wrapped(|ui| {
                    ui.label("Targets: ");
                    for t in &self.targets {
                        ui.label(format!("{t}, "));
                    }
                });

                ui.horizontal_wrapped(|ui| {
                    ui.label("Map path: ");
                    if let Some(p) = &self.map_path {
                        ui.label(p);
                    } else {
                        ui.label("None");
                    }
                });
            });
    }
}

/// Record for [`PerfectEstimator`].
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PerfectEstimatorRecord {
    /// Current state estimated
    pub world_state: WorldStateRecord,
    /// Last change of state
    pub last_time_prediction: f32,
}

#[cfg(feature = "gui")]
impl UIComponent for PerfectEstimatorRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| {
            egui::CollapsingHeader::new("World state").show(ui, |ui| {
                self.world_state.show(ui, ctx, unique_id);
            });
            ui.label(format!(
                "Last prediction time: {}",
                self.last_time_prediction
            ));
        });
    }
}

/// Estimation strategy without any error.
#[derive(Debug)]
pub struct PerfectEstimator {
    /// Estimation of the state on the `last_time_prediction`.
    world_state: WorldState,
    /// Prediction period, in seconds.
    prediction_period: f32,
    /// Last time the state was updated/predicted.
    last_time_prediction: f32,
}

impl PerfectEstimator {
    /// Create a new [`PerfectEstimator`] using default [`PerfectEstimatorConfig`].
    pub fn new() -> Self {
        Self::from_config(
            &PerfectEstimatorConfig::default(),
            &SimulatorConfig::default(),
            0.0,
        )
    }

    /// Creates a new [`PerfectEstimator`] from the given `config`.
    pub fn from_config(
        config: &PerfectEstimatorConfig,
        global_config: &SimulatorConfig,
        initial_time: f32,
    ) -> Self {
        let mut world_state = WorldState::new();
        for target in &config.targets {
            if target == "self" {
                world_state.ego = Some(State::new());
            } else {
                world_state.objects.insert(target.clone(), State::new());
            }
        }
        if let Some(map_path) = &config.map_path {
            let mut map_path = Path::new(map_path);
            let joined_path = global_config.base_path.join(map_path);
            if map_path.is_relative() {
                map_path = joined_path.as_path();
            }
            let landmarks = OrientedLandmarkSensor::load_map_from_path(map_path);
            for landmark in landmarks {
                world_state.landmarks.insert(
                    landmark.id,
                    State {
                        pose: landmark.pose,
                        velocity: [0., 0., 0.].into(),
                    },
                );
            }
        }

        Self {
            prediction_period: config.prediction_period,
            world_state,
            last_time_prediction: initial_time,
        }
    }
}

impl Default for PerfectEstimator {
    fn default() -> Self {
        Self::new()
    }
}

use super::{StateEstimator, StateEstimatorRecord};
use crate::node::Node;

impl StateEstimator for PerfectEstimator {
    fn prediction_step(&mut self, node: &mut Node, _command: Option<Command>, time: f32) {
        if (time - self.next_time_step()).abs() > TIME_ROUND / 2. {
            error!("Error trying to update estimate too soon !");
            return;
        }
        info!("Doing prediction step");
        if let Some(ego) = &mut self.world_state.ego {
            let arc_physic = node
                .physics()
                .expect("Node with state_estimator should have physics");
            let physic = arc_physic.read().unwrap();

            *ego = physic.state(time).clone();
        }
        let mut objects_to_delete = Vec::new();
        for (target, state) in &mut self.world_state.objects {
            *state = match node
                .service_manager()
                .read()
                .unwrap()
                .get_real_state(target, node, time)
            {
                Err(e) => match e.error_type() {
                    SimbaErrorTypes::ServiceError(ServiceError::Closed) => {
                        warn!(
                            "[{}] {target} does not have physics anymore, no perfect state can be computed: delete target from list!",
                            node.name()
                        );
                        objects_to_delete.push(target.clone());
                        state.clone()
                    }
                    _ => {
                        panic!(
                            "[{}] {target} does not have physics, no perfect state can be computed:\n{}",
                            node.name(),
                            e.detailed_error()
                        )
                    }
                },
                Ok(s) => s,
            };
        }
        for obj in objects_to_delete {
            self.world_state.objects.remove(&obj);
        }
        self.last_time_prediction = time;
    }

    fn correction_step(&mut self, _node: &mut Node, _observations: &[Observation], _time: f32) {}

    fn world_state(&self) -> WorldState {
        self.world_state.clone()
    }

    fn next_time_step(&self) -> f32 {
        round_precision(
            self.last_time_prediction + self.prediction_period,
            TIME_ROUND,
        )
        .unwrap()
    }

    fn pre_loop_hook(&mut self, _node: &mut Node, _time: f32) {}
}

impl Recordable<StateEstimatorRecord> for PerfectEstimator {
    fn record(&self) -> StateEstimatorRecord {
        StateEstimatorRecord::Perfect(PerfectEstimatorRecord {
            world_state: self.world_state.record(),
            last_time_prediction: self.last_time_prediction,
        })
    }
}
