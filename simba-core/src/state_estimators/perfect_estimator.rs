/*!
Module providing the [`PerfectEstimator`] strategy. This strategy uses directly
the groundtruth to provide the estimation. It can be used when the state used
by the controller should be perfect.
*/

use super::{State, WorldState, WorldStateRecord};
use crate::{
    constants::TIME_ROUND,
    errors::SimbaErrorTypes,
    networking::service_manager::ServiceError,
    physics::robot_models::Command,
    utils::{
        determinist_random_variable::DeterministRandomVariableFactory,
        periodicity::{Periodicity, PeriodicityConfig},
    },
};

#[cfg(feature = "gui")]
use crate::gui::{
    UIComponent,
    utils::string_checkbox,
};
use crate::recordable::Recordable;
use crate::sensors::Observation;
use crate::simulator::SimulatorConfig;
use log::{error, info, warn};
use serde_derive::{Deserialize, Serialize};
use simba_macros::config_derives;

/// Configuration for [`PerfectEstimator`].
#[config_derives]
pub struct PerfectEstimatorConfig {
    /// Prediction period.
    pub prediction_activation: Option<PeriodicityConfig>,
    pub targets: Vec<String>,
}

impl Default for PerfectEstimatorConfig {
    fn default() -> Self {
        Self {
            prediction_activation: Some(PeriodicityConfig {
                period: crate::config::NumberConfig::Num(0.1),
                offset: None,
                table: None,
            }),
            targets: vec!["self".to_string()],
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
                    ui.label("Prediction activation:");
                    if let Some(p) = &mut self.prediction_activation {
                        p.show_mut(
                            ui,
                            _ctx,
                            _buffer_stack,
                            global_config,
                            current_node_name,
                            unique_id,
                        );
                        if ui.button("Remove").clicked() {
                            self.prediction_activation = None;
                        }
                    } else {
                        ui.label("None");
                        if ui.button("Add").clicked() {
                            self.prediction_activation = Self::default().prediction_activation;
                        }
                    }
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
            });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new("Perfect Estimator")
            .id_salt(format!("perfect-estimator-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Prediction activation:");
                    if let Some(p) = &self.prediction_activation {
                        p.show(ui, _ctx, unique_id);
                    } else {
                        ui.label("None");
                    }
                });

                ui.horizontal_wrapped(|ui| {
                    ui.label("Targets: ");
                    for t in &self.targets {
                        ui.label(format!("{t}, "));
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
    prediction_activation: Option<Periodicity>,
    /// Last time the state was updated/predicted.
    last_time_prediction: f32,
}

impl PerfectEstimator {
    /// Creates a new [`PerfectEstimator`] from the given `config`.
    pub fn from_config(
        config: &PerfectEstimatorConfig,
        _global_config: &SimulatorConfig,
        va_factory: &DeterministRandomVariableFactory,
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

        let activation = config
            .prediction_activation
            .as_ref()
            .map(|p| Periodicity::from_config(p, va_factory, initial_time));
        let last_time = activation
            .as_ref()
            .map(|p| p.next_time())
            .unwrap_or(initial_time);
        Self {
            prediction_activation: activation,
            world_state,
            last_time_prediction: last_time,
        }
    }
}

use super::{StateEstimator, StateEstimatorRecord};
use crate::node::Node;

impl StateEstimator for PerfectEstimator {
    fn prediction_step(&mut self, node: &mut Node, _command: Option<Command>, time: f32) {
        if (time - self.next_time_step()).abs() > TIME_ROUND / 2. {
            error!(
                "Error trying to update estimate too soon! (it is {} but expecting {})",
                time,
                self.next_time_step()
            );
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

        let landmarks = node.environment().map().landmarks.clone();
        self.world_state.landmarks = landmarks
            .iter()
            .enumerate()
            .map(|(i, l)| (i as i32, State::from_vector(l.pose.as_slice())))
            .collect();

        if let Some(p) = self.prediction_activation.as_mut() {
            p.update(time);
        }
        self.last_time_prediction = time;
    }

    fn correction_step(&mut self, _node: &mut Node, _observations: &[Observation], _time: f32) {}

    fn world_state(&self) -> WorldState {
        self.world_state.clone()
    }

    fn next_time_step(&self) -> f32 {
        if let Some(period) = &self.prediction_activation {
            period.next_time()
        } else {
            f32::INFINITY
        }
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
