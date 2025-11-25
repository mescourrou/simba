/*!
Implementation of a [`Navigator`] strategy, which follows a polyline shaped
trajectory.
*/

use std::sync::{
    mpsc::{self, Receiver, Sender},
    Arc, Mutex,
};

#[cfg(feature = "gui")]
use crate::{gui::UIComponent, simulator::SimulatorConfig};

use crate::{
    navigators::navigator::{Navigator, NavigatorRecord},
    networking::{message_handler::MessageHandler, network::Envelope},
    utils::geometry::smallest_theta_diff,
};

extern crate nalgebra as na;
use config_checker::macros::Check;
use libm::atan2;

use nalgebra::SVector;
use pyo3::{pyclass, pymethods};
use serde_derive::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass(get_all, set_all)]
pub struct GoToMessage {
    pub target_point: Option<[f32; 2]>,
}

#[pymethods]
impl GoToMessage {
    #[new]
    #[pyo3(signature = (list=None))]
    pub fn new(list: Option<[f32; 2]>) -> Self {
        Self { target_point: list }
    }
}

/// Configuration of the [`GoTo`] strategy.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct GoToConfig {
    pub target_point: Option<[f32; 2]>,
    #[check(ge(0.))]
    pub target_speed: f32,
    /// Distance where to stop when reaching the end of the trajectory
    #[check(ge(0.))]
    pub stop_distance: f32,
    /// Coefficient of the target velocity, multiplied by the remaining distance
    #[check(ge(0.))]
    pub stop_ramp_coefficient: f32,
}

impl Default for GoToConfig {
    fn default() -> Self {
        Self {
            target_point: None,
            target_speed: 0.5,
            stop_distance: 0.2,
            stop_ramp_coefficient: 0.5,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for GoToConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Go To")
            .id_salt(format!("go-to-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    use crate::gui::utils::string_combobox;

                    ui.label("Target point: ".to_string());
                    let possible_values = vec!["None".to_string(), "Some".to_string()];
                    let buffer_key = format!("go-to-target-point-option-{unique_id}");
                    if !buffer_key.contains(&buffer_key) {
                        buffer_stack.insert(buffer_key.clone(), possible_values[0].clone());
                    }
                    let value = buffer_stack.get_mut(&buffer_key).unwrap();
                    string_combobox(ui, &possible_values, value, format!("go-to-{}", unique_id));
                    if *value == possible_values[0] {
                        self.target_point = None;
                    } else {
                        use egui::DragValue;

                        if self.target_point.is_none() {
                            self.target_point = Some([0., 0.]);
                        }
                        let point = &mut self.target_point.unwrap();
                        ui.label("(".to_string());
                        ui.add(DragValue::new(&mut point[0]).fixed_decimals(2));
                        ui.label(", ".to_string());
                        ui.add(DragValue::new(&mut point[1]).fixed_decimals(2));
                        ui.label(")".to_string());
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("Target speed:");
                    if self.target_speed < 0. {
                        self.target_speed = 0.;
                    }
                    ui.add(egui::DragValue::new(&mut self.target_speed).max_decimals(10));
                });

                ui.horizontal(|ui| {
                    ui.label("Stop distance:");
                    if self.stop_distance < 0. {
                        self.stop_distance = 0.;
                    }
                    ui.add(egui::DragValue::new(&mut self.stop_distance).max_decimals(10));
                });

                ui.horizontal(|ui| {
                    ui.label("Stop ramp coefficient:");
                    if self.stop_ramp_coefficient < 0. {
                        self.stop_ramp_coefficient = 0.;
                    }
                    ui.add(egui::DragValue::new(&mut self.stop_ramp_coefficient).max_decimals(10));
                });
            });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new("Go To")
            .id_salt(format!("go-to-{}", unique_id))
            .show(ui, |ui| {
                if let Some(target_point) = &self.target_point {
                    ui.label(format!(
                        "Target point: ({}, {})",
                        target_point[0], target_point[1]
                    ));
                } else {
                    ui.label("Target point: None".to_string());
                }

                ui.horizontal(|ui| {
                    ui.label(format!("Target speed: {}", self.target_speed));
                });

                ui.horizontal(|ui| {
                    ui.label(format!("Stop distance: {}", self.stop_distance));
                });

                ui.horizontal(|ui| {
                    ui.label(format!(
                        "Stop ramp coefficient: {}",
                        self.stop_ramp_coefficient
                    ));
                });
            });
    }
}

/// Record of the [`GoTo`].
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct GoToRecord {
    /// Current error
    pub error: ControllerError,
    pub current_point: Option<[f32; 2]>,
}

#[cfg(feature = "gui")]
impl UIComponent for GoToRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| {
            egui::CollapsingHeader::new("Error").show(ui, |ui| {
                self.error.show(ui, ctx, unique_id);
            });

            if let Some(current_point) = &self.current_point {
                ui.label(format!(
                    "Current point: ({}, {})",
                    current_point[0], current_point[1]
                ));
            } else {
                ui.label("Current point: None".to_string());
            }
        });
    }
}

/// [`Navigator`] strategy which goes to a given point. Can be received by message.
///
/// The lateral error is always 0.
#[derive(Debug)]
pub struct GoTo {
    /// Speed to reach in m/s
    target_speed: f32,
    /// Last error, stored to make the [`GoToRecord`]
    error: ControllerError,
    current_point: Option<[f32; 2]>,
    /// Distance where to stop when reaching the end of the trajectory
    stop_distance: f32,
    /// Coefficient of the target velocity, multiplied by the remaining distance
    stop_ramp_coefficient: f32,

    letter_box: Arc<Mutex<Receiver<Envelope>>>,
    letter_box_sender: Sender<Envelope>,
}

impl GoTo {
    /// Makes a new default [`GoTo`].
    pub fn new() -> Self {
        let (tx, rx) = mpsc::channel();
        Self {
            target_speed: 0.5,
            stop_distance: 0.2,
            stop_ramp_coefficient: 0.5,
            error: ControllerError::default(),
            current_point: None,
            letter_box: Arc::new(Mutex::new(rx)),
            letter_box_sender: tx,
        }
    }

    /// Makes a [`GoTo`] from the given config.
    ///
    /// ## Arguments
    /// * `config` - GoTo configuration
    /// * `plugin_api` - Not used there.
    /// * `global_config` - Global configuration of the simulator.
    pub fn from_config(config: &GoToConfig) -> Self {
        let (tx, rx) = mpsc::channel();
        Self {
            target_speed: config.target_speed,
            error: ControllerError::default(),
            current_point: config.target_point,
            stop_distance: config.stop_distance,
            stop_ramp_coefficient: config.stop_ramp_coefficient,
            letter_box: Arc::new(Mutex::new(rx)),
            letter_box_sender: tx,
        }
    }
}

impl Default for GoTo {
    fn default() -> Self {
        Self::new()
    }
}

use crate::controllers::controller::ControllerError;
use crate::node::Node;
use crate::state_estimators::state_estimator::WorldState;

impl Navigator for GoTo {
    /// Compute the error between the given `state` and the target point.
    ///
    fn compute_error(&mut self, _robot: &mut Node, world_state: WorldState) -> ControllerError {
        if world_state.ego.is_none() {
            panic!("StateEstimator should provide an ego estimate for GoTo navigator.")
        }

        let state = world_state.ego.unwrap().theta_modulo();

        if self.current_point.is_none() {
            return ControllerError {
                longitudinal: 0.,
                lateral: 0.,
                theta: 0.,
                velocity: state.velocity,
            };
        }
        let target_point = SVector::from_row_slice(&self.current_point.unwrap());
        let distance_to_final = (state.pose.fixed_view::<2, 1>(0, 0) - target_point).norm();
        self.target_speed = self
            .target_speed
            .min(distance_to_final * self.stop_ramp_coefficient);

        if distance_to_final < self.stop_distance {
            self.target_speed = 0.;
        }

        let target_direction = atan2(
            (target_point[1] - state.pose.y).into(),
            (target_point[0] - state.pose.x).into(),
        ) as f32;

        let theta_error = smallest_theta_diff(target_direction, state.pose.z);

        self.error.theta = theta_error;

        self.error.lateral = 0.;

        // Compute the velocity error
        self.error.velocity = self.target_speed - state.velocity;

        self.error.clone()
    }

    fn pre_loop_hook(&mut self, _node: &mut Node, _time: f32) {
        while let Ok(envelope) = self.letter_box.lock().unwrap().try_recv() {
            if let Ok(msg) = serde_json::from_value::<GoToMessage>(envelope.message) {
                self.current_point = msg.target_point;
                log::info!("Update target point to {:?}", self.current_point);
            }
        }
    }
}

use crate::recordable::Recordable;

impl Recordable<NavigatorRecord> for GoTo {
    fn record(&self) -> NavigatorRecord {
        NavigatorRecord::GoTo(GoToRecord {
            error: self.error.clone(),
            current_point: self.current_point,
        })
    }
}

impl MessageHandler for GoTo {
    fn get_letter_box(&self) -> Option<Sender<Envelope>> {
        Some(self.letter_box_sender.clone())
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn compute_error() {
        // let navigator = GoTo {
        //     trajectory: trajectory::tests::default_trajectory(),
        //     forward_distance: 1.,
        //     target_speed: 1.
        // };

        // // 1st segment
        // let pose = SVector::<f32, 3>::new(0., -0.5, 0.);
        // let error = navigator.compute_error(pose);
        // assert_eq!(error[0], -0.5, "lateral error should be 0.5, but is {}", error[0]);
        // assert_eq!(error[1], 0., "angle error should be 0., but is {}", error[1]);

        // // 1st segment, with angle
        // let pose = SVector::<f32, 3>::new(1., 0.5, PI / 4.);
        // let error = navigator.compute_error(pose);
        // assert_eq!(error[0], 0.5 + navigator.forward_distance * (PI / 4.).sin(), "lateral error should be {}, but is {}", 0.5 + navigator.forward_distance * (PI / 4.).sin(), error[0]);
        // assert_eq!(error[1], PI / 4., "angle error should be {}., but is {}", PI / 4., error[1]);

        // // 2nd segment
        // let pose = SVector::<f32, 3>::new(6., 1., PI / 2.);
        // let error = navigator.compute_error(pose);
        // assert_eq!(error[0], -1., "lateral error should be -1., but is {}", error[0]);
        // assert_eq!(error[1], 0., "angle error should be 0., but is {}", error[1]);

        // // Last segment (with looping)
        // let pose = SVector::<f32, 3>::new(-1., 3., -PI / 2.);
        // let error = navigator.compute_error(pose);
        // assert_eq!(error[0], -1., "lateral error should be -1., but is {}", error[0]);
        // assert_eq!(error[1], 0., "angle error should be 0., but is {}", error[1]);
    }
}
