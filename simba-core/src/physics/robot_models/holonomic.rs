use libm::atan2f;
use nalgebra::SMatrix;
use serde::{Deserialize, Serialize};
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::{gui::UIComponent, simulator::SimulatorConfig};
use crate::{
    physics::robot_models::{Command, RobotModel},
    state_estimators::State,
};

/// Command struct, to control the robot using velocity in both directions.
///
/// x is the axis oriented
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct HolonomicCommand {
    pub longitudinal_velocity: f32,
    pub lateral_velocity: f32,
    pub angular_velocity: f32,
}

#[cfg(feature = "gui")]
impl UIComponent for HolonomicCommand {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!(
                "Longitudinal speed: {}",
                self.longitudinal_velocity
            ));
            ui.label(format!("Lateral speed: {}", self.lateral_velocity));
            ui.label(format!("Angular speed: {}", self.angular_velocity));
        });
    }
}

#[config_derives]
pub struct HolonomicConfig {
    pub max_longitudinal_velocity: f32,
    pub max_lateral_velocity: f32,
    pub max_angular_velocity: f32,
}

impl Default for HolonomicConfig {
    fn default() -> Self {
        Self {
            max_longitudinal_velocity: 10.,
            max_lateral_velocity: 10.,
            max_angular_velocity: 1.,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for HolonomicConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Holonomic model")
            .id_salt(format!("holonomic-model-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Max longitudinal velocity:");
                    ui.add(egui::DragValue::new(&mut self.max_longitudinal_velocity).speed(0.1));
                });
                ui.horizontal(|ui| {
                    ui.label("Max lateral velocity:");
                    ui.add(egui::DragValue::new(&mut self.max_lateral_velocity).speed(0.1));
                });
                ui.horizontal(|ui| {
                    ui.label("Max angular velocity:");
                    ui.add(egui::DragValue::new(&mut self.max_angular_velocity).speed(0.01));
                });
            });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new("Holonomic model")
            .id_salt(format!("holonomic-model-{}", unique_id))
            .show(ui, |ui| {
                ui.label(format!(
                    "Max longitudinal velocity: {}",
                    self.max_longitudinal_velocity
                ));
                ui.label(format!(
                    "Max lateral velocity: {}",
                    self.max_lateral_velocity
                ));
                ui.label(format!(
                    "Max angular velocity: {}",
                    self.max_angular_velocity
                ));
            });
    }
}

#[derive(Debug, Clone)]
pub struct Holonomic {
    max_longitudinal_velocity: f32,
    max_lateral_velocity: f32,
    max_angular_velocity: f32,
}

impl Holonomic {
    pub fn from_config(config: &HolonomicConfig) -> Self {
        Self {
            max_longitudinal_velocity: config.max_longitudinal_velocity,
            max_lateral_velocity: config.max_lateral_velocity,
            max_angular_velocity: config.max_angular_velocity,
        }
    }
}

impl RobotModel for Holonomic {
    fn update_state(&mut self, state: &mut State, command: &Command, dt: f32) {
        let command = match command {
            Command::Holonomic(cmd) => cmd,
            _ => panic!("Holonomic robot model needs a Holonomic command"),
        };
        let theta = state.pose.z;

        let lateral_velocity = command
            .lateral_velocity
            .min(self.max_lateral_velocity)
            .max(-self.max_lateral_velocity);
        let longitudinal_velocity = command
            .longitudinal_velocity
            .min(self.max_longitudinal_velocity)
            .max(-self.max_longitudinal_velocity);
        let v_rotation = command
            .angular_velocity
            .min(self.max_angular_velocity)
            .max(-self.max_angular_velocity);

        // Using Lie theory
        // Reference: Sola, J., Deray, J., & Atchuthan, D. (2018). A micro lie theory for state estimation in robotics. arXiv preprint arXiv:1812.01537.

        let lie_action = SMatrix::<f32, 3, 3>::new(
            0.,
            -v_rotation,
            longitudinal_velocity,
            v_rotation,
            0.,
            lateral_velocity,
            0.,
            0.,
            0.,
        );
        let rot_mat = *nalgebra::Rotation2::new(theta).matrix();

        let mut se2_mat = SMatrix::<f32, 3, 3>::new(
            rot_mat[(0, 0)],
            rot_mat[(0, 1)],
            state.pose.x,
            rot_mat[(1, 0)],
            rot_mat[(1, 1)],
            state.pose.y,
            0.,
            0.,
            1.,
        );

        se2_mat *= (dt * lie_action).exp();

        // let rot = nalgebra::Rotation2::from_matrix(&se2_mat.fixed_view::<2, 2>(0, 0).into());
        // self.state.pose.z = rot.angle();
        state.pose.z = atan2f(se2_mat[(1, 0)], se2_mat[(0, 0)]);

        state.pose.x = se2_mat[(0, 2)];
        state.pose.y = se2_mat[(1, 2)];

        state.velocity = [longitudinal_velocity, lateral_velocity, v_rotation].into();
    }

    fn default_command(&self) -> Command {
        Command::Holonomic(HolonomicCommand {
            angular_velocity: 0.,
            lateral_velocity: 0.,
            longitudinal_velocity: 0.,
        })
    }
}
