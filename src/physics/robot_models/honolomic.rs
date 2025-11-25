use config_checker::macros::Check;
use libm::atan2f;
use nalgebra::SMatrix;
use serde::{Deserialize, Serialize};

#[cfg(feature = "gui")]
use crate::{gui::UIComponent, simulator::SimulatorConfig};
use crate::{
    physics::robot_models::{Command, RobotModel},
    state_estimators::state_estimator::State,
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

#[derive(Serialize, Deserialize, Debug, Clone, Check, Default)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct HonolomicConfig {}

#[cfg(feature = "gui")]
impl UIComponent for HonolomicConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Honolomic model")
            .id_salt(format!("honolomic-model-{}", unique_id))
            .show(ui, |_ui| {});
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new("Honolomic model")
            .id_salt(format!("honolomic-model-{}", unique_id))
            .show(ui, |_ui| {});
    }
}

#[derive(Debug, Clone)]
pub struct Honolomic {}

impl Honolomic {
    pub fn from_config(_config: &HonolomicConfig) -> Self {
        Self {}
    }
}

impl RobotModel for Honolomic {
    fn update_state(&mut self, state: &mut State, command: &Command, dt: f32) {
        let command = match command {
            Command::Honolomic(cmd) => cmd,
            _ => panic!("Honolomic robot model needs a Honolomic command"),
        };
        let theta = state.pose.z;

        let displacement_lateral = command.lateral_velocity * dt;
        let displacement_longitudinal = command.longitudinal_velocity * dt;
        let rotation = command.angular_velocity * dt;

        // Using Lie theory
        // Reference: Sola, J., Deray, J., & Atchuthan, D. (2018). A micro lie theory for state estimation in robotics. arXiv preprint arXiv:1812.01537.

        let lie_action = SMatrix::<f32, 3, 3>::new(
            0.,
            -rotation,
            displacement_longitudinal,
            rotation,
            0.,
            displacement_lateral,
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

        se2_mat *= lie_action.exp();

        // let rot = nalgebra::Rotation2::from_matrix(&se2_mat.fixed_view::<2, 2>(0, 0).into());
        // self.state.pose.z = rot.angle();
        state.pose.z = atan2f(se2_mat[(1, 0)], se2_mat[(0, 0)]);

        state.pose.x = se2_mat[(0, 2)];
        state.pose.y = se2_mat[(1, 2)];

        state.velocity =
            (command.longitudinal_velocity.powi(2) + command.lateral_velocity.powi(2)).sqrt();
    }

    fn default_command(&self) -> Command {
        Command::Honolomic(HolonomicCommand {
            angular_velocity: 0.,
            lateral_velocity: 0.,
            longitudinal_velocity: 0.,
        })
    }
}
