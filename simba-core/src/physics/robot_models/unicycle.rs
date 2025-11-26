use config_checker::macros::Check;
use libm::atan2f;
use nalgebra::SMatrix;
use serde::{Deserialize, Serialize};

#[cfg(feature = "gui")]
use crate::{gui::UIComponent, simulator::SimulatorConfig};
use crate::{
    physics::robot_models::{Command, RobotModel},
    state_estimators::State,
};

/// Command struct, to control both wheel speed, in m/s.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct UnicycleCommand {
    /// Left wheel speed.
    pub left_wheel_speed: f32,
    /// Right wheel speed.
    pub right_wheel_speed: f32,
}

impl Default for UnicycleCommand {
    fn default() -> Self {
        Self {
            left_wheel_speed: 0.,
            right_wheel_speed: 0.,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for UnicycleCommand {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!("Left wheel speed: {}", self.left_wheel_speed));
            ui.label(format!("Right wheel speed: {}", self.right_wheel_speed));
        });
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct UnicycleConfig {
    /// Distance between the two wheels, to compute the angular velocity from the wheel speeds.
    #[check(ge(0.))]
    pub wheel_distance: f32,
}

#[cfg(feature = "gui")]
impl UIComponent for UnicycleConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Unicycle model")
            .id_salt(format!("unicycle-model-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Wheel distance:");
                    if self.wheel_distance < 0. {
                        self.wheel_distance = 0.;
                    }
                    ui.add(egui::DragValue::new(&mut self.wheel_distance));
                });
            });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new("Unicycle model")
            .id_salt(format!("unicycle-model-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(format!("Wheel distance: {}", self.wheel_distance));
                });
            });
    }
}

impl Default for UnicycleConfig {
    fn default() -> Self {
        Self {
            wheel_distance: 0.25,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Unicycle {
    wheel_distance: f32,
}

impl Unicycle {
    pub fn from_config(config: &UnicycleConfig) -> Self {
        Self {
            wheel_distance: config.wheel_distance,
        }
    }
}

impl RobotModel for Unicycle {
    fn update_state(&mut self, state: &mut State, command: &Command, dt: f32) {
        let command = match command {
            Command::Unicycle(cmd) => cmd,
            _ => panic!("Unicycle robot model needs a Unicycle command"),
        };
        let theta = state.pose.z;

        let displacement_wheel_left = command.left_wheel_speed * dt;
        let displacement_wheel_right = command.right_wheel_speed * dt;

        let translation = (displacement_wheel_left + displacement_wheel_right) / 2.;
        let rotation = (displacement_wheel_right - displacement_wheel_left) / self.wheel_distance;

        // Using Lie theory
        // Reference: Sola, J., Deray, J., & Atchuthan, D. (2018). A micro lie theory for state estimation in robotics. arXiv preprint arXiv:1812.01537.

        let lie_action =
            SMatrix::<f32, 3, 3>::new(0., -rotation, translation, rotation, 0., 0., 0., 0., 0.);

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

        state.velocity = translation / dt;
    }

    fn default_command(&self) -> Command {
        Command::Unicycle(UnicycleCommand {
            left_wheel_speed: 0.,
            right_wheel_speed: 0.,
        })
    }
}
