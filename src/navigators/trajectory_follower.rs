/*!
Implementation of a [`Navigator`] strategy, which follows a polyline shaped
trajectory.
*/

#[cfg(feature = "gui")]
use crate::gui::{utils::path_finder, UIComponent};

use crate::{
    navigators::{
        navigator::{Navigator, NavigatorRecord},
        trajectory::{Trajectory, TrajectoryConfig, TrajectoryRecord},
    },
    plugin_api::PluginAPI,
    simulator::SimulatorConfig,
    utils::determinist_random_variable::DeterministRandomVariableFactory,
    utils::geometry::{mod2pi, smallest_theta_diff},
};

extern crate nalgebra as na;
use config_checker::macros::Check;
use libm::atan2;
use na::Vector3;

use serde_derive::{Deserialize, Serialize};

use std::path::Path;

/// Configuration of the [`TrajectoryFollower`] strategy.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct TrajectoryFollowerConfig {
    /// Path to load the path. The file should be compatible with [`TrajectoryConfig`].
    pub trajectory_path: String,
    /// Distance of the point which is projected on the trajectory.
    #[check(ge(0.))]
    pub forward_distance: f32,
    /// Speed to reach, in m/s.
    #[check(ge(0.))]
    pub target_speed: f32,
}

impl Default for TrajectoryFollowerConfig {
    fn default() -> Self {
        Self {
            trajectory_path: String::from(""),
            forward_distance: 1.0,
            target_speed: 0.5,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for TrajectoryFollowerConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new("Trajectory Follower")
            .id_salt(format!("trajectory-follower-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Trajectory path:");
                    path_finder(ui, &mut self.trajectory_path, &global_config.base_path);
                });

                ui.horizontal(|ui| {
                    ui.label("Forward distance:");
                    if self.forward_distance < 0. {
                        self.forward_distance = 0.;
                    }
                    ui.add(egui::DragValue::new(&mut self.forward_distance).max_decimals(10));
                });

                ui.horizontal(|ui| {
                    ui.label("Target speed:");
                    if self.target_speed < 0. {
                        self.target_speed = 0.;
                    }
                    ui.add(egui::DragValue::new(&mut self.target_speed).max_decimals(10));
                });
            });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, unique_id: &String) {
        egui::CollapsingHeader::new("Trajectory Follower")
            .id_salt(format!("trajectory-follower-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(format!("Trajectory path: {}", self.trajectory_path));
                });

                ui.horizontal(|ui| {
                    ui.label(format!("Forward distance: {}", self.forward_distance));
                });

                ui.horizontal(|ui| {
                    ui.label(format!("Target speed: {}", self.target_speed));
                });
            });
    }
}

/// Record of the [`TrajectoryFollower`].
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TrajectoryFollowerRecord {
    /// Current error
    pub error: ControllerError,
    /// Trajectory dynamic record.
    pub trajectory: TrajectoryRecord,
    pub projected_point: [f32; 2],
}

impl Default for TrajectoryFollowerRecord {
    fn default() -> Self {
        Self {
            error: ControllerError::default(),
            trajectory: TrajectoryRecord::default(),
            projected_point: [0., 0.],
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for TrajectoryFollowerRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &String) {
        ui.vertical(|ui| {
            egui::CollapsingHeader::new("Error").show(ui, |ui| {
                self.error.show(ui, ctx, unique_id);
            });

            egui::CollapsingHeader::new("Trajectory").show(ui, |ui| {
                self.trajectory.show(ui, ctx, unique_id);
            });

            ui.label(format!(
                "Projected point: ({}, {})",
                self.projected_point[0], self.projected_point[1]
            ));
        });
    }
}

/// [`Navigator`] strategy which follows a polyline.
///
/// The lateral error is computed using the projection on the closest
/// segment.
///
/// The orientation error is computed so that the robot goes to the projected
/// point.
#[derive(Debug)]
pub struct TrajectoryFollower {
    /// Trajectory to follow
    trajectory: Trajectory,
    /// Distance of the point which is projected on the trajectory
    /// to compute the error.
    forward_distance: f32,
    /// Speed to reach in m/s
    target_speed: f32,
    /// Last error, stored to make the [`TrajectoryFollowerRecord`]
    error: ControllerError,
    projected_point: [f32; 2],
}

impl TrajectoryFollower {
    /// Makes a new default [`TrajectoryFollower`].
    pub fn new() -> Self {
        Self {
            trajectory: Trajectory::new(),
            forward_distance: 0.2,
            target_speed: 0.5,
            error: ControllerError::default(),
            projected_point: [0., 0.],
        }
    }

    /// Makes a [`TrajectoryFollower`] from the given config.
    ///
    /// ## Arguments
    /// * `config` - Trajectory configuration
    /// * `plugin_api` - Not used there.
    /// * `global_config` - Global configuration of the simulator. Used there to get the
    /// path of the config, used as relative reference for the trajectory path.
    pub fn from_config(
        config: &TrajectoryFollowerConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        _va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        let mut path = Path::new(&config.trajectory_path);
        if config.trajectory_path == "" {
            return Self::new();
        }
        let joined_path = global_config.base_path.join(&config.trajectory_path);
        if path.is_relative() {
            path = joined_path.as_path();
        }
        TrajectoryFollower {
            trajectory: Self::load_trajectory_from_path(&path),
            forward_distance: config.forward_distance,
            target_speed: config.target_speed,
            error: ControllerError::default(),
            projected_point: [0., 0.],
        }
    }

    /// Load the trajectory from the given `path`. This file should be compatible
    /// with [`TrajectoryConfig`].
    fn load_trajectory_from_path(path: &Path) -> Trajectory {
        let trajectory: TrajectoryConfig = match confy::load_path(&path) {
            Ok(config) => config,
            Err(error) => {
                println!(
                    "Error from Confy while loading the trajectory file {} : {}",
                    path.display(),
                    error
                );
                return Trajectory::new();
            }
        };
        Trajectory::from_config(&trajectory)
    }
}

use crate::controllers::controller::ControllerError;
use crate::node::Node;
use crate::state_estimators::state_estimator::WorldState;

impl Navigator for TrajectoryFollower {
    /// Compute the error between the given `state` and the current trajectory.
    ///
    /// This error is computed through the following steps:
    /// 1. Compute the foward pose, using the `forward_distance`.
    /// 2. Use [`Trajectory::map_matching`] to get the matching segment and the projected
    /// point (no orientation).
    /// 3. Compute the orientation of the point to orient the robot to the projected point.
    /// 4. Compute the lateral error
    /// 5. Compute the velocity error
    fn compute_error(&mut self, _robot: &mut Node, world_state: WorldState) -> ControllerError {
        if world_state.ego.is_none() {
            panic!(
                "StateEstimator should provide an ego estimate for TrajectoryFollower navigator."
            )
        }

        let state = world_state.ego.unwrap().theta_modulo();

        // let forward_pose = state.pose
        //     + self.forward_distance * Vector3::new(state.pose.z.cos(), state.pose.z.sin(), 0.);
        let (segment, projected_point, end) = self.trajectory.map_matching(
            state.pose.fixed_view::<2, 1>(0, 0).into(),
            self.forward_distance,
        );
        if end {
            self.target_speed = self
                .target_speed
                .min((state.pose.fixed_view::<2, 1>(0, 0) - projected_point).norm());
        }
        let segment_angle: f32 = atan2(
            (segment.1.y - segment.0.y).into(),
            (segment.1.x - segment.0.x).into(),
        ) as f32;
        let projected_point = Vector3::new(projected_point.x, projected_point.y, segment_angle);
        // Compute the orientation error
        let mut projected_point_direction = atan2(
            (projected_point.y - state.pose.y).into(),
            (projected_point.x - state.pose.x).into(),
        ) as f32;

        self.projected_point = [projected_point.x, projected_point.y];

        projected_point_direction = mod2pi(projected_point_direction);

        let theta_error = smallest_theta_diff(projected_point_direction, state.pose.z);

        let theta_error = mod2pi(theta_error);
        self.error.theta = theta_error;

        // Compute the lateral error
        let pose_with_segment: f32 = segment_angle
            - atan2(
                (state.pose.y - segment.0.y).into(),
                (state.pose.x - segment.0.x).into(),
            ) as f32;
        self.error.lateral = ((state.pose.x - projected_point.x).powf(2.)
            + (state.pose.y - projected_point.y).powf(2.))
        .sqrt();
        if pose_with_segment < 0. {
            self.error.lateral *= -1.;
        }

        // Compute the velocity error
        self.error.velocity = self.target_speed - state.velocity;

        self.error.clone()
    }
}

use crate::recordable::Recordable;

impl Recordable<NavigatorRecord> for TrajectoryFollower {
    fn record(&self) -> NavigatorRecord {
        NavigatorRecord::TrajectoryFollower(TrajectoryFollowerRecord {
            error: self.error.clone(),
            trajectory: self.trajectory.record(),
            projected_point: self.projected_point,
        })
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn compute_error() {
        // let navigator = TrajectoryFollower {
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
