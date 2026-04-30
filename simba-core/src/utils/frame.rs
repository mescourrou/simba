#[cfg(feature = "gui")]
use std::f32::consts::PI;
use std::ops::Mul;

use nalgebra::{ArrayStorage, Const, Isometry2, Storage};
use serde::{Deserialize, Serialize};
use simba_macros::config_derives;

use crate::{
    context::Context, recordable::Recordable, state_estimators::State, utils::geometry::mod2pi,
};
#[cfg(feature = "gui")]
use crate::{gui::UIComponent, simulator::SimulatorConfig};

/// Configuration for a 2D frame, including position and orientation.
#[config_derives]
#[derive(Copy)]
pub struct FrameConfig {
    /// The x-coordinate of the frame's position.
    /// Default is `0.0`.
    pub x: f32,
    /// The y-coordinate of the frame's position.
    /// Default is `0.0`.
    pub y: f32,
    /// The orientation of the frame in radians.
    /// Default is `0.0` (facing along the positive x-axis).
    pub theta: f32,
}

impl Default for FrameConfig {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for FrameConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Frame")
            .id_salt(format!("frame-{}", unique_id))
            .show(ui, |ui| {
                ui.vertical(|ui| {
                    ui.horizontal(|ui| {
                        ui.label("X:");
                        ui.add(egui::DragValue::new(&mut self.x).speed(0.1));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Y:");
                        ui.add(egui::DragValue::new(&mut self.y).speed(0.1));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Theta:");
                        ui.add(egui::DragValue::new(&mut self.theta).speed(0.01));
                        if self.theta < -PI {
                            self.theta = -PI;
                        } else if self.theta > PI {
                            self.theta = PI;
                        }
                    });
                });
            });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new("Frame")
            .id_salt(format!("frame-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("X:");
                    ui.label(self.x.to_string());
                });
                ui.horizontal(|ui| {
                    ui.label("Y:");
                    ui.label(self.y.to_string());
                });
                ui.horizontal(|ui| {
                    ui.label("Theta:");
                    ui.label(self.theta.to_string());
                });
            });
    }
}

/// A 2D frame representing a position and orientation in space for frame management and transformations.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Frame {
    transform: Isometry2<f32>,
}

impl Default for Frame {
    fn default() -> Self {
        Self {
            transform: Isometry2::identity(),
        }
    }
}

impl Frame {
    /// Create a new [`Frame`] from a given [`FrameConfig`].
    pub fn from_config(config: &FrameConfig) -> Self {
        Frame::new(config.x, config.y, config.theta)
    }

    pub fn new(x: f32, y: f32, theta: f32) -> Self {
        Self {
            transform: Isometry2::new(nalgebra::Vector2::new(x, y), theta),
        }
    }

    /// Get the transformation of the frame as an isometry.
    pub fn transform(&self) -> &Isometry2<f32> {
        &self.transform
    }

    pub fn attach_to_state(&self, state: &State) -> AttachedFrame {
        AttachedFrame::from_frame(self, state)
    }
}

impl Recordable<FrameRecord> for Frame {
    fn record(&self, _context: &Context) -> FrameRecord {
        let translation = self.transform.translation.vector;
        let rotation = self.transform.rotation.angle();
        FrameRecord {
            x: translation.x,
            y: translation.y,
            theta: rotation,
        }
    }
}

/// Record type for a [`Frame`], containing the position and orientation data.
pub type FrameRecord = FrameConfig;

/// An attached frame that is associated with a specific state. The [`Frame`] is defined relative to a node state and the [`AttachedFrame`]
/// provides the transformed state of the frame origin point and methods to transform points between the frame's coordinate system and
/// the global coordinate system.
pub struct AttachedFrame {
    frame_state: State,
    transform: Isometry2<f32>,
}

impl AttachedFrame {
    fn from_frame(frame: &Frame, state: &State) -> Self {
        let state_world_transformation =
            Isometry2::new(state.pose.fixed_rows::<2>(0).into_owned(), state.pose[2]);
        let transform = state_world_transformation * frame.transform;

        let new_position = transform.translation.vector;
        let new_theta = transform.rotation.angle();
        let new_theta = mod2pi(new_theta);

        let angular_velocity = state.velocity[2];
        let level_arms = nalgebra::Vector2::new(frame.transform.translation.y, -frame.transform.translation.x);
        let new_velocity = state.velocity.fixed_rows::<2>(0)
            - angular_velocity * level_arms;
        let new_velocity = frame
            .transform
            .rotation
            .inverse_transform_vector(&new_velocity);
        Self {
            frame_state: State {
                pose: nalgebra::Vector3::new(new_position.x, new_position.y, new_theta),
                velocity: nalgebra::Vector3::new(new_velocity.x, new_velocity.y, state.velocity[2]),
            },
            transform,
        }
    }

    /// Get the state of the frame origin point (including transformed velocity)
    pub fn state(&self) -> &State {
        &self.frame_state
    }

    /// Get the transformation of the attached frame as an isometry.
    ///
    /// WARNING: when using Isometry, use homogeneous coordinates (x, y, 1).
    pub fn transform(&self) -> &Isometry2<f32> {
        &self.transform
    }

    /// Transform a given point from the frame's coordinate system to the global coordinate system.
    pub fn transform_point_frame_to_global<'a, S>(
        &self,
        point: &'a nalgebra::Matrix<f32, Const<2>, Const<1>, S>,
    ) -> nalgebra::Matrix<f32, Const<2>, Const<1>, ArrayStorage<f32, 2, 1>>
    where
        S: Storage<f32, Const<2>, Const<1>>,
    {
        let p = nalgebra::Point2::new(point[0], point[1]);
        self.transform.transform_point(&p).coords
    }

    /// Transform a given point from the global coordinate system to the frame's coordinate system.
    pub fn transform_point_global_to_frame<S>(
        &self,
        point: &nalgebra::Matrix<f32, Const<2>, Const<1>, S>,
    ) -> nalgebra::Matrix<f32, Const<2>, Const<1>, ArrayStorage<f32, 2, 1>>
    where
        S: Storage<f32, Const<2>, Const<1>>,
    {
        let p = nalgebra::Point2::new(point[0], point[1]);
        self.transform.inverse_transform_point(&p).coords
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Rotation2;

    use super::*;

    #[test]
    fn identity() {
        let frame = Frame::from_config(&FrameConfig::default());
        let state = State {
            pose: nalgebra::Vector3::new(1.0, 2.0, PI / 4.0),
            velocity: nalgebra::Vector3::new(0.5, 0.5, 0.1),
        };
        let attached_frame = frame.attach_to_state(&state);
        assert_eq!(attached_frame.state().pose, state.pose);
        assert_eq!(attached_frame.state().velocity, state.velocity);

        let global_point = nalgebra::Vector2::new(1.0, 1.0);
        let frame_point = attached_frame.transform_point_global_to_frame(&global_point);

        let expected_frame_point =
            Rotation2::new(-PI / 4.) * (global_point - nalgebra::Vector2::new(1.0, 2.0));

        assert_eq!(frame_point, expected_frame_point);
        assert_eq!(
            attached_frame.transform_point_frame_to_global(&expected_frame_point),
            global_point
        );
    }

    #[test]
    fn transform_along_x() {
        let frame = Frame::new(1.0, 0.0, PI / 2.0);
        let state = State {
            pose: nalgebra::Vector3::new(2.0, 3.0, PI / 4.0),
            velocity: nalgebra::Vector3::new(0.5, 0.5, 0.1),
        };
        let attached_frame = frame.attach_to_state(&state);
        let expected_pose = nalgebra::Vector3::new(
            2.0 + (PI / 4.).cos(),
            3.0 + (PI / 4.).sin(),
            mod2pi(PI / 4.0 + PI / 2.0),
        );
        let expected_velocity = nalgebra::Vector3::new(0.5 + 0.1, -0.5, 0.1);
        assert!((attached_frame.state().pose - expected_pose).norm() < 1e-6, "Expected pose: {:?}, got: {:?}", expected_pose, attached_frame.state().pose);
        assert!((attached_frame.state().velocity - expected_velocity).norm() < 1e-6, "Expected velocity: {:?}, got: {:?}", expected_velocity, attached_frame.state().velocity);
    }

    #[test]
    fn transform_along_y() {
        let frame = Frame::new(0.0, 1.0, PI / 2.0);
        let state = State {
            pose: nalgebra::Vector3::new(2.0, 3.0, PI / 4.0),
            velocity: nalgebra::Vector3::new(0.5, 0.5, 0.1),
        };
        let attached_frame = frame.attach_to_state(&state);
        let expected_pose = nalgebra::Vector3::new(
            2.0 - (PI / 4.).sin(),
            3.0 + (PI / 4.).cos(),
            mod2pi(PI / 4.0 + PI / 2.0),
        );
        let expected_velocity = nalgebra::Vector3::new(0.5, -0.5 + 0.1, 0.1);
        assert!((attached_frame.state().pose - expected_pose).norm() < 1e-6, "Expected pose: {:?}, got: {:?}", expected_pose, attached_frame.state().pose);
        assert!((attached_frame.state().velocity - expected_velocity).norm() < 1e-6, "Expected velocity: {:?}, got: {:?}", expected_velocity, attached_frame.state().velocity);
    }
}
