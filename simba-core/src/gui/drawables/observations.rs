use egui::{Color32, Rect, Shape, Stroke, Vec2};
use nalgebra::{Rotation2, Vector2, Vector3};

use crate::{
    gui::app::PainterInfo,
    sensors::{
        gnss_sensor::{GNSSObservationRecord, GNSSSensorConfig},
        oriented_landmark_sensor::{
            OrientedLandmarkObservationRecord, OrientedLandmarkSensorConfig,
        },
        robot_sensor::{OrientedRobotObservationRecord, RobotSensorConfig},
    },
    simulator::SimulatorConfig,
};

use super::map::Map;

pub struct OrientedRobotObservation {
    color: Color32,
    arrow_len: f32,
}

impl OrientedRobotObservation {
    pub fn init(_config: &RobotSensorConfig, _sim_config: &SimulatorConfig) -> Self {
        Self {
            color: Color32::from_rgb(255, 165, 0), // Orange
            arrow_len: 0.2,
        }
    }

    pub fn draw(
        &self,
        _ui: &mut egui::Ui,
        _viewport: &Rect,
        painter_info: &PainterInfo,
        scale: f32,
        obs: &OrientedRobotObservationRecord,
        robot_pose: &Vector3<f32>,
    ) -> Result<Vec<Shape>, Vec2> {
        let mut shapes = Vec::new();
        let center = painter_info.zero(scale);

        let rot_matrix = Rotation2::new(robot_pose[2]);
        let obs_pose =
            rot_matrix * Vector2::new(obs.pose[0], obs.pose[1]) + robot_pose.fixed_rows::<2>(0);

        let robot_position = center + Vec2::new(robot_pose.x, robot_pose.y) * scale;
        let obs_position = Vec2::new(obs_pose.x, obs_pose.y);
        if !painter_info.is_inside(&obs_position) {
            return Err(obs_position);
        }
        let arrow_tip = obs_position
            + Vec2 {
                x: self.arrow_len * (obs.pose[2] + robot_pose[2]).cos(),
                y: self.arrow_len * (obs.pose[2] + robot_pose[2]).sin(),
            };
        if !painter_info.is_inside(&arrow_tip) {
            return Err(arrow_tip);
        }
        let obs_position = center + obs_position * scale;
        let arrow_tip = center + arrow_tip * scale;

        shapes.push(Shape::line_segment(
            [robot_position, obs_position],
            Stroke {
                color: self.color,
                width: 0.01 * scale,
            },
        ));
        shapes.push(Shape::line_segment(
            [obs_position, arrow_tip],
            Stroke {
                color: self.color,
                width: 0.01 * scale,
            },
        ));
        Ok(shapes)
    }
}

pub struct OrientedLandmarkObservation {
    color: Color32,
    arrow_len: f32,
}

impl OrientedLandmarkObservation {
    pub fn init(_config: &OrientedLandmarkSensorConfig, _sim_config: &SimulatorConfig) -> Self {
        Self {
            color: Color32::from_rgb(255, 165, 0), // Orange
            arrow_len: 0.2,
        }
    }

    pub fn draw(
        &self,
        _ui: &mut egui::Ui,
        _viewport: &Rect,
        painter_info: &PainterInfo,
        scale: f32,
        obs: &OrientedLandmarkObservationRecord,
        robot_pose: &Vector3<f32>,
    ) -> Result<Vec<Shape>, Vec2> {
        let mut shapes = Vec::new();
        let center = painter_info.zero(scale);

        let rot_matrix = Rotation2::new(robot_pose[2]);
        let obs_pose =
            rot_matrix * Vector2::new(obs.pose[0], obs.pose[1]) + robot_pose.fixed_rows::<2>(0);

        let robot_position = center + Vec2::new(robot_pose.x, robot_pose.y) * scale; // The robot should be inside the painter area (managed by the robot draw)

        let obs_position = Vec2::new(obs_pose.x, obs_pose.y);
        if !painter_info.is_inside(&obs_position) {
            return Err(obs_position);
        }
        let arrow_tip = obs_position
            + Vec2 {
                x: self.arrow_len * (obs.pose[2] + robot_pose[2]).cos(),
                y: self.arrow_len * (obs.pose[2] + robot_pose[2]).sin(),
            };
        if !painter_info.is_inside(&arrow_tip) {
            return Err(arrow_tip);
        }
        let obs_position = center + obs_position * scale;
        let arrow_tip = center + arrow_tip * scale;

        shapes.push(Shape::line_segment(
            [robot_position, obs_position],
            Stroke {
                color: self.color,
                width: 0.01 * scale,
            },
        ));
        shapes.push(Shape::line_segment(
            [obs_position, arrow_tip],
            Stroke {
                color: self.color,
                width: 0.01 * scale,
            },
        ));
        if obs.width > 0.0 {
            let half_width = obs.width / 2.0;
            let dir_vector = Vec2::new(
                half_width * (obs.pose[2] + robot_pose[2] + std::f32::consts::FRAC_PI_2).cos(),
                half_width * (obs.pose[2] + robot_pose[2] + std::f32::consts::FRAC_PI_2).sin(),
            );
            let p1 = obs_position + dir_vector * scale;
            let p2 = obs_position - dir_vector * scale;
            shapes.push(Shape::line_segment(
                [p1, p2],
                Stroke {
                    color: self.color,
                    width: 0.01 * scale,
                },
            ));
            shapes.push(Shape::circle_filled(p1, 0.05 * scale, self.color));
            shapes.push(Shape::circle_filled(p2, 0.05 * scale, self.color));
        }
        Ok(shapes)
    }
}

pub struct GNSSObservation {
    color: Color32,
}

impl GNSSObservation {
    pub fn init(_config: &GNSSSensorConfig, _sim_config: &SimulatorConfig) -> Self {
        Self {
            color: Color32::from_rgb(255, 165, 0), // Orange
        }
    }

    pub fn draw(
        &self,
        _ui: &mut egui::Ui,
        _viewport: &Rect,
        painter_info: &PainterInfo,
        scale: f32,
        obs: &GNSSObservationRecord,
    ) -> Result<Vec<Shape>, Vec2> {
        let mut shapes = Vec::new();
        let center = painter_info.zero(scale);

        let obs_position = Vec2::new(obs.pose[0], obs.pose[1]);
        if !painter_info.is_inside(&obs_position) {
            return Err(obs_position);
        }
        let arrow_tip = obs_position
            + Vec2 {
                x: obs.velocity[0],
                y: obs.velocity[1],
            };
        if !painter_info.is_inside(&arrow_tip) {
            return Err(arrow_tip);
        }
        let obs_position = center + obs_position * scale;
        let arrow_tip = center + arrow_tip * scale;

        shapes.push(Shape::line_segment(
            [obs_position, arrow_tip],
            Stroke {
                color: self.color,
                width: 0.01 * scale,
            },
        ));
        Ok(shapes)
    }
}
