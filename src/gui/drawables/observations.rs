use egui::{Color32, Painter, Rect, Response, Shape, Stroke, Vec2};
use nalgebra::{Rotation2, Vector2, Vector3};

use crate::{
    gui::app::PainterInfo,
    sensors::{
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
    map: Map,
    arrow_len: f32,
}

impl OrientedLandmarkObservation {
    pub fn init(config: &OrientedLandmarkSensorConfig, sim_config: &SimulatorConfig) -> Self {
        Self {
            color: Color32::from_rgb(255, 165, 0), // Orange
            map: Map::init(&config.map_path, sim_config),
            arrow_len: 0.2,
        }
    }

    pub fn draw_map(
        &self,
        ui: &mut egui::Ui,
        viewport: &Rect,
        painter_info: &PainterInfo,
        scale: f32,
    ) -> Result<Vec<Shape>, Vec2> {
        self.map.draw(ui, viewport, painter_info, scale)
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
        Ok(shapes)
    }
}
