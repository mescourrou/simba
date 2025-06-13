use egui::{Color32, Painter, Pos2, Rect, Response, Stroke, Vec2};
use nalgebra::{Rotation2, Rotation3, Vector2, Vector3};

use crate::{
    node_factory::{RobotConfig, RobotRecord},
    physics::physic::PhysicConfig,
    sensors::{
        oriented_landmark_sensor::{
            self, OrientedLandmarkObservationRecord, OrientedLandmarkSensorConfig,
        },
        robot_sensor::{OrientedRobotObservationRecord, RobotSensorConfig},
    },
    simulator::SimulatorConfig,
    utils::time_ordered_data::TimeOrderedData,
};

use super::map::Map;

pub struct OrientedRobotObservation {
    color: Color32,
    arrow_len: f32,
}

impl OrientedRobotObservation {
    pub fn init(config: &RobotSensorConfig, sim_config: &SimulatorConfig) -> Self {
        Self {
            color: Color32::from_rgb(255, 165, 0), // Orange
            arrow_len: 0.2,
        }
    }

    pub fn draw(
        &self,
        ui: &mut egui::Ui,
        viewport: &Rect,
        response: &Response,
        painter: &Painter,
        scale: f32,
        obs: &OrientedRobotObservationRecord,
        robot_pose: &Vector3<f32>,
    ) {
        let center = response.rect.center();

        let rot_matrix = Rotation2::new(robot_pose[2]);
        let obs_pose =
            rot_matrix * Vector2::new(obs.pose[0], obs.pose[1]) + robot_pose.fixed_rows::<2>(0);

        let robot_position = center + Vec2::new(robot_pose.x, robot_pose.y) * scale;
        let obs_position = center + Vec2::new(obs_pose.x, obs_pose.y) * scale;
        let arrow_tip = obs_position
            + Vec2 {
                x: self.arrow_len * (obs.pose[2] + robot_pose[2]).cos() * scale,
                y: self.arrow_len * (obs.pose[2] + robot_pose[2]).sin() * scale,
            };

        painter.line_segment(
            [robot_position, obs_position],
            Stroke {
                color: self.color,
                width: 0.01 * scale,
            },
        );
        painter.line_segment(
            [obs_position, arrow_tip],
            Stroke {
                color: self.color,
                width: 0.01 * scale,
            },
        );
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
        response: &Response,
        painter: &Painter,
        scale: f32,
    ) {
        self.map.draw(ui, viewport, response, painter, scale);
    }

    pub fn draw(
        &self,
        ui: &mut egui::Ui,
        viewport: &Rect,
        response: &Response,
        painter: &Painter,
        scale: f32,
        obs: &OrientedLandmarkObservationRecord,
        robot_pose: &Vector3<f32>,
    ) {
        let center = response.rect.center();

        let rot_matrix = Rotation2::new(robot_pose[2]);
        let obs_pose =
            rot_matrix * Vector2::new(obs.pose[0], obs.pose[1]) + robot_pose.fixed_rows::<2>(0);

        let robot_position = center + Vec2::new(robot_pose.x, robot_pose.y) * scale;
        let obs_position = center + Vec2::new(obs_pose.x, obs_pose.y) * scale;
        let arrow_tip = obs_position
            + Vec2 {
                x: self.arrow_len * (obs.pose[2] + robot_pose[2]).cos() * scale,
                y: self.arrow_len * (obs.pose[2] + robot_pose[2]).sin() * scale,
            };

        painter.line_segment(
            [robot_position, obs_position],
            Stroke {
                color: self.color,
                width: 0.01 * scale,
            },
        );
        painter.line_segment(
            [obs_position, arrow_tip],
            Stroke {
                color: self.color,
                width: 0.01 * scale,
            },
        );
    }
}
