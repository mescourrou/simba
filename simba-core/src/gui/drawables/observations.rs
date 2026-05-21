use egui::{Color32, Rect, Shape, Stroke, Vec2};
use nalgebra::{Const, OPoint, Rotation2, Vector2, Vector3};

use crate::{
    gui::app::PainterInfo,
    sensors::{
        gnss_sensor::{GNSSObservationRecord, GNSSSensorConfig},
        oriented_landmark_sensor::{
            OrientedLandmarkObservationRecord, OrientedLandmarkSensorConfig,
        },
        robot_sensor::{OrientedRobotObservationRecord, RobotSensorConfig},
        scan_sensor::{ScanObservationRecord, ScanSensorConfig},
    },
    simulator::SimulatorConfig,
    state_estimators::State,
    utils::frame::{Frame, FrameConfig},
};

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

        let frame = obs.frame;
        let frame = Frame::new(frame.x, frame.y, frame.theta);
        let robot_state = State {
            pose: *robot_pose,
            ..Default::default()
        };
        let attached_frame = frame.attach_to_state(&robot_state);
        let robot_pose = attached_frame.state().pose;
        let robot_position = Vector2::new(robot_pose.x, robot_pose.y);

        let robot_position = Vec2::new(robot_position.x, robot_position.y);
        if !painter_info.is_inside(&robot_position) {
            return Err(robot_position);
        }
        let robot_position = center + robot_position * scale;

        let obs_pose =
            attached_frame.transform_point_frame_to_global(&Vector2::new(obs.pose[0], obs.pose[1]));
        let obs_position = Vec2::new(obs_pose.x, obs_pose.y);
        let obs_angle = obs.pose[2];
        if !painter_info.is_inside(&obs_position) {
            return Err(obs_position);
        }
        let arrow_tip = obs_position
            + Vec2 {
                x: self.arrow_len * (obs_angle + robot_pose[2]).cos(),
                y: self.arrow_len * (obs_angle + robot_pose[2]).sin(),
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

        let frame = obs.frame;
        let frame = Frame::new(frame.x, frame.y, frame.theta);
        let robot_state = State {
            pose: *robot_pose,
            ..Default::default()
        };
        let attached_frame = frame.attach_to_state(&robot_state);
        let robot_pose = attached_frame.state().pose;
        let robot_position = Vector2::new(robot_pose.x, robot_pose.y);
        let robot_angle = robot_pose[2];

        let obs_pose =
            attached_frame.transform_point_frame_to_global(&Vector2::new(obs.pose[0], obs.pose[1]));
        // rot_matrix * Vector2::new(obs.pose[0], obs.pose[1]) + robot_position;

        let robot_position = Vec2::new(robot_position.x, robot_position.y);
        if !painter_info.is_inside(&robot_position) {
            return Err(robot_position);
        }
        let robot_position = center + robot_position * scale;

        let obs_position = Vec2::new(obs_pose.x, obs_pose.y);
        let obs_angle = obs.pose[2];
        if !painter_info.is_inside(&obs_position) {
            return Err(obs_position);
        }
        let arrow_tip = obs_position
            + Vec2 {
                x: self.arrow_len * (obs_angle + robot_angle).cos(),
                y: self.arrow_len * (obs_angle + robot_angle).sin(),
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
                half_width * (obs_angle + robot_angle + std::f32::consts::FRAC_PI_2).cos(),
                half_width * (obs_angle + robot_angle + std::f32::consts::FRAC_PI_2).sin(),
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
        let arrow_angle = obs.velocity[1].atan2(obs.velocity[0]);
        let arrow_length = (obs.velocity[0].powi(2) + obs.velocity[1].powi(2)).sqrt() * 0.2;
        let arrow_wing_left = arrow_tip
            - Vec2 {
                x: (arrow_angle + std::f32::consts::FRAC_PI_4).cos() * arrow_length,
                y: (arrow_angle + std::f32::consts::FRAC_PI_4).sin() * arrow_length,
            };
        let arrow_wing_right = arrow_tip
            - Vec2 {
                x: (arrow_angle - std::f32::consts::FRAC_PI_4).cos() * arrow_length,
                y: (arrow_angle - std::f32::consts::FRAC_PI_4).sin() * arrow_length,
            };

        let obs_position = center + obs_position * scale;
        let arrow_tip = center + arrow_tip * scale;
        let arrow_wing_left = center + arrow_wing_left * scale;
        let arrow_wing_right = center + arrow_wing_right * scale;

        shapes.push(Shape::line_segment(
            [obs_position, arrow_tip],
            Stroke {
                color: self.color,
                width: 0.01 * scale,
            },
        ));
        shapes.push(Shape::line_segment(
            [arrow_tip, arrow_wing_left],
            Stroke {
                color: self.color,
                width: 0.01 * scale,
            },
        ));
        shapes.push(Shape::line_segment(
            [arrow_tip, arrow_wing_right],
            Stroke {
                color: self.color,
                width: 0.01 * scale,
            },
        ));
        Ok(shapes)
    }
}

pub struct ScanObservation {
    color: Color32,
}

impl ScanObservation {
    pub fn init(_config: &ScanSensorConfig, _sim_config: &SimulatorConfig) -> Self {
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
        obs: &ScanObservationRecord,
        robot_pose: &Vector3<f32>,
    ) -> Result<Vec<Shape>, Vec2> {
        let mut shapes = Vec::new();
        let center = painter_info.zero(scale);

        let frame = obs.frame;
        let frame = Frame::new(frame.x, frame.y, frame.theta);
        let robot_state = State {
            pose: *robot_pose,
            ..Default::default()
        };
        let attached_frame = frame.attach_to_state(&robot_state);
        for (d, angle) in obs.distances.iter().zip(obs.angles.iter()) {
            let obs_position = Vector2::new(d * angle.cos(), d * angle.sin());
            let obs_position = attached_frame.transform_point_frame_to_global(&obs_position);
            let obs_position = Vec2::new(obs_position.x, obs_position.y);
            if !painter_info.is_inside(&obs_position) {
                return Err(obs_position);
            }
            let obs_position = center + obs_position * scale;
            shapes.push(Shape::circle_filled(obs_position, 0.05 * scale, self.color));
        }
        Ok(shapes)
    }
}
