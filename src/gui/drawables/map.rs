use std::{
    collections::HashMap,
    path::Path,
    sync::{Mutex, RwLock},
};

use egui::{Color32, Painter, Pos2, Rect, Response, Stroke, Vec2};

use crate::{
    node_factory::{RobotConfig, RobotRecord},
    physics::physic::PhysicConfig,
    sensors::oriented_landmark_sensor::{OrientedLandmark, OrientedLandmarkSensor},
    simulator::SimulatorConfig,
    utils::time_ordered_data::TimeOrderedData,
};

static LOADED_MAPS: Mutex<Vec<(String, Vec<OrientedLandmark>)>> = Mutex::new(Vec::new());

pub struct Map {
    color: Color32,
    landmarks: Vec<OrientedLandmark>,
    arrow_len: f32,
}

impl Map {
    pub fn init(path: &String, sim_config: &SimulatorConfig) -> Self {
        let mut loaded_maps = LOADED_MAPS.lock().unwrap();
        for (p, v) in loaded_maps.iter() {
            if p == path {
                return Self {
                    color: Color32::RED,
                    landmarks: v.clone(),
                    arrow_len: 0.2,
                };
            }
        }

        let landmarks =
            OrientedLandmarkSensor::load_map_from_path(&sim_config.base_path.join(Path::new(path)));
        loaded_maps.push((path.clone(), landmarks.clone()));
        Self {
            color: Color32::RED,
            landmarks,
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
    ) {
        let center = response.rect.center();
        for landmark in &self.landmarks {
            let position = Vec2::new(landmark.pose[0], landmark.pose[1]) * scale;
            let position = center + position;
            let arrow_tip = position
                + Vec2 {
                    x: self.arrow_len * landmark.pose[2].cos() * scale,
                    y: self.arrow_len * landmark.pose[2].sin() * scale,
                };

            painter.rect_filled(
                Rect::from_center_size(
                    position,
                    Vec2 {
                        x: 0.05 * scale,
                        y: 0.05 * scale,
                    },
                ),
                0.01 * scale,
                self.color,
            );
            painter.line_segment(
                [position, arrow_tip],
                Stroke {
                    color: self.color,
                    width: 0.05 * scale,
                },
            );
        }
    }
}
