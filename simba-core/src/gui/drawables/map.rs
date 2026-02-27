use std::{path::Path, sync::Mutex};

use egui::{Color32, Rect, Shape, Stroke, Vec2};

use crate::{
    environment::{self, EnvironmentConfig, oriented_landmark::OrientedLandmark},
    gui::app::PainterInfo,
    sensors::oriented_landmark_sensor::OrientedLandmarkSensor,
    simulator::SimulatorConfig,
};

static LOADED_MAPS: Mutex<Vec<(String, Vec<OrientedLandmark>)>> = Mutex::new(Vec::new());

pub struct Map {
    color: Color32,
    landmarks: Vec<OrientedLandmark>,
    arrow_len: f32,
}

impl Default for Map {
    fn default() -> Self {
        Self {
            color: Color32::RED,
            landmarks: Vec::new(),
            arrow_len: 0.2,
        }
    }
}

impl Map {
    pub fn init(environment_config: &EnvironmentConfig, sim_config: &SimulatorConfig) -> Self {
        let path = &environment_config.map_path;
        let landmarks = if let Some(path) = path {
            environment::Map::load_from_path(&sim_config.base_path.join(path))
                .expect("Failed to load map")
                .landmarks
        } else {
            Vec::new()
        };
        log::info!("Loaded map with {} landmarks", landmarks.len());
        Self {
            color: Color32::RED,
            landmarks,
            arrow_len: 0.2,
        }
    }

    pub fn draw(
        &self,
        _ui: &mut egui::Ui,
        _viewport: &Rect,
        painter_info: &PainterInfo,
        scale: f32,
    ) -> Result<Vec<Shape>, Vec2> {
        let mut shapes = Vec::new();
        let center = painter_info.zero(scale);
        for landmark in &self.landmarks {
            let position = Vec2::new(landmark.pose[0], landmark.pose[1]);
            if !painter_info.is_inside(&position) {
                return Err(position);
            }
            let arrow_tip = position
                + Vec2 {
                    x: self.arrow_len * landmark.pose[2].cos(),
                    y: self.arrow_len * landmark.pose[2].sin(),
                };
            if !painter_info.is_inside(&arrow_tip) {
                return Err(arrow_tip);
            }
            let position = center + position * scale;
            let arrow_tip = center + arrow_tip * scale;

            shapes.push(Shape::rect_filled(
                Rect::from_center_size(
                    position,
                    Vec2 {
                        x: 0.05 * scale,
                        y: 0.05 * scale,
                    },
                ),
                0.01 * scale,
                self.color,
            ));
            shapes.push(Shape::line_segment(
                [position, arrow_tip],
                Stroke {
                    color: self.color,
                    width: 0.05 * scale,
                },
            ));
            if landmark.width > 0.0 {
                let half_width = landmark.width / 2.0;
                let dir_vector = Vec2::new(
                    half_width * (landmark.pose[2] + std::f32::consts::FRAC_PI_2).cos(),
                    half_width * (landmark.pose[2] + std::f32::consts::FRAC_PI_2).sin(),
                );
                let p1 = position + dir_vector * scale;
                let p2 = position - dir_vector * scale;
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
        }
        Ok(shapes)
    }
}
