use std::{path::Path, sync::Mutex};

use egui::{Color32, Rect, Shape, Stroke, Vec2};

use crate::{
    gui::app::PainterInfo,
    sensors::oriented_landmark_sensor::{OrientedLandmark, OrientedLandmarkSensor},
    simulator::SimulatorConfig,
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
