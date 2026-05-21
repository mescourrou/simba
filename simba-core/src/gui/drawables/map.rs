use std::sync::Arc;

use egui::{Color32, Rect, Shape, Stroke, Vec2};

use crate::{
    context::Context,
    environment::{self, Environment, EnvironmentConfig, map::MapRecord, oriented_landmark::OrientedLandmark},
    gui::app::PainterInfo,
    info,
    simulator::SimulatorConfig,
};

pub struct Map {
    color: Color32,
    current_map: MapRecord,
    arrow_len: f32,
}

impl Default for Map {
    fn default() -> Self {
        Self {
            color: Color32::RED,
            current_map: MapRecord {
                landmarks: Vec::new(),
            },
            arrow_len: 0.2,
        }
    }
}

impl Map {
    /// Initialize the map drawable from environment and simulator configuration.
    ///
    /// ## Arguments
    /// * `sim_config` - Simulator configuration used to resolve the map path.
    /// * `context` - Shared simulation context used for initialization logs.
    pub fn init(
        current_map: MapRecord,
        _sim_config: &SimulatorConfig,
        _context: &Context,
    ) -> Self {
        Self {
            color: Color32::RED,
            arrow_len: 0.2,
            current_map,
        }
    }

    pub fn update_map(&mut self, new_map: MapRecord) {
        self.current_map = new_map;
    }

    /// Build the set of map shapes for rendering in the current viewport.
    pub fn draw(
        &self,
        _ui: &mut egui::Ui,
        _viewport: &Rect,
        painter_info: &PainterInfo,
        scale: f32,
    ) -> Result<Vec<Shape>, Vec2> {
        let mut shapes = Vec::new();
        let center = painter_info.zero(scale);
        for landmark in self.current_map.landmarks.iter() {
            let position = Vec2::new(landmark.x, landmark.y);
            if !painter_info.is_inside(&position) {
                return Err(position);
            }
            let arrow_tip = position
                + Vec2 {
                    x: self.arrow_len * landmark.theta.cos(),
                    y: self.arrow_len * landmark.theta.sin(),
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
                    half_width * (landmark.theta + std::f32::consts::FRAC_PI_2).cos(),
                    half_width * (landmark.theta + std::f32::consts::FRAC_PI_2).sin(),
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
