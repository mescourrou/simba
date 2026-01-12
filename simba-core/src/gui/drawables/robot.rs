use egui::{Color32, Rect, Response, Shape, Stroke, Vec2};
use nalgebra::Vector3;

use crate::{
    gui::{UIComponent, app::PainterInfo},
    node::node_factory::{RobotConfig, RobotRecord},
    sensors::{SensorConfig, SensorObservationRecord},
    simulator::SimulatorConfig,
    utils::time_ordered_data::TimeOrderedData,
};

use super::observations::{OrientedLandmarkObservation, OrientedRobotObservation};

pub struct Robot {
    color: Color32,
    records: TimeOrderedData<RobotRecord>,
    arrow_len: f32,
    landmark_obs: Option<OrientedLandmarkObservation>,
    robot_obs: Option<OrientedRobotObservation>,
    context_info_enabled: bool,
}

impl Robot {
    pub fn init(config: &RobotConfig, sim_config: &SimulatorConfig) -> Self {
        let mut landmark_obs = None;
        let mut robot_obs = None;

        for sensor_conf in &config.sensor_manager.sensors {
            match &sensor_conf.config {
                SensorConfig::GNSSSensor(_) => {}
                SensorConfig::OdometrySensor(_) => {}
                SensorConfig::OrientedLandmarkSensor(c) => {
                    landmark_obs = Some(OrientedLandmarkObservation::init(c, sim_config))
                }
                SensorConfig::RobotSensor(c) => {
                    robot_obs = Some(OrientedRobotObservation::init(c, sim_config))
                }
                SensorConfig::External(_) => {}
            }
        }

        Self {
            color: Color32::BLUE,
            records: TimeOrderedData::new(),
            arrow_len: 0.2,
            landmark_obs,
            robot_obs,
            context_info_enabled: false,
        }
    }

    pub fn add_record(&mut self, time: f32, record: RobotRecord) {
        self.records.insert(time, record, true);
    }
    pub fn draw(
        &self,
        ui: &mut egui::Ui,
        viewport: &Rect,
        painter_info: &PainterInfo,
        scale: f32,
        time: f32,
    ) -> Result<Vec<Shape>, Vec2> {
        let mut shapes = Vec::new();
        let center = painter_info.zero(scale);

        if let Some(lobs) = &self.landmark_obs {
            shapes.extend(lobs.draw_map(ui, viewport, painter_info, scale)?);
        }

        if let Some((_, record)) = self.records.get_data_beq_time(time) {
            let pose = record.physics.pose();
            let position = Vec2::new(pose[0], pose[1]);
            if !painter_info.is_inside(&position) {
                return Err(position);
            }

            let position = position * scale;

            let position = center + position;
            let arrow_tip = position
                + Vec2 {
                    x: self.arrow_len * pose[2].cos() * scale,
                    y: self.arrow_len * pose[2].sin() * scale,
                };

            shapes.push(Shape::circle_filled(position, 0.1 * scale, self.color));
            shapes.push(Shape::line_segment(
                [position, arrow_tip],
                Stroke {
                    color: self.color,
                    width: 0.05 * scale,
                },
            ));

            for obs in &record.sensors.last_observations {
                match &obs.sensor_observation {
                    SensorObservationRecord::OrientedLandmark(o) => {
                        shapes.extend(self.landmark_obs.as_ref().unwrap().draw(
                            ui,
                            viewport,
                            painter_info,
                            scale,
                            o,
                            &Vector3::from(pose),
                        )?)
                    }
                    SensorObservationRecord::OrientedRobot(o) => {
                        shapes.extend(self.robot_obs.as_ref().unwrap().draw(
                            ui,
                            viewport,
                            painter_info,
                            scale,
                            o,
                            &Vector3::from(pose),
                        )?)
                    }
                    _ => {}
                }
            }
        }
        Ok(shapes)
    }

    pub fn react(
        &mut self,
        _ui: &mut egui::Ui,
        ctx: &egui::Context,
        response: &Response,
        painter_info: &PainterInfo,
        scale: f32,
        time: f32,
    ) {
        if let Some((t, record)) = self.records.get_data_beq_time(time) {
            let pose = record.physics.pose();
            let position = Vec2::new(pose[0], pose[1]);

            if painter_info.is_position_clicked(response.interact_pointer_pos(), scale, position) {
                self.context_info_enabled = true;
            }
            if self.context_info_enabled {
                egui::Window::new(&record.name).show(ctx, |ui| {
                    if ui.button("Close").clicked() {
                        self.context_info_enabled = false;
                    }
                    let unique_id = format!("record-robot-{}", record.name);
                    ui.label(format!("Time: {:.3} s", t));

                    egui::ScrollArea::both().show(ui, |ui| {
                        record.show(ui, ctx, &unique_id);
                    });
                });
            }
        }
    }
}
