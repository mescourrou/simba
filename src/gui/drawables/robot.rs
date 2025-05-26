use egui::{Color32, Painter, Pos2, Rect, Response, Stroke, Vec2};
use nalgebra::Vector3;

use crate::{node_factory::{RobotConfig, RobotRecord}, physics::physic::PhysicConfig, sensors::sensor::{SensorConfig, SensorObservation, SensorObservationRecord}, simulator::SimulatorConfig, utils::time_ordered_data::TimeOrderedData};

use super::observations::OrientedLandmarkObservation;

pub struct Robot {
    color: Color32,
    records: TimeOrderedData<RobotRecord>,
    arrow_len: f32,
    landmark_obs: Option<OrientedLandmarkObservation>,
}

impl Robot {
    pub fn init(config: &RobotConfig, sim_config: &SimulatorConfig) -> Self {
        let mut landmark_obs = None;

        for sensor_conf in &config.sensor_manager.sensors {
            match &sensor_conf.config {
                SensorConfig::GNSSSensor(_) => {},
                SensorConfig::OdometrySensor(_) => {},
                SensorConfig::OrientedLandmarkSensor(c) => landmark_obs = Some(OrientedLandmarkObservation::init(c, sim_config)),
                SensorConfig::RobotSensor(_) => {},
            }
        }


        Self {
            color: Color32::BLUE,
            records: TimeOrderedData::new(),
            arrow_len: 0.2,
            landmark_obs
        }

    }

    pub fn add_record(&mut self, time: f32, record: RobotRecord) {
        self.records.insert(time, record, true);
    }
    pub fn draw(&self, ui: &mut egui::Ui, viewport: &Rect, response: &Response, painter: &Painter, scale: f32, time: f32) {
        let center = response.rect.center();

        if let Some(lobs) = &self.landmark_obs {
            lobs.draw_map(ui, viewport, response, painter, scale);
        }

        if let Some((_, record)) = self.records.get_data_beq_time(time) {
            let pose = record.physic.pose();
            let position = Vec2::new(pose[0], pose[1])* scale;
        
            let position = center + position;
            let arrow_tip = position + Vec2 {
                x: self.arrow_len * pose[2].cos() * scale,
                y: self.arrow_len * pose[2].sin() * scale,
            };

            painter.circle_filled(position, 0.1*scale, self.color);
            painter.line_segment([position, arrow_tip], Stroke{ color: self.color, width: 0.05*scale});

            for obs in &record.sensors.last_observations {
                match &obs.sensor_observation {
                    SensorObservationRecord::OrientedLandmark(o) => self.landmark_obs.as_ref().unwrap().draw(ui, viewport, response, painter, scale, o, &Vector3::from(pose)),
                    _ => {}
                }
            }

        }
    }
}
