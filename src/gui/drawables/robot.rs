use egui::{Color32, Painter, Pos2, Rect, Response, Vec2};

use crate::{node_factory::{RobotConfig, RobotRecord}, physics::physic::PhysicConfig, utils::time_ordered_data::TimeOrderedData};

pub struct Robot {
    color: Color32,
    records: TimeOrderedData<RobotRecord>,
}

impl Robot {
    pub fn init(config: &RobotConfig) -> Self {
        Self { color: Color32::BLUE, records: TimeOrderedData::new() }

    }

    pub fn add_record(&mut self, time: f32, record: RobotRecord) {
        self.records.insert(time, record, true);
    }
    pub fn draw(&self, ui: &mut egui::Ui, viewport: &Rect, response: &Response, painter: &Painter, scale: f32, time: f32) {
        let center = response.rect.center();
        if let Some((_, record)) = self.records.get_data_beq_time(time) {
            let pose = record.physic.pose();
            let position = Vec2::new(pose[0], pose[1])* scale;
        
            painter.circle_filled(center + position , 0.1*scale, self.color);
        }
    }
}
