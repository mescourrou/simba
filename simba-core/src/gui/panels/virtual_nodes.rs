use std::collections::BTreeMap;

use simba_com::time_ordered_data::TimeOrderedData;

use crate::{gui::UIComponent, node::node_factory::NodeRecord};

pub struct VirtualNodesPanel {
    records: BTreeMap<String, TimeOrderedData<NodeRecord>>,
}

impl VirtualNodesPanel {
    pub fn new() -> Self {
        Self {
            records: BTreeMap::new(),
        }
    }

    pub fn add_record(&mut self, node_name: String, time: f32, record: NodeRecord) {
        if let Some(records) = self.records.get_mut(&node_name) {
            records.insert(time, record, true);
        } else {
            let mut records = TimeOrderedData::new(0.01);
            records.insert(time, record, true);
            self.records.insert(node_name, records);
        }
    }

    pub fn draw(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str, time: f32) {
        egui::CollapsingHeader::new("Virtual Nodes").show(ui, |ui| {
            for (node_name, records) in &self.records {
                if let Some((_t, record)) = records.get_data_beq_time(time) {
                    egui::CollapsingHeader::new(node_name).show(ui, |ui| {
                        record.show(ui, ctx, unique_id);
                    });
                }
            }
        });
    }
}
