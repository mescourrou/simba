use std::str::FromStr;

use simba_com::pub_sub::{BrokerTrait, PathKey};

use crate::{simulator::SimbaBroker, utils::SharedRoLock};

pub struct BrokerPanel {
    broker: SharedRoLock<SimbaBroker>,
}

impl BrokerPanel {
    pub fn new(broker: SharedRoLock<SimbaBroker>) -> Self {
        Self { broker }
    }

    fn draw_subtree(ui: &mut egui::Ui, nodes: &Vec<(PathKey, PathKey)>, current_node: PathKey) {
        let mut children = Vec::new();
        for (path, parent) in nodes {
            if path == parent {
                continue; // Skip the root node
            }
            if parent == &current_node {
                children.push(path.clone());
            }
        }
        let current_node_str = current_node
            .to_vec()
            .last()
            .unwrap_or(&"/".to_string())
            .clone();
        if children.is_empty() {
            ui.label(current_node_str);
            return;
        }
        egui::CollapsingHeader::new(current_node_str).show(ui, |ui| {
            for child in children {
                Self::draw_subtree(ui, nodes, child);
            }
        });
    }

    pub fn draw(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str, _time: f32) {
        egui::CollapsingHeader::new("Channels").show(ui, |ui| {
            let tree = self.broker.read().unwrap().meta_tree();
            Self::draw_subtree(ui, &tree, PathKey::from_str("/").unwrap());
        });
    }
}
