use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::{
    sensors::{SensorObservation, sensor_filters::SensorFilter},
    state_estimators::State,
};

#[config_derives]
pub struct IdFilterConfig {
    /// Identifiers (id for landmark, name for robots) to accept. Can be regexp patterns.
    pub accepted: Vec<String>,
    /// Identifiers (id for landmark, name for robots) to reject. Can be regexp patterns.
    pub rejected: Vec<String>,
    /// How to manage intersection between accepted and rejected sets.
    /// If true, first accept then reject. If false, first reject then accept.
    pub accept_then_reject: bool,
}

impl Default for IdFilterConfig {
    fn default() -> Self {
        Self {
            accepted: Vec::new(),
            rejected: Vec::new(),
            accept_then_reject: true,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for IdFilterConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &crate::simulator::SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        ui.vertical(|ui| {
            ui.label("Accepted identifiers (regexp):");
            let mut to_remove = Vec::new();
            for (i, id) in self.accepted.iter_mut().enumerate() {
                ui.horizontal(|ui| {
                    use crate::gui::utils::text_singleline_with_apply;

                    let unique_var_id = format!("accepted-{i}-{unique_id}");
                    text_singleline_with_apply(ui, &unique_var_id, buffer_stack, id);
                    if ui.button("-").clicked() {
                        to_remove.push(i);
                    }
                });
            }
            for i in to_remove.iter().rev() {
                self.accepted.remove(*i);
            }
            if ui.button("+").clicked() {
                self.accepted.push(String::new());
            }

            ui.label("Rejected identifiers (regexp):");
            let mut to_remove = Vec::new();
            for (i, id) in self.rejected.iter_mut().enumerate() {
                ui.horizontal(|ui| {
                    use crate::gui::utils::text_singleline_with_apply;

                    let unique_var_id = format!("rejected-{i}-{unique_id}");
                    ui.label("- ");
                    text_singleline_with_apply(ui, &unique_var_id, buffer_stack, id);
                    if ui.button("-").clicked() {
                        to_remove.push(i);
                    }
                });
            }
            for i in to_remove.iter().rev() {
                self.rejected.remove(*i);
            }
            if ui.button("+").clicked() {
                self.rejected.push(String::new());
            }

            ui.horizontal(|ui| {
                ui.label("Accept first:");
                ui.checkbox(&mut self.accept_then_reject, "");
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label("Accepted identifiers (regexp):");
            for (i, var) in self.accepted.iter().enumerate() {
                ui.label(format!("- '{}'", var));
            }
            ui.label("Rejected identifiers (regexp):");
            for (i, var) in self.rejected.iter().enumerate() {
                ui.label(format!("- '{}'", var));
            }
            ui.label(format!("Accept first: {}", self.accept_then_reject));
        });
    }
}

#[derive(Debug, Clone)]
pub struct IdFilter {
    accepted: Vec<regex::Regex>,
    rejected: Vec<regex::Regex>,
    accept_then_reject: bool,
}

impl IdFilter {
    pub fn from_config(config: &IdFilterConfig, _initial_time: f32) -> Self {
        Self {
            accepted: config
                .accepted
                .iter()
                .map(|s| regex::Regex::new(s).unwrap())
                .collect(),
            rejected: config
                .rejected
                .iter()
                .map(|s| regex::Regex::new(s).unwrap())
                .collect(),
            accept_then_reject: config.accept_then_reject,
        }
    }

    fn keep_label(&self, label: &str) -> bool {
        if self.accept_then_reject {
            if !self.accepted.is_empty() && self
                .accepted
                .iter()
                .any(|re| re.is_match(label))
            {
                return true;
            } else if self
                .rejected
                .iter()
                .any(|re| re.is_match(label))
            {
                return false;
            }
        } else {
            if self
                .rejected
                .iter()
                .any(|re| re.is_match(label))
            {
                return false;
            } else if !self.accepted.is_empty() && self
                .accepted
                .iter()
                .any(|re| re.is_match(label))
            {
                return true;
            }
        }
        self.accepted.is_empty()
    }
}

impl SensorFilter for IdFilter {
    fn filter(
        &self,
        _time: f32,
        observation: SensorObservation,
        _observer_state: &State,
        _observee_state: Option<&State>,
    ) -> Option<SensorObservation> {
        match &observation {
            SensorObservation::GNSS(_) => {
                unimplemented!("IdFilter cannot filter GNSSObservation");
            }
            SensorObservation::Odometry(_) => {
                unimplemented!("IdFilter cannot filter OdometryObservation");
            }
            SensorObservation::OrientedLandmark(obs) => {
                let label = &obs.id.to_string();
                if !self.keep_label(label) {
                    return None;
                }
            }
            SensorObservation::OrientedRobot(obs) => {
                let label = &obs.name;
                if !self.keep_label(label) {
                    return None;
                }
            }
            SensorObservation::External(_) => {
                panic!("IdFilter cannot filter ExternalObservation");
            }
        }

        Some(observation)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accept_first() {
        let config = IdFilterConfig {   
            accepted: vec!["robot_good".to_string()],
            rejected: vec!["robot_.*".to_string()],
            accept_then_reject: true,
        };
        let filter = IdFilter::from_config(&config, 0.0);
        assert!(!filter.keep_label("robot_1"));
        assert!(filter.keep_label("robot_good"));
        assert!(!filter.keep_label("landmark_1"));
    }

    #[test]
    fn reject_first() {
        let config = IdFilterConfig {   
            accepted: vec!["robot_.*".to_string()],
            rejected: vec!["robot_bad".to_string()],
            accept_then_reject: false,
        };
        let filter = IdFilter::from_config(&config, 0.0);
        assert!(filter.keep_label("robot_1"));
        assert!(!filter.keep_label("robot_bad"));
        assert!(!filter.keep_label("landmark_1"));
    }

    #[test]
    fn accept_empty() {
        let config = IdFilterConfig {   
            accepted: vec![],
            rejected: vec!["robot_.*".to_string()],
            accept_then_reject: true,
        };
        let filter = IdFilter::from_config(&config, 0.0);
        assert!(!filter.keep_label("robot_1"));
        assert!(!filter.keep_label("robot_bad"));
        assert!(filter.keep_label("landmark_1"));
    }
}