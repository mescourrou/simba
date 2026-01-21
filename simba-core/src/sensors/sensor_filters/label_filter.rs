use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::{
    sensors::{SensorObservation, sensor_filters::SensorFilter},
    state_estimators::State,
};

#[config_derives]
pub struct LabelFilterConfig {
    /// Labels to accept. Can be regexp patterns.
    pub accepted: Vec<String>,
    /// Labels to reject. Can be regexp patterns.
    pub rejected: Vec<String>,
    /// How to manage intersection between accepted and rejected sets.
    /// If true, when a label is in the accepted and rejected sets, it is accepted.
    pub priority_accept: bool,
}

impl Default for LabelFilterConfig {
    fn default() -> Self {
        Self {
            accepted: Vec::new(),
            rejected: Vec::new(),
            priority_accept: true,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for LabelFilterConfig {
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
            ui.label("Accepted labels (regexp):");
            let mut to_remove = Vec::new();
            for (i, id) in self.accepted.iter_mut().enumerate() {
                ui.horizontal(|ui| {
                    use crate::gui::utils::text_singleline_with_apply;

                    let unique_var_id = format!("accepted-labels-{i}-{unique_id}");
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

            ui.label("Rejected labels (regexp):");
            let mut to_remove = Vec::new();
            for (i, id) in self.rejected.iter_mut().enumerate() {
                ui.horizontal(|ui| {
                    use crate::gui::utils::text_singleline_with_apply;

                    let unique_var_id = format!("rejected-labels-{i}-{unique_id}");
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
                ui.label("Priority Accept:");
                ui.checkbox(&mut self.priority_accept, "");
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label("Accepted labels (regexp):");
            for var in &self.accepted {
                ui.label(format!("- '{}'", var));
            }
            ui.label("Rejected labels (regexp):");
            for var in &self.rejected {
                ui.label(format!("- '{}'", var));
            }
            ui.label(format!("Priority Accept: {}", self.priority_accept));
        });
    }
}

#[derive(Debug, Clone)]
pub struct LabelFilter {
    accepted: Vec<regex::Regex>,
    rejected: Vec<regex::Regex>,
    priority_accept: bool,
}

impl LabelFilter {
    pub fn from_config(config: &LabelFilterConfig, _initial_time: f32) -> Self {
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
            priority_accept: config.priority_accept,
        }
    }

    fn keep_labels(&self, labels: &[String]) -> bool {
        if self.priority_accept {
            if !self.accepted.is_empty()
                && labels
                    .iter()
                    .any(|label| self.accepted.iter().any(|re| re.is_match(label)))
            {
                return true;
            } else if labels
                .iter()
                .any(|label| self.rejected.iter().any(|re| re.is_match(label)))
            {
                return false;
            }
        } else {
            if labels
                .iter()
                .any(|label| self.rejected.iter().any(|re| re.is_match(label)))
            {
                return false;
            } else if !self.accepted.is_empty()
                && labels
                    .iter()
                    .any(|label| self.accepted.iter().any(|re| re.is_match(label)))
            {
                return true;
            }
        }
        self.accepted.is_empty()
    }
}

impl SensorFilter for LabelFilter {
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
            #[allow(deprecated)]
            SensorObservation::Speed(_) | SensorObservation::Odometry(_) => {
                unimplemented!("IdFilter cannot filter SpeedObservation");
            }
            SensorObservation::Displacement(_) => {
                unimplemented!("IdFilter cannot filter DisplacementObservation");
            }
            SensorObservation::OrientedLandmark(obs) => {
                if !self.keep_labels(&obs.labels) {
                    return None;
                }
            }
            SensorObservation::OrientedRobot(obs) => {
                if !self.keep_labels(&obs.labels) {
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
        let config = LabelFilterConfig {
            accepted: vec!["robot_good".to_string()],
            rejected: vec!["robot_.*".to_string()],
            priority_accept: true,
        };
        let filter = LabelFilter::from_config(&config, 0.0);
        assert!(!filter.keep_labels(&["robot_1".to_string(), "robot_bad".to_string()]));
        assert!(filter.keep_labels(&["robot_good".to_string(), "robot_bad".to_string()]));
        assert!(filter.keep_labels(&["robot_bad".to_string(), "robot_good".to_string()]));
        assert!(!filter.keep_labels(&["landmark_1".to_string()]));
    }

    #[test]
    fn reject_first() {
        let config = LabelFilterConfig {
            accepted: vec!["robot_.*".to_string()],
            rejected: vec!["robot_bad".to_string()],
            priority_accept: false,
        };
        let filter = LabelFilter::from_config(&config, 0.0);
        assert!(filter.keep_labels(&["robot_1".to_string()]));
        assert!(!filter.keep_labels(&["robot_1".to_string(), "robot_bad".to_string()]));
        assert!(!filter.keep_labels(&["robot_bad".to_string(), "robot_1".to_string()]));
        assert!(!filter.keep_labels(&["landmark_1".to_string()]));
    }

    #[test]
    fn accept_empty() {
        let config = LabelFilterConfig {
            accepted: vec![],
            rejected: vec!["robot_.*".to_string()],
            priority_accept: true,
        };
        let filter = LabelFilter::from_config(&config, 0.0);
        assert!(!filter.keep_labels(&["robot_1".to_string()]));
        assert!(!filter.keep_labels(&["robot_bad".to_string()]));
        assert!(filter.keep_labels(&["landmark_1".to_string()]));
    }
}
