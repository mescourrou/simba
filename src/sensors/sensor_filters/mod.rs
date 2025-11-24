use config_checker::macros::Check;
use serde::{Deserialize, Serialize};
use simba_macros::{EnumToString, ToVec};

use crate::utils::enum_tools::ToVec;
#[cfg(feature = "gui")]
use crate::{gui::UIComponent, simulator::SimulatorConfig};
use crate::{
    sensors::{
        sensor::SensorObservation,
        sensor_filters::range_filter::{RangeFilter, RangeFilterConfig},
    },
    state_estimators::state_estimator::State,
};

pub mod range_filter;

#[derive(Debug, Clone, Serialize, Deserialize, Check, EnumToString, ToVec)]
pub enum SensorFilterConfig {
    RangeFilter(RangeFilterConfig),
}

pub trait SensorFilter: Send + Sync + std::fmt::Debug {
    fn filter(
        &self,
        observation: SensorObservation,
        observer_state: &State,
        observee_state: Option<&State>,
    ) -> Option<SensorObservation>;
}

#[cfg(feature = "gui")]
impl UIComponent for SensorFilterConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        let self_str = self.to_string();
        egui::CollapsingHeader::new(self_str)
            .id_salt(format!("sensor-filter-{}", unique_id))
            .show(ui, |ui| {
                match self {
                    Self::RangeFilter(cfg) => cfg.show_mut(
                        ui,
                        ctx,
                        buffer_stack,
                        global_config,
                        current_node_name,
                        unique_id,
                    ),
                };
            });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &String) {
        let self_str = self.to_string();
        egui::CollapsingHeader::new(self_str)
            .id_salt(format!("sensor-filter-{}", unique_id))
            .show(ui, |ui| {
                match self {
                    Self::RangeFilter(cfg) => cfg.show(ui, ctx, unique_id),
                };
            });
    }
}

#[cfg(feature = "gui")]
impl SensorFilterConfig {
    pub fn show_filters_mut(
        filters: &mut Vec<SensorFilterConfig>,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        use crate::gui::utils::string_combobox;

        ui.label("Filters:");
        let mut filter_to_remove = None;
        for (i, filter) in filters.iter_mut().enumerate() {
            ui.horizontal_top(|ui| {
                let unique_filter_id = format!("filter-{i}-{unique_id}");
                filter.show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    &unique_filter_id,
                );

                if ui.button("X").clicked() {
                    filter_to_remove = Some(i);
                }
            });
        }
        if let Some(i) = filter_to_remove {
            filters.remove(i);
        }

        ui.horizontal(|ui| {
            let buffer_key = format!("selected-new-filter-{unique_id}");
            if !buffer_stack.contains_key(&buffer_key) {
                buffer_stack.insert(buffer_key.clone(), "RangeFilter".to_string());
            }
            string_combobox(
                ui,
                &SensorFilterConfig::to_vec()
                    .iter()
                    .map(|x| String::from(*x))
                    .collect(),
                buffer_stack.get_mut(&buffer_key).unwrap(),
                format!("filter-choice-{}", unique_id),
            );
            if ui.button("Add").clicked() {
                let selected_filter = buffer_stack.get(&buffer_key).unwrap();
                match selected_filter.as_str() {
                    "RangeFilter" => {
                        filters.push(SensorFilterConfig::RangeFilter(RangeFilterConfig::default()))
                    }
                    _ => panic!("Where did you find this fault?"),
                };
            }
        });
    }

    pub fn show_filters(
        filters: &Vec<SensorFilterConfig>,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        unique_id: &String,
    ) {
        ui.label("Filters:");
        for (i, filter) in filters.iter().enumerate() {
            ui.horizontal_top(|ui| {
                let unique_filter_id = format!("filter-{i}-{unique_id}");
                filter.show(ui, ctx, &unique_filter_id);
            });
        }
    }
}

pub fn make_sensor_filter_from_config(config: &SensorFilterConfig) -> Box<dyn SensorFilter> {
    match &config {
        SensorFilterConfig::RangeFilter(cfg) => {
            Box::new(RangeFilter::from_config(&cfg)) as Box<dyn SensorFilter>
        }
    }
}
