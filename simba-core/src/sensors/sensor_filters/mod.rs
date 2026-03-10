use std::sync::Arc;

use simba_macros::{EnumToString, config_derives};

use crate::{
    errors::SimbaResult,
    networking::network::Network,
    node::Node,
    plugin_api::PluginAPI,
    sensors::{
        SensorObservation,
        sensor_filters::{
            external_filter::ExternalFilter,
            python_filter::{PythonFilter, PythonFilterConfig},
            range_filter::{RangeFilter, RangeFilterConfig},
            string_filter::{StringFilter, StringFilterConfig},
        },
    },
    simulator::SimulatorConfig,
    state_estimators::State,
    utils::{
        SharedRwLock, determinist_random_variable::DeterministRandomVariableFactory,
        enum_tools::EnumVariables,
    },
};
#[cfg(feature = "gui")]
use crate::{
    gui::UIComponent, sensors::sensor_filters::external_filter::ExternalFilterConfig,
    utils::enum_tools::ToVec,
};

pub mod external_filter;
pub mod python_filter;
pub mod range_filter;
pub mod string_filter;

#[config_derives]
pub enum SensorFilterConfig<SV: EnumVariables> {
    #[check]
    Range(RangeFilterConfig<SV>),
    #[check]
    Id(StringFilterConfig),
    #[check]
    Label(StringFilterConfig),
    #[check]
    Python(PythonFilterConfig),
    #[check]
    External(ExternalFilterConfig),
}

pub trait SensorFilter: Send + Sync + std::fmt::Debug {
    #[allow(unused_variables)]
    fn post_init(&mut self, node: &mut crate::node::Node, initial_time: f32) -> SimbaResult<()> {
        Ok(())
    }
    fn filter(
        &self,
        time: f32,
        observation: SensorObservation,
        observer_state: &State,
        observee_state: Option<&State>,
    ) -> Option<SensorObservation>;
}

#[cfg(feature = "gui")]
impl<SV: EnumVariables> UIComponent for SensorFilterConfig<SV> {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        let self_str = self.to_string();
        egui::CollapsingHeader::new(self_str)
            .id_salt(format!("sensor-filter-{}", unique_id))
            .show(ui, |ui| match self {
                Self::Range(cfg) => cfg.show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                ),
                Self::Python(cfg) => cfg.show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                ),
                Self::Id(cfg) | Self::Label(cfg) => cfg.show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                ),
                Self::External(cfg) => cfg.show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                ),
            });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        let self_str = self.to_string();
        egui::CollapsingHeader::new(self_str)
            .id_salt(format!("sensor-filter-{}", unique_id))
            .show(ui, |ui| {
                match self {
                    Self::Range(cfg) => cfg.show(ui, ctx, unique_id),
                    Self::Python(cfg) => cfg.show(ui, ctx, unique_id),
                    Self::Id(cfg) | Self::Label(cfg) => cfg.show(ui, ctx, unique_id),
                    Self::External(cfg) => cfg.show(ui, ctx, unique_id),
                };
            });
    }
}

#[cfg(feature = "gui")]
impl<SV: EnumVariables> SensorFilterConfig<SV> {
    pub fn show_filters_mut(
        filters: &mut Vec<SensorFilterConfig<SV>>,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
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
                buffer_stack.insert(buffer_key.clone(), "Range".to_string());
            }
            string_combobox(
                ui,
                &SensorFilterConfig::<SV>::to_vec(),
                buffer_stack.get_mut(&buffer_key).unwrap(),
                format!("filter-choice-{}", unique_id),
            );
            if ui.button("Add").clicked() {
                let selected_filter = buffer_stack.get(&buffer_key).unwrap();
                match selected_filter.as_str() {
                    "Range" => {
                        filters.push(SensorFilterConfig::Range(RangeFilterConfig::default()))
                    }
                    "Python" => {
                        filters.push(SensorFilterConfig::Python(PythonFilterConfig::default()))
                    }
                    "Id" => filters.push(SensorFilterConfig::Id(StringFilterConfig::default())),
                    "Label" => {
                        filters.push(SensorFilterConfig::Label(StringFilterConfig::default()))
                    }
                    "External" => {
                        filters.push(SensorFilterConfig::External(ExternalFilterConfig::default()))
                    }
                    _ => panic!("Where did you find this fault?"),
                };
            }
        });
    }

    pub fn show_filters(
        filters: &[SensorFilterConfig<SV>],
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        unique_id: &str,
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

#[derive(Debug, EnumToString)]
pub enum SensorFilterType<SV: EnumVariables> {
    RangeFilter(RangeFilter<SV>),
    PythonFilter(PythonFilter),
    IdFilter(StringFilter),
    LabelFilter(StringFilter),
    External(Box<dyn SensorFilter>),
}

impl<SV: EnumVariables> SensorFilterType<SV> {
    pub fn post_init(&mut self, node: &mut Node, initial_time: f32) -> SimbaResult<()> {
        match self {
            Self::PythonFilter(f) => f.post_init(node, initial_time),
            Self::External(f) => f.post_init(node, initial_time),
            Self::IdFilter(_) | Self::LabelFilter(_) | Self::RangeFilter(_) => Ok(()),
        }
    }
}

pub fn make_sensor_filter_from_config<SV: EnumVariables>(
    config: &SensorFilterConfig<SV>,
    plugin_api: &Option<Arc<dyn PluginAPI>>,
    global_config: &SimulatorConfig,
    va_factory: &Arc<DeterministRandomVariableFactory>,
    initial_time: f32,
) -> SimbaResult<SensorFilterType<SV>> {
    Ok(match &config {
        SensorFilterConfig::Range(cfg) => {
            SensorFilterType::RangeFilter(RangeFilter::from_config(cfg, initial_time))
        }
        SensorFilterConfig::Python(cfg) => SensorFilterType::PythonFilter(
            PythonFilter::from_config(cfg, global_config, initial_time)?,
        ),
        SensorFilterConfig::Id(cfg) => {
            SensorFilterType::IdFilter(StringFilter::from_config(cfg, initial_time))
        }
        SensorFilterConfig::Label(cfg) => {
            SensorFilterType::LabelFilter(StringFilter::from_config(cfg, initial_time))
        }
        SensorFilterConfig::External(cfg) => SensorFilterType::External(Box::new(
            ExternalFilter::from_config(cfg, plugin_api, global_config, va_factory, initial_time)?,
        )
            as Box<dyn SensorFilter>),
    })
}
