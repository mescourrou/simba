//! Sensor observation filtering framework.
//!
//! This module provides a flexible filtering system for sensor observations based on multiple criteria.
//! Filters can be configured to accept or reject observations based on numeric ranges (for enumerated variables), string patterns (for sensor IDs or labels), or custom logic via Python or external plugins.
//! Multiple filters can be chained together (the order is important); all custom filters must accept an observation for it to pass through.
//!
//! The module provides two key types:
//! - [`SensorFilterConfig`]: Configuration enum for different filter strategies (declarative)
//! - [`SensorFilterType`]: Runtime enum containing instantiated filter implementations (operational)
//! - [`SensorFilter`]: Base trait for all custom filter implementations

use std::sync::Arc;

use simba_macros::{EnumToString, config_derives};

use crate::{
    errors::SimbaResult,
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
        determinist_random_variable::DeterministRandomVariableFactory, enum_tools::EnumVariables,
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

/// Configuration enum selecting among multiple sensor observation filtering strategies.
///
/// This enum provides a unified interface for declaring sensor filters with different decision logic.
/// Each variant wraps the configuration for a specific filter type.
/// When multiple filters are applied to a sensor, all must agree to keep the observation.
/// Configuration is typically loaded from YAML and later instantiated into runtime [`SensorFilterType`] instances.
/// 
/// WARNING: all filter are not available for all sensors. This will be changed in the future.
/// 
/// # Config example:
/// ```yaml
/// filters:
///   - type: Range
///     variables: [r, theta]  # Variables names must match the sensor specific variables
///     max_range: [100, 1.57]
///     min_range: [1., -1.57]
///     inside: true
/// ```
#[config_derives]
pub enum SensorFilterConfig<SV: EnumVariables> {
    /// Range-based filtering on enumerated variables: excludes observations where numeric values fall outside specified bounds.
    #[check]
    Range(RangeFilterConfig<SV>),
    /// String pattern filtering on observed object unique id (name for nodes, id for landmarks): excludes observations matching configured regexp patterns.
    #[check]
    Id(StringFilterConfig),
    /// String pattern filtering on observed object labels: excludes observations matching configured regexp patterns.
    #[check]
    Label(StringFilterConfig),
    /// Python-based custom filtering: delegates exclusion logic to user-defined Python methods.
    #[check]
    Python(PythonFilterConfig),
    /// Plugin-based custom filtering: delegates exclusion logic to external compiled or scripted plugins.
    #[check]
    External(ExternalFilterConfig),
}

/// Trait defining the sensor observations filter interface for custom implementation.
///
/// All custom filter implementations must satisfy this trait to integrate with the observation filtering system.
/// A filter's decision is binary: it returns `Some(observation)` to keep the observation (can be modified) or `None` to exclude it.
pub trait SensorFilter: Send + Sync + std::fmt::Debug {
    /// Initializes the filter with node context and current simulation time.
    ///
    /// Called once at simulation before starting the simulation loop to allow filters to set up internal state, validate configurations,
    /// or perform any needed interactions with the node. This is optional; default implementation does nothing.
    #[allow(unused_variables)]
    fn post_init(&mut self, node: &mut crate::node::Node, initial_time: f32) -> SimbaResult<()> {
        Ok(())
    }
    /// Applies the filter to an observation and decides whether to keep or exclude it. The observation can be modified.
    ///
    /// Returns `Some(observation)` to keep, or `None` to exclude from further processing.
    /// The filter has access to observer and observee states for context-aware decisions.
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
    /// Renders mutable GUI controls for managing a list of sensor filters.
    ///
    /// Displays each filter with edit controls and remove buttons, plus an interactive dropdown
    /// to select and add new filters of different types to the list.
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

    /// Renders read-only GUI display for a list of sensor filters.
    ///
    /// Shows each filter with its current configuration in collapsed sections,
    /// but does not allow modifications (no edit controls or remove buttons).
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

/// Enum containing instantiated sensor filter implementations.
///
/// This is the runtime counterpart to [`SensorFilterConfig`]: once a config is loaded,
/// it is constructed into the appropriate variant of this enum.
/// Each variant holds the fully initialized filter instance ready for observation processing.
#[derive(Debug, EnumToString)]
pub enum SensorFilterType<SV: EnumVariables> {
    /// Instantiated range-based variable filter.
    RangeFilter(RangeFilter<SV>),
    /// Instantiated Python-based custom filter.
    PythonFilter(PythonFilter),
    /// Instantiated string pattern filter for sensor IDs.
    IdFilter(StringFilter),
    /// Instantiated string pattern filter for sensor labels.
    LabelFilter(StringFilter),
    /// Instantiated external plugin-based filter.
    External(Box<dyn SensorFilter>),
}

impl<SV: EnumVariables> SensorFilterType<SV> {
    /// Initializes the filter with node context and current simulation time.
    ///
    /// Delegates to the underlying filter's [`SensorFilter::post_init`] implementation.
    /// Some filters (e.g., range and string filters) are stateless and skip initialization.
    pub fn post_init(&mut self, node: &mut Node, initial_time: f32) -> SimbaResult<()> {
        match self {
            Self::PythonFilter(f) => f.post_init(node, initial_time),
            Self::External(f) => f.post_init(node, initial_time),
            Self::IdFilter(_) | Self::LabelFilter(_) | Self::RangeFilter(_) => Ok(()),
        }
    }
}

/// Factory function that constructs a runtime filter from its configuration.
///
/// Takes a [`SensorFilterConfig`] and instantiates the appropriate filter variant,
/// handling initialization and dependency injection (e.g., plugins, factories).
/// This function bridges configuration-time and runtime-time filter implementations.
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
