/*!
Provide the [`Navigator`] trait and the configuration and record enumerations.
*/

extern crate confy;
use std::sync::{Arc, RwLock};

use config_checker::macros::Check;
use serde_derive::{Deserialize, Serialize};
use simba_macros::{EnumToString, ToVec};

use super::{external_navigator, trajectory_follower};

use crate::controllers::controller::ControllerError;
#[cfg(feature = "gui")]
use crate::gui::{utils::string_combobox, UIComponent};
use crate::navigators::{go_to, python_navigator};
use crate::networking::message_handler::MessageHandler;
use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::state_estimator::WorldState;

/// Enumerate the configuration of the different strategies.
#[derive(Serialize, Deserialize, Debug, Clone, Check, EnumToString, ToVec)]
#[serde(deny_unknown_fields)]
pub enum NavigatorConfig {
    TrajectoryFollower(trajectory_follower::TrajectoryFollowerConfig),
    External(external_navigator::ExternalNavigatorConfig),
    Python(python_navigator::PythonNavigatorConfig),
    GoTo(go_to::GoToConfig),
}

#[cfg(feature = "gui")]
impl UIComponent for NavigatorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        let mut current_str = self.to_string();
        ui.horizontal(|ui| {
            ui.label("Navigator:");
            string_combobox(
                ui,
                &NavigatorConfig::to_vec()
                    .iter()
                    .map(|x| String::from(*x))
                    .collect(),
                &mut current_str,
                format!("navigator-choice-{}", unique_id),
            );
        });
        if current_str != self.to_string() {
            match current_str.as_str() {
                "TrajectoryFollower" => {
                    *self = NavigatorConfig::TrajectoryFollower(
                        trajectory_follower::TrajectoryFollowerConfig::default(),
                    )
                }
                "External" => {
                    *self = NavigatorConfig::External(
                        external_navigator::ExternalNavigatorConfig::default(),
                    )
                }
                "Python" => {
                    *self =
                        NavigatorConfig::Python(python_navigator::PythonNavigatorConfig::default())
                }
                "GoTo" => *self = NavigatorConfig::GoTo(go_to::GoToConfig::default()),
                _ => panic!("Where did you find this value?"),
            };
        }
        match self {
            NavigatorConfig::TrajectoryFollower(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            NavigatorConfig::External(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            NavigatorConfig::Python(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
            NavigatorConfig::GoTo(c) => c.show_mut(
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            ),
        }
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &String) {
        ui.horizontal(|ui| {
            ui.label(format!("Navigator: {}", self.to_string()));
        });

        match self {
            NavigatorConfig::TrajectoryFollower(c) => c.show(ui, ctx, unique_id),
            NavigatorConfig::External(c) => c.show(ui, ctx, unique_id),
            NavigatorConfig::Python(c) => c.show(ui, ctx, unique_id),
            NavigatorConfig::GoTo(c) => c.show(ui, ctx, unique_id),
        }
    }
}

/// Enumeration of the record of the different strategies.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum NavigatorRecord {
    TrajectoryFollower(trajectory_follower::TrajectoryFollowerRecord),
    External(external_navigator::ExternalNavigatorRecord),
    Python(python_navigator::PythonNavigatorRecord),
    GoTo(go_to::GoToRecord),
}

#[cfg(feature = "gui")]
impl UIComponent for NavigatorRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &String) {
        ui.vertical(|ui| match self {
            Self::TrajectoryFollower(r) => {
                egui::CollapsingHeader::new("TrajectoryFollower").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
            Self::External(r) => {
                egui::CollapsingHeader::new("ExternalNavigator").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
            Self::Python(r) => {
                egui::CollapsingHeader::new("ExternalPythonNavigator").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
            Self::GoTo(r) => {
                egui::CollapsingHeader::new("GoTo").show(ui, |ui| {
                    r.show(ui, ctx, unique_id);
                });
            }
        });
    }
}

use crate::node::Node;
use crate::recordable::Recordable;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
#[cfg(feature = "gui")]
use crate::utils::enum_tools::ToVec;

/// Trait managing the path planning, and providing the error to the planned path.
pub trait Navigator:
    std::fmt::Debug
    + std::marker::Send
    + std::marker::Sync
    + Recordable<NavigatorRecord>
    + MessageHandler
{
    /// Compute the error ([`ControllerError`]) between the given `state` to the planned path.
    fn compute_error(&mut self, node: &mut Node, state: WorldState) -> ControllerError;

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32);
}

/// Helper function to create a navigator from the given configuration.
///
/// ## Arguments
/// - `config`: The configuration of the navigator.
/// - `plugin_api`: The plugin API, to be used by the navigator.
/// - `meta_config`: The meta configuration of the simulator.
/// - `va_factory`: Random variables factory for determinist behavior.
pub fn make_navigator_from_config(
    config: &NavigatorConfig,
    plugin_api: &Option<Box<&dyn PluginAPI>>,
    global_config: &SimulatorConfig,
    va_factory: &DeterministRandomVariableFactory,
) -> Arc<RwLock<Box<dyn Navigator>>> {
    Arc::new(RwLock::new(match config {
        NavigatorConfig::TrajectoryFollower(c) => Box::new(
            trajectory_follower::TrajectoryFollower::from_config(c, global_config),
        ) as Box<dyn Navigator>,
        NavigatorConfig::External(c) => {
            Box::new(external_navigator::ExternalNavigator::from_config(
                c,
                plugin_api,
                global_config,
                va_factory,
            )) as Box<dyn Navigator>
        }
        NavigatorConfig::Python(c) => {
            Box::new(python_navigator::PythonNavigator::from_config(c, global_config).unwrap())
                as Box<dyn Navigator>
        }
        NavigatorConfig::GoTo(c) => Box::new(go_to::GoTo::from_config(c)) as Box<dyn Navigator>,
    }))
}
