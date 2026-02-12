/*!
Module providing the [`Navigator`](navigator::Navigator) strategy, which
compute the error from the desired position.

This module also propose a implemented strategy, [`trajectory_follower`].
*/

pub mod go_to;
pub mod trajectory;
pub mod trajectory_follower;

pub mod external_navigator;
pub mod python_navigator;

pub mod pybinds;

extern crate confy;
use std::sync::{Arc, RwLock};

use serde_derive::{Deserialize, Serialize};
use simba_macros::config_derives;

use crate::controllers::ControllerError;
use crate::errors::SimbaResult;
#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::string_combobox};
use crate::networking::network::{self, Network};
use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::WorldState;

/// Enumerate the configuration of the different strategies.
#[config_derives]
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
        unique_id: &str,
    ) {
        let mut current_str = self.to_string();
        ui.horizontal(|ui| {
            ui.label("Navigator:");
            string_combobox(
                ui,
                &NavigatorConfig::to_vec()
                    .iter()
                    .map(|x: &&str| String::from(*x))
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

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.horizontal(|ui| {
            ui.label(format!("Navigator: {}", self));
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
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
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
use crate::utils::SharedRwLock;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
#[cfg(feature = "gui")]
use crate::utils::enum_tools::ToVec;

/// Trait managing the path planning, and providing the error to the planned path.
pub trait Navigator:
    std::fmt::Debug + std::marker::Send + std::marker::Sync + Recordable<NavigatorRecord>
{
    /// Compute the error ([`ControllerError`]) between the given `state` to the planned path.
    fn compute_error(&mut self, node: &mut Node, state: WorldState) -> ControllerError;

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32);

    /// Optional: return the time of the next time step. Needed if using messages
    fn next_time_step(&self) -> Option<f32> {
        None
    }
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
    plugin_api: &Option<Arc<dyn PluginAPI>>,
    global_config: &SimulatorConfig,
    va_factory: &Arc<DeterministRandomVariableFactory>,
    network: &SharedRwLock<Network>,
    initial_time: f32,
) -> SimbaResult<SharedRwLock<Box<dyn Navigator>>> {
    Ok(Arc::new(RwLock::new(match config {
        NavigatorConfig::TrajectoryFollower(c) => Box::new(
            trajectory_follower::TrajectoryFollower::from_config(c, global_config, initial_time),
        ) as Box<dyn Navigator>,
        NavigatorConfig::External(c) => {
            Box::new(external_navigator::ExternalNavigator::from_config(
                c,
                plugin_api,
                global_config,
                va_factory,
                network,
                initial_time,
            )?) as Box<dyn Navigator>
        }
        NavigatorConfig::Python(c) => Box::new(
            python_navigator::PythonNavigator::from_config(c, global_config, initial_time).unwrap(),
        ) as Box<dyn Navigator>,
        NavigatorConfig::GoTo(c) => {
            Box::new(go_to::GoTo::from_config(c, network, initial_time)) as Box<dyn Navigator>
        }
    })))
}
