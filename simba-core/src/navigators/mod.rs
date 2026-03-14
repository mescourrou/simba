//! Navigation strategies and shared navigation interfaces.
//!
//! This module defines the [`Navigator`] trait used by nodes to compute control errors from a
//! target behavior. It also exposes strategy-specific configuration and record enums:
//! [`NavigatorConfig`] and [`NavigatorRecord`].
//!
//! Built-in strategies include trajectory following, point-target navigation, and external/Python
//! implementations.

pub mod go_to;
pub mod trajectory;
pub mod trajectory_follower;

pub mod external_navigator;
pub mod python_navigator;

pub mod pybinds;

extern crate confy;
use std::sync::{Arc, RwLock};

use config_checker::*;
use serde_derive::{Deserialize, Serialize};
use simba_macros::config_derives;

use crate::controllers::ControllerError;
use crate::errors::SimbaResult;
#[cfg(feature = "gui")]
use crate::gui::{UIComponent, utils::string_combobox};
use crate::networking::network::Network;
use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::WorldState;

/// Enumerate the configuration of the different strategies.
///
/// The navigator computes control errors from a target behavior.
#[config_derives]
pub enum NavigatorConfig {
    /// Configuration for [`trajectory_follower::TrajectoryFollower`].
    #[check]
    TrajectoryFollower(trajectory_follower::TrajectoryFollowerConfig),
    /// Configuration for [`external_navigator::ExternalNavigator`].
    #[check]
    External(external_navigator::ExternalNavigatorConfig),
    /// Configuration for [`python_navigator::PythonNavigator`].
    #[check]
    Python(python_navigator::PythonNavigatorConfig),
    /// Configuration for [`go_to::GoTo`].
    #[check]
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
                &NavigatorConfig::to_vec(),
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
    /// Runtime record for [`TrajectoryFollower`](trajectory_follower::TrajectoryFollower).
    TrajectoryFollower(trajectory_follower::TrajectoryFollowerRecord),
    /// Runtime record for [`ExternalNavigator`](external_navigator::ExternalNavigator).
    External(external_navigator::ExternalNavigatorRecord),
    /// Runtime record for [`PythonNavigator`](python_navigator::PythonNavigator).
    Python(python_navigator::PythonNavigatorRecord),
    /// Runtime record for [`GoTo`](go_to::GoTo).
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
                egui::CollapsingHeader::new("PythonNavigator").show(ui, |ui| {
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

/// Trait managing the planning, and providing the error to the planned behavior.
pub trait Navigator:
    std::fmt::Debug + std::marker::Send + std::marker::Sync + Recordable<NavigatorRecord>
{
    /// Performs optional one-time initialization when the node starts.
    #[allow(unused_variables)]
    fn post_init(&mut self, node: &mut Node) -> SimbaResult<()> {
        Ok(())
    }

    /// Compute the error ([`ControllerError`]) between the given `state` to the planned path.
    fn compute_error(&mut self, node: &mut Node, state: WorldState) -> ControllerError;

    /// Executes per-step side effects before controller computation.
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
/// - `global_config`: The global configuration of the simulator.
/// - `va_factory`: Random variables factory for determinist behavior.
/// - `network`: Shared reference to the network, for navigators using messages.
/// - `initial_time`: Initial node time.
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
