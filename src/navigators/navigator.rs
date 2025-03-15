/*!
Provide the [`Navigator`] trait and the configuration and record enumerations.
*/

extern crate confy;
use std::sync::{Arc, RwLock};

use config_checker::macros::Check;
use pyo3::pyclass;
use serde_derive::{Deserialize, Serialize};

use super::{external_navigator, trajectory_follower};

use crate::controllers::controller::ControllerError;
use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorConfig;
use crate::state_estimators::state_estimator::State;

/// Enumerate the configuration of the different strategies.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(deny_unknown_fields)]
pub enum NavigatorConfig {
    TrajectoryFollower(Box<trajectory_follower::TrajectoryFollowerConfig>),
    External(Box<external_navigator::ExternalNavigatorConfig>),
}

/// Enumeration of the record of the different strategies.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass(get_all)]
pub enum NavigatorRecord {
    TrajectoryFollower(trajectory_follower::TrajectoryFollowerRecord),
    External(external_navigator::ExternalNavigatorRecord),
}

use crate::robot::Robot;
use crate::stateful::Stateful;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;

/// Trait managing the path planning, and providing the error to the planned path.
pub trait Navigator:
    std::fmt::Debug + std::marker::Send + std::marker::Sync + Stateful<NavigatorRecord>
{
    /// Compute the error ([`ControllerError`]) between the given `state` to the planned path.
    fn compute_error(&mut self, robot: &mut Robot, state: State) -> ControllerError;
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
        NavigatorConfig::TrajectoryFollower(c) => {
            Box::new(trajectory_follower::TrajectoryFollower::from_config(
                c,
                plugin_api,
                global_config,
                va_factory,
            )) as Box<dyn Navigator>
        }
        NavigatorConfig::External(c) => {
            Box::new(external_navigator::ExternalNavigator::from_config(
                c,
                plugin_api,
                global_config,
                va_factory,
            )) as Box<dyn Navigator>
        }
    }))
}
