/*!
Module providing the trait to link the simulator to external implementations.

Example to use an external state estimator:
```ignore
use simba::state_estimators::state_estimator::StateEstimator;
use simba::{plugin_api::PluginAPI, simulator::SimulatorConfig};

use serde_json::Value;

pub struct MyPlugin;

impl PluginAPI for MyPlugin {
    fn get_state_estimator(
        &self,
        config: &Value,
        global_config: &SimulatorConfig,
    ) -> Box<dyn StateEstimator> {
        Box::new(MyFilter::from_config(
            &serde_json::from_value(config.clone())
                .expect("Error during parsing MyFilter config"),
        ))
    }
}
// You should use the simulator as a library. Your main.rs could be:

use simba::simulator::Simulator;
use std::path::Path;

fn main() {
    let plugin = MyPlugin {};

    let config_path = Path::new("configs/simulator_config.yaml");
    let mut simulator = Simulator::from_config_path(
        config_path,
        &Some(Box::new(plugin)),
        );

    simulator.run();


}
```
*/

use std::sync::Arc;

use crate::{
    controllers::controller::Controller, navigators::navigator::Navigator,
    physics::physics::Physics, simulator::SimulatorConfig,
    state_estimators::state_estimator::StateEstimator,
    utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use serde_json::Value;

/// Trait to link the simulator to the external implementation.
pub trait PluginAPI: Send + Sync {
    /// Return the [`StateEstimator`] to be used by the
    /// [`ExternalEstimator`](`crate::state_estimators::external_estimator::ExternalEstimator`).
    ///
    /// # Arguments
    /// * `config` - Config for the external state estimator. The configuration
    /// is given using [`serde_json::Value`]. It should be converted by the
    /// external plugin to the specific configuration.
    /// * `global_config` - Full configuration of the simulator.
    ///
    /// # Return
    ///
    /// Returns the [`StateEstimator`] to use.
    fn get_state_estimator(
        &self,
        _config: &Value,
        _global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn StateEstimator> {
        panic!("The given PluginAPI does not provide a state estimator");
    }

    /// Return the [`Controller`] to be used by the
    /// [`ExternalController`](`crate::controllers::external_controller::ExternalController`).
    ///
    /// # Arguments
    /// * `config` - Config for the external controller. The configuration
    /// is given using [`serde_json::Value`]. It should be converted by the
    /// external plugin to the specific configuration.
    /// * `global_config` - Full configuration of the simulator.
    ///
    /// # Return
    ///
    /// Returns the [`Controller`] to use.
    fn get_controller(
        &self,
        _config: &Value,
        _global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn Controller> {
        panic!("The given PluginAPI does not provide a controller");
    }

    /// Return the [`Navigator`] to be used by the
    /// [`ExternalNavigator`](`crate::navigators::external_navigator::ExternalNavigator`).
    ///
    /// # Arguments
    /// * `config` - Config for the external navigator. The configuration
    /// is given using [`serde_json::Value`]. It should be converted by the
    /// external plugin to the specific configuration.
    /// * `global_config` - Full configuration of the simulator.
    ///
    /// # Return
    ///
    /// Returns the [`Navigator`] to use.
    fn get_navigator(
        &self,
        _config: &Value,
        _global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn Navigator> {
        panic!("The given PluginAPI does not provide a navigator");
    }

    /// Return the [`Physics`] to be used by the
    /// [`ExternalPhysics`](`crate::physics::external_physics::ExternalPhysics`).
    ///
    /// # Arguments
    /// * `config` - Config for the external physics. The configuration
    /// is given using [`serde_json::Value`]. It should be converted by the
    /// external plugin to the specific configuration.
    /// * `global_config` - Full configuration of the simulator.
    ///
    /// # Return
    ///
    /// Returns the [`Physics`] to use.
    fn get_physics(
        &self,
        _config: &Value,
        _global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn Physics> {
        panic!("The given PluginAPI does not provide physics");
    }

    fn check_requests(&self) {}
}
