/*!
Module providing the trait to link the simulator to external implementations.

Example to use an external state estimator:
```
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
```

You should use the simulator as a library. Your main.rs could be:
```compile_fail
use simba::simulator::Simulator;
use std::path::Path;

fn main() {
    let plugin = MyPlugin {};

    let config_path = Path::new("configs/simulator_config.yaml");
    let mut simulator = Simulator::from_config_path(
        config_path,
        Some(Box::new(plugin)),
        );

    simulator.run(5.);


}
```
*/

use std::sync::{Arc, RwLock};

use crate::{
    controllers::controller::Controller, navigators::navigator::Navigator, networking::message_handler::MessageHandler, physics::physic::Physic, robot::Robot, simulator::SimulatorConfig, state_estimators::state_estimator::StateEstimator
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
    ) -> Box<dyn Navigator> {
        panic!("The given PluginAPI does not provide a navigator");
    }

    /// Return the [`Physic`] to be used by the
    /// [`ExternalPhysic`](`crate::physcs::external_physic::ExternalPhysic`).
    ///
    /// # Arguments
    /// * `config` - Config for the external physic. The configuration
    /// is given using [`serde_json::Value`]. It should be converted by the
    /// external plugin to the specific configuration.
    /// * `global_config` - Full configuration of the simulator.
    ///
    /// # Return
    ///
    /// Returns the [`Physic`] to use.
    fn get_physic(&self, _config: &Value, _global_config: &SimulatorConfig) -> Box<dyn Physic> {
        panic!("The given PluginAPI does not provide a physic");
    }

    fn get_message_handlers(&self, _robot: &Robot) -> Option<Vec<Arc<RwLock<dyn MessageHandler>>>> {
        None
    }
}
