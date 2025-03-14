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

use crate::{
    controllers::controller::Controller, simulator::SimulatorConfig,
    state_estimators::state_estimator::StateEstimator,
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
    /// * `meta_config` - Meta configuration of the simulator, containing
    /// information on the simulator itself (see [`SimulatorMetaConfig`]).
    ///
    /// # Return
    ///
    /// Returns the StateEstimator to use.
    fn get_state_estimator(
        &self,
        _config: &Value,
        _global_config: &SimulatorConfig,
    ) -> Box<dyn StateEstimator> {
        panic!("The given PluginAPI does not provide a state estimator");
    }

    /// Return the [`StateEstimator`] to be used by the
    /// [`ExternalEstimator`](`crate::state_estimators::external_estimator::ExternalEstimator`).
    ///
    /// # Arguments
    /// * `config` - Config for the external state estimator. The configuration
    /// is given using [`serde_json::Value`]. It should be converted by the
    /// external plugin to the specific configuration.
    /// * `meta_config` - Meta configuration of the simulator, containing
    /// information on the simulator itself (see [`SimulatorMetaConfig`]).
    ///
    /// # Return
    ///
    /// Returns the StateEstimator to use.
    fn get_controller(
        &self,
        _config: &Value,
        _global_config: &SimulatorConfig,
    ) -> Box<dyn Controller> {
        panic!("The given PluginAPI does not provide a controller");
    }
}
