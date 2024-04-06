/*!
Module providing the trait to link the simulator to external implementations.

Example to use an external state estimator:
```
use turtlebot_simulator::state_estimators::state_estimator::StateEstimator;
use turtlebot_simulator::{plugin_api::PluginAPI, simulator::SimulatorMetaConfig};

use serde_json::Value;

pub struct MyPlugin;

impl PluginAPI for MyPlugin {
    fn get_state_estimator(
        &self,
        config: &Value,
        meta_config: SimulatorMetaConfig,
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
use turtlebot_simulator::simulator::Simulator;
use std::path::Path;

fn main() {
    let plugin = MyPlugin {};

    let config_path = Path::new("configs/simulator_config.yaml");
    let mut simulator = Simulator::from_config_path(
        config_path,
        Some(Box::new(plugin)),
        Some(Box::from(Path::new("result.json"))),
        false,
        false);

    simulator.run(5.);


}
```
*/

use crate::simulator::SimulatorMetaConfig;
use crate::state_estimators::state_estimator::StateEstimator;

use serde_json::Value;

/// Trait to link the simulator to the external implementation.
pub trait PluginAPI {
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
        config: &Value,
        meta_config: SimulatorMetaConfig,
    ) -> Box<dyn StateEstimator>;
}
