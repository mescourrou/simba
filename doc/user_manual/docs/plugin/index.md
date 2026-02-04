# Write a plugin

SiMBA embed some minimal algorithms for each module. The user is then expected to implement its own algorithms using the plugin API.

Minimal code that build is available in the [example directory](https://gitlab.laas.fr/mescourrou/simba/-/tree/master/examples).

## Structure

To write a plugin, let say for a state estimator, you need to have at least two structs.

1. A first struct with the plugin interface, PluginAPI. It is given to the Simulator and allows the simulator to create instances of the StateEstimator.
2. A second struct which implements the state estimator, with the StateEstimator interface.

In the majority of cases, you will want to add two other structs:
3. The Configuration, which allows a customization of the StateEstimator from the global config. This struct is in the form of a serde_json::Value when the plugin interface is called. That is your responsibility to deserialize it.
4. The Record, which has two functions. First it allows to get back to a previous state (and should then include all the necessary information to get back in time), and second, it allows computing results at the end.

## Extensible modules

Presently, you can extend the following modules:

- The [state estimator](state_estimator.md), which builds a world representation from sensors and/or physics modules.
- The [navigator](navigator.md), which computes an error to the goal/trajectory from the world representation given by the state estimator.
- The [controller](controller.md), which makes commands for the robot to reduce the error computed by the navigator.
- [Physics](physics.md), which simulate the dynamics of the moving objects from the given command.
- The [sensor](sensor.md), which simulate sensor measurements from the physics state.

## Plugin API implementation
To provide the simulator with the according module instance, a struct implementing `PluginAPI` trait should be given to it.

```Rust
struct MyWonderfulPlugin {}

impl PluginAPI for MyWonderfulPlugin {
    fn get_controller(
        &self,
        config: &serde_json::Value,
        _global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> Box<dyn Controller> {
        Box::new(MyWonderfulController::from_config(
            serde_json::from_value(config.clone()).unwrap(),
            initial_time,
        ))
    }

    fn get_navigator(
        &self,
        config: &serde_json::Value,
        _global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> Box<dyn Navigator> {
        Box::new(MyWonderfulNavigator::from_config(
            serde_json::from_value(config.clone()).unwrap(),
            initial_time,
        ))
    }

    fn get_physics(
        &self,
        config: &serde_json::Value,
        _global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> Box<dyn Physics> {
        Box::new(MyWonderfulPhysics::from_config(
            serde_json::from_value(config.clone()).unwrap(),
            initial_time,
        ))
    }

    fn get_state_estimator(
        &self,
        config: &serde_json::Value,
        _global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> Box<dyn StateEstimator> {
        Box::new(MyWonderfulStateEstimator::from_config(
            serde_json::from_value(config.clone()).unwrap(),
            initial_time,
        ))
    }

    fn get_sensor(
        &self,
        config: &serde_json::Value,
        _global_config: &SimulatorConfig,
        initial_time: f32,
    ) -> Box<dyn Sensor> {
        Box::new(MyWonderfulSensor::from_config(
            serde_json::from_value(config.clone()).unwrap(),
            initial_time,
        ))
    }
}
```

Note that you can return multiple types of modules depending on the configuration. Here an example with a StateEstimator:
```Rust
struct MyWonderfulPlugin {}

#[derive(Debug, Serialize, Deserialize)]
enum StateEstimatorConfig {
    StateEstimator1(StateEstimator1Config),
    StateEstimator2(StateEstimator2Config),
}

// Definitions of StateEstimator1, StateEstimator2, ...

impl PluginAPI for MyWonderfulPlugin {
    fn get_state_estimator(
        &self,
        config: &serde_json::Value,
        _global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
        _initial_time: f32,
    ) -> Box<dyn StateEstimator> {
        match serde_json::from_value::<StateEstimatorConfig>(config.clone()).unwrap() {
            StateEstimatorConfig::StateEstimator1(c) => Box::new(StateEstimator1::from_config(&c)) as Box<dyn StateEstimator>,
            StateEstimatorConfig::StateEstimator2(c) => Box::new(StateEstimator2::from_config(&c)) as Box<dyn StateEstimator>,
        }
    }

}
```