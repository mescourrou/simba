# Navigator

The navigator computes the error to the goal given the world estimated by StateEstinator.

The minimal code is composed of a struct which implements the `Navigator` trait and the `Recordable<NavigatorRecord>` trait.
The `Navigator` trait has only one function:`
```Rust
fn compute_error(&mut self, robot: &mut Node, state: WorldState) -> ControllerError;
```

The `Recordable<NavigatorRecord>` has to be implemented, but it can be as minimal as below if no record is needed:
```Rust
impl Recordable<NavigatorRecord> for MyWonderfulNavigator {
    fn record(&self) -> NavigatorRecord {
        NavigatorRecord::External(ExternalNavigatorRecord {
            record: serde_json::to_value(MyWonderfulNavigatorRecord {}).unwrap(),
        })
    }
}
```

The `MessageHandler` trait must also be implemented to allow message reception. If no message handling is needed, the `get_letter_box` method can simply return `None`.


## Code template

```Rust
#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulNavigatorRecord {}

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulNavigatorConfig {}

#[derive(Debug)]
struct MyWonderfulNavigator {}

impl MyWonderfulNavigator {
    pub fn from_config(_config: MyWonderfulNavigatorConfig, _initial_time: f32) -> Self {
        Self {}
    }
}

impl Navigator for MyWonderfulNavigator {
    fn compute_error(
        &mut self,
        _robot: &mut simba::node::Node,
        _state: WorldState,
    ) -> ControllerError {
        ControllerError {
            lateral: 0.,
            theta: 0.,
            velocity: 0.,
            longitudinal: 0.,
        }
    }

    fn pre_loop_hook(&mut self, _node: &mut simba::node::Node, _time: f32) {}
}

impl Recordable<NavigatorRecord> for MyWonderfulNavigator {
    fn record(&self) -> NavigatorRecord {
        NavigatorRecord::External(ExternalNavigatorRecord {
            record: serde_json::to_value(MyWonderfulNavigatorRecord {}).unwrap(),
        })
    }
}

```