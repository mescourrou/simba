# Navigator

The navigator computes the error to the goal given the world estimated by StateEstinator.

The minimal code is composed of a struct which implements the `Navigator` trait and the `Recordable<NavigatorRecord>` trait.
The `Navigator` trait has one required function and optional hooks:
```Rust
fn compute_error(&mut self, node: &mut Node, state: WorldState, context: &Context) -> ControllerError;
fn pre_loop_hook(&mut self, node: &mut Node, time: f32, context: &Context);
fn next_time_step(&self, context: &Context) -> Option<f32>;
```

The `Recordable<NavigatorRecord>` has to be implemented, but it can be as minimal as below if no record is needed:
```Rust
impl Recordable<NavigatorRecord> for MyWonderfulNavigator {
    fn record(&self, _context: &Context) -> NavigatorRecord {
        NavigatorRecord::External(ExternalNavigatorRecord {
            record: serde_json::to_value(MyWonderfulNavigatorRecord {}).unwrap(),
        })
    }
}
```

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
        _context: &simba::context::Context,
    ) -> ControllerError {
        ControllerError {
            lateral: 0.,
            theta: 0.,
            velocity: 0.,
            longitudinal: 0.,
        }
    }

    fn pre_loop_hook(
        &mut self,
        _node: &mut simba::node::Node,
        _time: f32,
        _context: &simba::context::Context,
    ) {
    }
}

impl Recordable<NavigatorRecord> for MyWonderfulNavigator {
    fn record(&self, _context: &Context) -> NavigatorRecord {
        NavigatorRecord::External(ExternalNavigatorRecord {
            record: serde_json::to_value(MyWonderfulNavigatorRecord {}).unwrap(),
        })
    }
}

```