# Navigator

The navigator computes the error to the goal given the world estimated by StateEstinator.

The minimal code is composed of a struct which implements the `Navigator` trait and the `Recordable<NavigatorRecord> trait.
The `Navigator` trait has only one function:`
```Rust
fn compute_error(&mut self, robot: &mut Node, state: WorldState) -> ControllerError;
```

The `Recordable<NavigatorRecord>` has to be implemented, but it can be as minimal as below if no record is needed:
```Rust
impl Recordable<NavigatorRecord> for MyWonderfulController {
    fn record(&self) -> NavigatorRecord {}
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
    pub fn from_config(config: MyWonderfulNavigatorConfig) -> Self {
        Self {}
    }
}

impl Navigator for MyWonderfulNavigator {
    fn compute_error(
        &mut self,
        robot: &mut simba::node::Node,
        state: WorldState,
    ) -> ControllerError {
        ControllerError {
            lateral: 0.,
            theta: 0.,
            velocity: 0.,
        }
    }
}

impl Recordable<NavigatorRecord> for MyWonderfulNavigator {
    fn record(&self) -> NavigatorRecord {
        NavigatorRecord::External(ExternalNavigatorRecord {
            record: serde_json::to_value(MyWonderfulNavigatorRecord {}).unwrap(),
        })
    }
}

```