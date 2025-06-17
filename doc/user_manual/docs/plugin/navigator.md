# Navigator

The navigator computes the error to the goal given the world estimated by StateEstinator.

The minimal code is composed of a struct which implements the `Navigator` trait and the `Stateful<NavigatorRecord> trait.
The `Navigator` trait has only one function:`
```Rust
fn compute_error(&mut self, robot: &mut Node, state: WorldState) -> ControllerError;
```

The `Stateful<NavigatorRecord>` has to be implemented, but it can be as minimal as below if no record is needed:
```Rust
impl Stateful<NavigatorRecord> for MyWonderfulController {
    fn from_record(&mut self, record: NavigatorRecord) { }

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

impl Stateful<NavigatorRecord> for MyWonderfulNavigator {
    fn from_record(&mut self, record: NavigatorRecord) {
        let _my_record: MyWonderfulNavigatorRecord = match record {
            NavigatorRecord::External(r) => serde_json::from_value(r.record).unwrap(),
            _ => {
                panic!("Bad record");
            }
        };
    }

    fn record(&self) -> NavigatorRecord {
        NavigatorRecord::External(ExternalNavigatorRecord {
            record: serde_json::to_value(MyWonderfulNavigatorRecord {}).unwrap(),
        })
    }
}

```