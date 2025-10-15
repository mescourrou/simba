# Controller

The controller makes the command to apply to the robot given the error computed by the Navigator.

The minimal code is composed of a struct which implements the `Controller` trait and the `Recordable<ControllerRecord>` trait.
The `Controller` trait has only one function:
```Rust
fn make_command(&mut self, robot: &mut Node, error: &ControllerError, time: f32) -> Command;
```

The `Recordable<ControllerRecord>` has to be implemented, but it can be as minimal as below if no record is needed:
```Rust
impl Recordable<ControllerRecord> for MyWonderfulController {
    fn record(&self) -> ControllerRecord {}
}
```

## Code template

```Rust
#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulControllerRecord {}

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulControllerConfig {}

#[derive(Debug)]
struct MyWonderfulController {}

impl MyWonderfulController {
    pub fn from_config(config: MyWonderfulControllerConfig) -> Self {
        Self {}
    }
}

impl Controller for MyWonderfulController {
    fn make_command(
        &mut self,
        robot: &mut simba::node::Node,
        error: &simba::controllers::controller::ControllerError,
        time: f32,
    ) -> Command {
        Command {
            left_wheel_speed: 0.,
            right_wheel_speed: 0.,
        }
    }
}

impl Recordable<ControllerRecord> for MyWonderfulController {
    fn record(&self) -> ControllerRecord {
        ControllerRecord::External(ExternalControllerRecord {
            record: serde_json::to_value(MyWonderfulControllerRecord {}).unwrap(),
        })
    }
}
```