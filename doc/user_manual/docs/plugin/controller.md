# Controller

The controller makes the command to apply to the robot given the error computed by the Navigator.

The minimal code is composed of a struct which implements the `Controller` trait and the `Recordable<ControllerRecord>` trait.
The `Controller` trait has one required function and optional hooks:
```Rust
fn make_command(&mut self, robot: &mut Node, error: &ControllerError, time: f32, context: &Context) -> Command;
fn pre_loop_hook(&mut self, node: &mut Node, time: f32, context: &Context);
fn next_time_step(&self, context: &Context) -> Option<f32>;
```

The `Recordable<ControllerRecord>` trait has to be implemented, but it can be as minimal as below if no record is needed:
```Rust
impl Recordable<ControllerRecord> for MyWonderfulController {
    fn record(&self, _context: &Context) -> ControllerRecord {
        ControllerRecord::External(ExternalControllerRecord {
            record: serde_json::to_value(MyWonderfulControllerRecord {}).unwrap(),
        })
    }
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
    pub fn from_config(_config: MyWonderfulControllerConfig, _initial_time: f32) -> Self {
        Self {}
    }
}

impl Controller for MyWonderfulController {
    fn make_command(
        &mut self,
        _robot: &mut simba::node::Node,
        _error: &simba::controllers::ControllerError,
        _time: f32,
        _context: &simba::context::Context,
    ) -> Command {
        Command::Unicycle(UnicycleCommand {
            left_wheel_speed: 0.,
            right_wheel_speed: 0.,
        })
    }

    fn pre_loop_hook(
        &mut self,
        _node: &mut simba::node::Node,
        _time: f32,
        _context: &simba::context::Context,
    ) {
    }
}

impl Recordable<ControllerRecord> for MyWonderfulController {
    fn record(&self, _context: &Context) -> ControllerRecord {
        ControllerRecord::External(ExternalControllerRecord {
            record: serde_json::to_value(MyWonderfulControllerRecord {}).unwrap(),
        })
    }
}

```