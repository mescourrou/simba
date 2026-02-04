# Controller

The controller makes the command to apply to the robot given the error computed by the Navigator.

The minimal code is composed of a struct which implements the `Controller` trait and the `Recordable<ControllerRecord>` trait.
The `Controller` trait has only one function:
```Rust
fn make_command(&mut self, robot: &mut Node, error: &ControllerError, time: f32) -> Command;
```

The `Recordable<ControllerRecord>` trait has to be implemented, but it can be as minimal as below if no record is needed:
```Rust
impl Recordable<ControllerRecord> for MyWonderfulController {
    fn record(&self) -> ControllerRecord {
        ControllerRecord::External(ExternalControllerRecord {
            record: serde_json::to_value(MyWonderfulControllerRecord {}).unwrap(),
        })
    }
}
```

The `MessageHandler` trait must also be implemented to allow message reception. If no message handling is needed, the `get_letter_box` method can simply return `None`.

## Code template

```Rust
#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulControllerRecord {}

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulControllerConfig {}

#[derive(Debug)]
struct MyWonderfulController {
    letter_box_rx: SharedMutex<Receiver<Envelope>>,
    letter_box_tx: Sender<Envelope>,
}

impl MyWonderfulController {
    pub fn from_config(_config: MyWonderfulControllerConfig, _initial_time: f32) -> Self {
        let (tx, rx) = mpsc::channel();
        Self {
            letter_box_rx: Arc::new(Mutex::new(rx)),
            letter_box_tx: tx,
        }
    }
}

impl Controller for MyWonderfulController {
    fn make_command(
        &mut self,
        _robot: &mut simba::node::Node,
        _error: &simba::controllers::ControllerError,
        _time: f32,
    ) -> Command {
        Command::Unicycle(UnicycleCommand {
            left_wheel_speed: 0.,
            right_wheel_speed: 0.,
        })
    }

    fn pre_loop_hook(&mut self, _node: &mut simba::node::Node, _time: f32) {
        while let Ok(_envelope) = self.letter_box_rx.lock().unwrap().try_recv() {
            // i.e. Do something with received messages
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

impl MessageHandler for MyWonderfulController {
    fn get_letter_box(&self) -> Option<Sender<Envelope>> {
        Some(self.letter_box_tx.clone())
    }
}
```