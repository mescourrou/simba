# Physics
Physics apply the command computed by the Controller to the simulated robot.

The minimal code is composed of a struct which implements the `Physics` trait and the `Recordable<PhysicsRecord>` trait.
The `Physics` trait has multiple functions and internal attributes should exist (at least for the state, where a reference is given):
```Rust
fn apply_command(&mut self, command: &Command, time: f32, context: &Context);

fn update_state(&mut self, time: f32, context: &Context);

fn state(&self, time: f32, context: &Context) -> State;

fn next_time_step(&self, context: &Context) -> Option<f32>;
```

The `Recordable<PhysicsRecord>` has to be implemented, but it can be as minimal as below if no record is needed:
```Rust
impl Recordable<PhysicsRecord> for MyWonderfulPhysics {
    fn record(&self, _context: &Context) -> PhysicsRecord {
        PhysicsRecord::External(ExternalPhysicsRecord {
            record: serde_json::to_value(MyWonderfulPhysicsRecord {}).unwrap(),
        })
    }
}
```

## Code template

```Rust
#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulPhysicsRecord {
    state: StateRecord,
}

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulPhysicsConfig {}

#[derive(Debug)]
struct MyWonderfulPhysics {
    state: State,
}

impl MyWonderfulPhysics {
    pub fn from_config(_config: MyWonderfulPhysicsConfig, _initial_time: f32) -> Self {
        Self {
            state: State {
                pose: Vector3::zeros(),
                velocity: Vector3::zeros(),
            },
        }
    }
}

impl Physics for MyWonderfulPhysics {
    fn apply_command(
        &mut self,
        _command: &Command,
        _time: f32,
        _context: &simba::context::Context,
    ) {
    }

    fn state(&self, _time: f32, _context: &simba::context::Context) -> State {
        self.state.clone()
    }

    fn update_state(&mut self, _time: f32, _context: &simba::context::Context) {}
}

impl Recordable<PhysicsRecord> for MyWonderfulPhysics {
    fn record(&self, context: &Context) -> PhysicsRecord {
        PhysicsRecord::External(ExternalPhysicsRecord {
            record: serde_json::to_value(MyWonderfulPhysicsRecord {
                state: self.state.record(context),
            })
            .unwrap(),
        })
    }
}
```