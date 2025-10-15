# Physics
Physics apply the command computed by the Controller to the simulated robot.

The minimal code is composed of a struct which implements the `Physics` trait, the `Recordable<PhysicsRecord>` trait and the `HasService<GetRealStateReq, GetRealStateResp>`.
The `Physics` trait has multiple functions and internal attributes should exist (at least for the state, where a reference is given):
```Rust
fn apply_command(&mut self, command: &Command, time: f32);

fn update_state(&mut self, time: f32);

fn state(&self, time: f32) -> &State;
```

The `Recordable<PhysicsRecord>` has to be implemented, but it can be as minimal as below if no record is needed:
```Rust
impl Recordable<PhysicsRecord> for MyWonderfulController {
    fn record(&self) -> PhysicsRecord {}
}
```

The trait `HasService<GetRealStateReq, GetRealStateResp>` is used to get the real pose of the robots for the PerfectEstimator and for the RobotSensor.
There is only one function:
```Rust
fn handle_service_requests(&mut self, req: GetRealStateReq, time: f32) -> Result<GetRealStateResp, String>;
```
with `GetRealStateReq` being an empty struct and `GetRealStateResp`:
```Rust
pub struct GetRealStateResp {
    pub state: State,
}
```

## Code template

```Rust
#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulPhysicsRecord {}

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulPhysicsConfig {}

#[derive(Debug)]
struct MyWonderfulPhysics {
    state: State,
}

impl MyWonderfulPhysics {
    pub fn from_config(config: MyWonderfulPhysicsConfig) -> Self {
        Self {
            state: State {
                pose: Vector3::zeros(),
                velocity: 0.,
            },
        }
    }
}

impl Physics for MyWonderfulPhysics {
    fn apply_command(&mut self, command: &Command, time: f32) {}

    fn state(&self, time: f32) -> &State {
        &self.state
    }

    fn update_state(&mut self, time: f32) {}
}

impl HasService<GetRealStateReq, GetRealStateResp> for MyWonderfulPhysics {
    fn handle_service_requests(
        &mut self,
        req: GetRealStateReq,
        time: f32,
    ) -> Result<GetRealStateResp, String> {
        Err(String::new())
    }
}

impl Recordable<PhysicsRecord> for MyWonderfulPhysics {
    fn record(&self) -> PhysicsRecord {
        PhysicsRecord::External(ExternalPhysicsRecord {
            record: serde_json::to_value(MyWonderfulNavigatorRecord {}).unwrap(),
        })
    }
}
```