# State Estimator

The state estimator produce a World representation used by the Navigator, using Sensors and/or Physics.

The minimal code is composed of a struct which implements the `StateEstimator` trait and the `Recordable<StateEstimatorRecord>` trait.
The `StateEstimator` trait has multiple functions which need to be implemented:
```Rust
fn prediction_step(&mut self, node: &mut simba::node::Node, command: Option<Command>, time: f32, context: &Context);

fn correction_step(&mut self, node: &mut simba::node::Node, observations: &[Observation], time: f32, context: &Context);

fn world_state(&self, context: &Context) -> WorldState;

fn next_time_step(&self, context: &Context) -> f32;

fn pre_loop_hook(&mut self, node: &mut simba::node::Node, time: f32, context: &Context);
```
The `next_time_step` is very important as it will trigger the call of `prediction_step`. It should not return a constant as time should be increasing.

**Tips**: You can use `simba::utils::maths::round_precision(time, simba::constants::TIME_ROUND)` to round the returned time to the simulator precision, avoiding time drift when using additions on floats.

The `Recordable<StateEstimatorRecord>` has to be implemented, but it can be as minimal as below if no record is needed:
```Rust
impl Recordable<StateEstimatorRecord> for MyWonderfulStateEstimator {
    fn record(&self, context: &Context) -> StateEstimatorRecord;
}
```

## Code template

```Rust
#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulStateEstimatorRecord {
    pub last_prediction: f32,
}

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulStateEstimatorConfig {}

#[derive(Debug)]
struct MyWonderfulStateEstimator {
    last_prediction: f32,
}

impl MyWonderfulStateEstimator {
    pub fn from_config(_config: MyWonderfulStateEstimatorConfig, initial_time: f32) -> Self {
        Self {
            last_prediction: initial_time,
        }
    }
}

impl StateEstimator for MyWonderfulStateEstimator {
    fn prediction_step(
        &mut self,
        _robot: &mut simba::node::Node,
        _command: Option<Command>,
        time: f32,
        _context: &simba::context::Context,
    ) {
        self.last_prediction = time;
    }

    fn correction_step(
        &mut self,
        _robot: &mut simba::node::Node,
        _observations: &[Observation],
        _time: f32,
        _context: &simba::context::Context,
    ) {
    }

    fn next_time_step(&self, _context: &simba::context::Context) -> f32 {
        self.last_prediction + 0.5
    }

    fn world_state(&self, _context: &simba::context::Context) -> WorldState {
        WorldState::new()
    }

    fn pre_loop_hook(
        &mut self,
        _node: &mut simba::node::Node,
        _time: f32,
        _context: &simba::context::Context,
    ) {
    }
}

impl Recordable<StateEstimatorRecord> for MyWonderfulStateEstimator {
    fn record(&self, _context: &Context) -> StateEstimatorRecord {
        StateEstimatorRecord::External(ExternalEstimatorRecord {
            record: serde_json::to_value(MyWonderfulStateEstimatorRecord {
                last_prediction: self.last_prediction,
            })
            .unwrap(),
        })
    }
}
```