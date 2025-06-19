# State Estimator

The state estimator produce a World representation used by the Navigator, using Sensors and/or Physics.

The minimal code is composed of a struct which implements the `StateEstimator` trait and the `Stateful<StateEstimatorRecord>` trait.
The `StateEstimator` trait has multiple functions which need to be implemented:
```Rust
fn prediction_step(&mut self, robot: &mut simba::node::Node, time: f32);

fn correction_step(&mut self, robot: &mut Node, observations: &Vec<Observation>, time: f32);

fn world_state(&self) -> WorldState;

fn next_time_step(&self) -> f32;
```
The `next_time_step` is very important as it will trigger the call of `prediction_step`. It should not return a constant as time should be increasing.

**Tips**: You can use `simba::utils::maths::round_precision(time, simba::constants::TIME_ROUND)` to round the returned time to the simulator precision, avoiding time drift when using additions on floats.

The `Stateful<StateEstimatorRecord>` has to be implemented. At the difference of the other modules, the record is required to keep track of the last prediction time.
```Rust
impl Stateful<StateEstimatorRecord> for MyWonderfulController {
    fn from_record(&mut self, record: StateEstimatorRecord);
    fn record(&self) -> StateEstimatorRecord;
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
    pub fn from_config(config: MyWonderfulStateEstimatorConfig) -> Self {
        Self {
            last_prediction: 0.,
        }
    }
}

impl StateEstimator for MyWonderfulStateEstimator {
    fn prediction_step(&mut self, robot: &mut simba::node::Node, time: f32) {
        self.last_prediction = time;
    }

    fn correction_step(
        &mut self,
        robot: &mut simba::node::Node,
        observations: &Vec<simba::sensors::sensor::Observation>,
        time: f32,
    ) {
    }

    fn next_time_step(&self) -> f32 {
        self.last_prediction + 0.5
    }

    fn world_state(&self) -> WorldState {
        WorldState::new()
    }
}

impl Stateful<StateEstimatorRecord> for MyWonderfulStateEstimator {
    fn from_record(&mut self, record: StateEstimatorRecord) {
        let my_record: MyWonderfulStateEstimatorRecord = match record {
            StateEstimatorRecord::External(r) => serde_json::from_value(r.record).unwrap(),
            _ => {
                panic!("Bad record");
            }
        };
        self.last_prediction = my_record.last_prediction;
    }

    fn record(&self) -> StateEstimatorRecord {
        StateEstimatorRecord::External(ExternalEstimatorRecord {
            record: serde_json::to_value(MyWonderfulStateEstimatorRecord {
                last_prediction: self.last_prediction,
            }).unwrap(),
        })
    }
}
```