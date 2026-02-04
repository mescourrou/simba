# Sensor

The sensor uses Physics to simulate realistic measurements of the robot state. It can add noise and faults to the measurements to simulate real-world conditions.

The minimal code is composed of a struct which implements the `Sensor` trait and the `Recordable<SensorRecord>` trait.
The `Sensor` trait has multiple functions which need to be implemented:
```Rust
fn init(&mut self, node: &mut Node);

fn get_observations(&mut self, node: &mut Node, time: f32) -> Vec<SensorObservation>;

fn next_time_step(&self) -> f32;
```
The `next_time_step` is very important as it will trigger the call of `get_observations`. It should not return a constant as time should be increasing.

The `Recordable<SensorRecord>` has to be implemented, but it can be as minimal as below if no record is needed:
```Rust
impl Recordable<SensorRecord> for MyWonderfulSensor {
    fn record(&self) -> SensorRecord;
}
```

## Code template

```Rust
#[derive(Clone, Debug, Serialize, Deserialize)]
struct MyWonderfulSensorObservation {
    pub data: f32,
}

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulSensorConfig {
    pub period: f32,
}

#[derive(Debug)]
struct MyWonderfulSensor {
    last_time: f32,
    period: f32,
    last_observation: Option<MyWonderfulSensorObservation>,
}

impl MyWonderfulSensor {
    pub fn from_config(config: MyWonderfulSensorConfig, initial_time: f32) -> Self {
        Self {
            last_time: initial_time,
            period: config.period,
            last_observation: None,
        }
    }
}

impl Sensor for MyWonderfulSensor {
    fn init(&mut self, node: &mut Node) {
        println!("Initializing MyWonderfulSensor for node {}", node.name());
    }

    fn get_observations(&mut self, _node: &mut Node, time: f32) -> Vec<SensorObservation> {
        self.last_observation = Some(MyWonderfulSensorObservation { data: time });
        self.last_time = time;
        // Return a custom observation here, but you can return an existing one as well (e.g. SpeedObservation)
        vec![SensorObservation::External(ExternalObservation {
            observation: serde_json::to_value(self.last_observation.as_ref().unwrap()).unwrap(),
        })]
    }

    fn next_time_step(&self) -> f32 {
        self.last_time + self.period
    }
}

impl Recordable<SensorRecord> for MyWonderfulSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::External(ExternalSensorRecord {
            record: serde_json::to_value(self.last_observation.clone()).unwrap(),
        })
    }
}
```