#[allow(unused_variables)]
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use simba::controllers::controller::{Controller, ControllerError, ControllerRecord};
use simba::controllers::external_controller::ExternalControllerRecord;
use simba::navigators::external_navigator::ExternalNavigatorRecord;
use simba::navigators::navigator::{Navigator, NavigatorRecord};
use simba::networking::message_handler::MessageHandler;
use simba::networking::network::Envelope;
use simba::networking::service::HasService;
use simba::physics::external_physics::ExternalPhysicsRecord;
use simba::physics::physics::{GetRealStateReq, GetRealStateResp, Physics, PhysicsRecord};
use simba::physics::robot_models::unicycle::UnicycleCommand;
use simba::physics::robot_models::Command;
use simba::plugin_api::PluginAPI;
use simba::recordable::Recordable;
use simba::simulator::{Simulator, SimulatorConfig};
use simba::state_estimators::external_estimator::ExternalEstimatorRecord;
use simba::state_estimators::state_estimator::{
    State, StateEstimator, StateEstimatorRecord, WorldState,
};
use simba::utils::determinist_random_variable::DeterministRandomVariableFactory;
use std::path::Path;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};

///////////////////////////////////
/// CONTROLLER TEMPLATE
///////////////////////////////////

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulControllerRecord {}

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulControllerConfig {}

#[derive(Debug)]
struct MyWonderfulController {
    letter_box_rx: Arc<Mutex<Receiver<Envelope>>>,
    letter_box_tx: Sender<Envelope>,
}

impl MyWonderfulController {
    pub fn from_config(_config: MyWonderfulControllerConfig) -> Self {
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
        _error: &simba::controllers::controller::ControllerError,
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

///////////////////////////////////
/// NAVIGATOR TEMPLATE
///////////////////////////////////

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulNavigatorRecord {}

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulNavigatorConfig {}

#[derive(Debug)]
struct MyWonderfulNavigator {
    letter_box_rx: Arc<Mutex<Receiver<Envelope>>>,
    letter_box_tx: Sender<Envelope>,
}

impl MyWonderfulNavigator {
    pub fn from_config(_config: MyWonderfulNavigatorConfig) -> Self {
        let (tx, rx) = mpsc::channel();
        Self {
            letter_box_rx: Arc::new(Mutex::new(rx)),
            letter_box_tx: tx,
        }
    }
}

impl Navigator for MyWonderfulNavigator {
    fn compute_error(
        &mut self,
        _robot: &mut simba::node::Node,
        _state: WorldState,
    ) -> ControllerError {
        ControllerError {
            lateral: 0.,
            theta: 0.,
            velocity: 0.,
            longitudinal: 0.,
        }
    }

    fn pre_loop_hook(&mut self, _node: &mut simba::node::Node, _time: f32) {
        while let Ok(_envelope) = self.letter_box_rx.lock().unwrap().try_recv() {
            // i.e. Do something with received messages
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

impl MessageHandler for MyWonderfulNavigator {
    fn get_letter_box(&self) -> Option<Sender<Envelope>> {
        Some(self.letter_box_tx.clone())
    }
}

///////////////////////////////////
/// PHYSICS TEMPLATE
///////////////////////////////////

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulPhysicsRecord {}

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulPhysicsConfig {}

#[derive(Debug)]
struct MyWonderfulPhysics {
    state: State,
}

impl MyWonderfulPhysics {
    pub fn from_config(_config: MyWonderfulPhysicsConfig) -> Self {
        Self {
            state: State {
                pose: Vector3::zeros(),
                velocity: 0.,
            },
        }
    }
}

impl Physics for MyWonderfulPhysics {
    fn apply_command(&mut self, _command: &Command, _time: f32) {}

    fn state(&self, _time: f32) -> State {
        self.state.clone()
    }

    fn update_state(&mut self, _time: f32) {}
}

impl HasService<GetRealStateReq, GetRealStateResp> for MyWonderfulPhysics {
    fn handle_service_requests(
        &mut self,
        _req: GetRealStateReq,
        _time: f32,
    ) -> Result<GetRealStateResp, String> {
        Err("Unimplemented".to_string())
    }
}

impl Recordable<PhysicsRecord> for MyWonderfulPhysics {
    fn record(&self) -> PhysicsRecord {
        PhysicsRecord::External(ExternalPhysicsRecord {
            record: serde_json::to_value(MyWonderfulPhysicsRecord {}).unwrap(),
        })
    }
}

///////////////////////////////////
/// STATE ESTIMATOR TEMPLATE
///////////////////////////////////

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulStateEstimatorRecord {
    pub last_prediction: f32,
}

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulStateEstimatorConfig {}

#[derive(Debug)]
struct MyWonderfulStateEstimator {
    last_prediction: f32,
    letter_box_rx: Arc<Mutex<Receiver<Envelope>>>,
    letter_box_tx: Sender<Envelope>,
}

impl MyWonderfulStateEstimator {
    pub fn from_config(_config: MyWonderfulStateEstimatorConfig) -> Self {
        let (tx, rx) = mpsc::channel();
        Self {
            last_prediction: 0.,
            letter_box_rx: Arc::new(Mutex::new(rx)),
            letter_box_tx: tx,
        }
    }
}

impl StateEstimator for MyWonderfulStateEstimator {
    fn prediction_step(&mut self, _robot: &mut simba::node::Node, time: f32) {
        self.last_prediction = time;
    }

    fn correction_step(
        &mut self,
        _robot: &mut simba::node::Node,
        _observations: &Vec<simba::sensors::sensor::Observation>,
        _time: f32,
    ) {
    }

    fn next_time_step(&self) -> f32 {
        self.last_prediction + 0.5
    }

    fn world_state(&self) -> WorldState {
        WorldState::new()
    }

    fn pre_loop_hook(&mut self, _node: &mut simba::node::Node, _time: f32) {
        while let Ok(_envelope) = self.letter_box_rx.lock().unwrap().try_recv() {
            // i.e. Do something with received messages
        }
    }
}

impl Recordable<StateEstimatorRecord> for MyWonderfulStateEstimator {
    fn record(&self) -> StateEstimatorRecord {
        StateEstimatorRecord::External(ExternalEstimatorRecord {
            record: serde_json::to_value(MyWonderfulStateEstimatorRecord {
                last_prediction: self.last_prediction,
            })
            .unwrap(),
        })
    }
}

impl MessageHandler for MyWonderfulStateEstimator {
    fn get_letter_box(&self) -> Option<Sender<Envelope>> {
        Some(self.letter_box_tx.clone())
    }
}

struct MyWonderfulPlugin {}

impl PluginAPI for MyWonderfulPlugin {
    fn get_controller(
        &self,
        config: &serde_json::Value,
        _global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn Controller> {
        Box::new(MyWonderfulController::from_config(
            serde_json::from_value(config.clone()).unwrap(),
        ))
    }

    fn get_navigator(
        &self,
        config: &serde_json::Value,
        _global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn Navigator> {
        Box::new(MyWonderfulNavigator::from_config(
            serde_json::from_value(config.clone()).unwrap(),
        ))
    }

    fn get_physics(
        &self,
        config: &serde_json::Value,
        _global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn Physics> {
        Box::new(MyWonderfulPhysics::from_config(
            serde_json::from_value(config.clone()).unwrap(),
        ))
    }

    fn get_state_estimator(
        &self,
        config: &serde_json::Value,
        _global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn StateEstimator> {
        Box::new(MyWonderfulStateEstimator::from_config(
            serde_json::from_value(config.clone()).unwrap(),
        ))

        // Example: use already existing state estimator (PythonEstimator here)
        // let config = serde_json::from_value(config.clone()).unwrap();
        // Box::new(PythonEstimator::from_config(&config, global_config).unwrap())
    }
}

fn main() {
    // Initialize the environment
    Simulator::init_environment();

    let my_plugin = MyWonderfulPlugin {};

    println!("Load configuration...");
    let mut simulator = Simulator::from_config_path(
        Path::new("config_example/config_plugin.yaml"),
        &Some(Box::new(&my_plugin)), //<- plugin API, to load external modules
    )
    .unwrap();

    println!("------------------ RUN TO 10");
    simulator.set_max_time(10.);
    // Run the simulator for the time given in the configuration
    // It also save the results to json
    simulator.run().unwrap();
    println!("{:?}", simulator.get_records(false));

    println!("------------------ RUN TO 20");
    simulator.set_max_time(20.);
    // Run the simulator for the time given in the configuration
    // It also save the results to json
    simulator.run().unwrap();
    println!("{:?}", simulator.get_records(false));

    simulator.compute_results().unwrap();
}
