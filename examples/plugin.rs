#[allow(unused_variables)]
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use simba::controllers::controller::{Controller, ControllerError, ControllerRecord};
use simba::controllers::external_controller::ExternalControllerRecord;
use simba::navigators::external_navigator::ExternalNavigatorRecord;
use simba::navigators::navigator::{Navigator, NavigatorRecord};
use simba::networking::message_handler::MessageHandler;
use simba::networking::service::HasService;
use simba::physics::external_physics::ExternalPhysicsRecord;
use simba::physics::physics::{Command, GetRealStateReq, GetRealStateResp, Physics, PhysicsRecord};
use simba::plugin_api::PluginAPI;
use simba::simulator::{Simulator, SimulatorConfig};
use simba::state_estimators::external_estimator::ExternalEstimatorRecord;
use simba::state_estimators::state_estimator::{
    State, StateEstimator, StateEstimatorRecord, WorldState,
};
use simba::recordable::Recordable;
use std::path::Path;
use std::sync::{Arc, RwLock};

///////////////////////////////////
/// CONTROLLER TEMPLATE
///////////////////////////////////

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulControllerRecord {}

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulControllerConfig {}

#[derive(Debug)]
struct MyWonderfulController {}

impl MyWonderfulController {
    pub fn from_config(_config: MyWonderfulControllerConfig) -> Self {
        Self {}
    }
}

impl Controller for MyWonderfulController {
    fn make_command(
        &mut self,
        _robot: &mut simba::node::Node,
        _error: &simba::controllers::controller::ControllerError,
        _time: f32,
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

///////////////////////////////////
/// NAVIGATOR TEMPLATE
///////////////////////////////////

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulNavigatorRecord {}

#[derive(Debug, Serialize, Deserialize)]
struct MyWonderfulNavigatorConfig {}

#[derive(Debug)]
struct MyWonderfulNavigator {}

impl MyWonderfulNavigator {
    pub fn from_config(_config: MyWonderfulNavigatorConfig) -> Self {
        Self {}
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
}

impl MyWonderfulStateEstimator {
    pub fn from_config(_config: MyWonderfulStateEstimatorConfig) -> Self {
        Self {
            last_prediction: 0.,
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

///////////////////////////////////
/// MESSAGE HANDLER TEMPLATE
///////////////////////////////////

#[derive(Debug)]
struct MyWonderfulMessageHandler {}

#[derive(Debug, Serialize, Deserialize)]
struct MyMessage {}

impl MessageHandler for MyWonderfulMessageHandler {
    fn handle_message(
        &mut self,
        _robot: &mut simba::node::Node,
        _from: &String,
        message: &serde_json::Value,
        _time: f32,
    ) -> Result<(), ()> {
        match serde_json::from_value::<MyMessage>(message.clone()) {
            Err(_) => Err(()),
            Ok(m) => {
                println!("Receive message {:?}", &m);
                Ok(())
            }
        }
    }
}

struct MyWonderfulPlugin {}

impl PluginAPI for MyWonderfulPlugin {
    fn get_controller(
        &self,
        config: &serde_json::Value,
        _global_config: &SimulatorConfig,
    ) -> Box<dyn Controller> {
        Box::new(MyWonderfulController::from_config(
            serde_json::from_value(config.clone()).unwrap(),
        ))
    }

    fn get_navigator(
        &self,
        config: &serde_json::Value,
        _global_config: &SimulatorConfig,
    ) -> Box<dyn Navigator> {
        Box::new(MyWonderfulNavigator::from_config(
            serde_json::from_value(config.clone()).unwrap(),
        ))
    }

    fn get_physics(
        &self,
        config: &serde_json::Value,
        _global_config: &SimulatorConfig,
    ) -> Box<dyn Physics> {
        Box::new(MyWonderfulPhysics::from_config(
            serde_json::from_value(config.clone()).unwrap(),
        ))
    }

    fn get_state_estimator(
        &self,
        config: &serde_json::Value,
        _global_config: &SimulatorConfig,
    ) -> Box<dyn StateEstimator> {
        Box::new(MyWonderfulStateEstimator::from_config(
            serde_json::from_value(config.clone()).unwrap(),
        ))
    }

    fn get_message_handlers(
        &self,
        _robot: &simba::node::Node,
    ) -> Option<Vec<Arc<RwLock<dyn MessageHandler>>>> {
        Some(vec![Arc::new(RwLock::new(MyWonderfulMessageHandler {}))])
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

    // Run the simulator for the time given in the configuration
    // It also save the results to json
    simulator.run().unwrap();

    simulator.compute_results().unwrap();
}
