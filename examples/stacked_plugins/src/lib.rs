#[allow(unused_variables)]
use serde::{Deserialize, Serialize};
use simba::api::async_api::{AsyncApiRunner, PluginAsyncAPI};
use simba::controllers::controller::ControllerError;
use simba::navigators::external_navigator::ExternalNavigatorRecord;
use simba::navigators::navigator::{Navigator, NavigatorRecord};
use simba::networking::message_handler::MessageHandler;
use simba::networking::network::Envelope;
use simba::plugin_api::PluginAPI;
use simba::recordable::Recordable;
use simba::simulator::{AsyncSimulator, Simulator, SimulatorConfig};
use simba::state_estimators::state_estimator::{
    StateEstimator, WorldState,
};
use simba::utils::determinist_random_variable::DeterministRandomVariableFactory;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use simba::pybinds::PythonAPI;

use pyo3::prelude::*;

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
        println!("---Compute error with Rust");
        ControllerError {
            lateral: 0.,
            theta: 0.,
            velocity: 0.,
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

struct MyWonderfulPlugin {
    pub python_api: PythonAPI,
}

impl PluginAPI for MyWonderfulPlugin {
    
    // Rust Navigator
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

    // Python state estimator
    fn get_state_estimator(
        &self,
        config: &serde_json::Value,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn StateEstimator> {
        self.python_api.get_state_estimator(config, global_config, va_factory)
    }

    fn check_requests(&self) {
        self.python_api.check_requests();
    }
}

#[pymodule]
mod my_rust_plugin {
    #[pymodule_export]
    use simba::simba;
    #[pymodule_export]
    use super::start;
    
}

#[pyfunction]
fn start(python_api: Py<PyAny>) {
    // Initialize the environment
    Simulator::init_environment();

    let my_plugin = MyWonderfulPlugin {
        python_api: PythonAPI::new(python_api),
    };

    let my_plugin = Some(Box::new(my_plugin) as Box<dyn PluginAPI>);

    let mut simulator = AsyncSimulator::from_config("config_plugin.yaml".to_string(), &my_plugin);
    simulator.run(&my_plugin);
}
