use serde::{Deserialize, Serialize};
use simba::controllers::ControllerError;
use simba::navigators::external_navigator::ExternalNavigatorRecord;
use simba::navigators::{Navigator, NavigatorRecord};
use simba::networking::network::{Envelope, Network};
use simba::plugin_api::PluginAPI;
use simba::pybinds::PythonAPI;
use simba::recordable::Recordable;
use simba::simulator::{AsyncSimulator, Simulator, SimulatorConfig};
use simba::state_estimators::{StateEstimator, WorldState};
use simba::utils::determinist_random_variable::DeterministRandomVariableFactory;
use simba::utils::{SharedMutex, SharedRwLock};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};

use pyo3::prelude::*;

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
    pub fn from_config(_config: MyWonderfulNavigatorConfig, _initial_time: f32) -> Self {
        Self {}
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
            longitudinal: 0.,
            theta: 0.,
            velocity: 0.,
        }
    }

    fn pre_loop_hook(&mut self, _node: &mut simba::node::Node, _time: f32) {}
}

impl Recordable<NavigatorRecord> for MyWonderfulNavigator {
    fn record(&self) -> NavigatorRecord {
        NavigatorRecord::External(ExternalNavigatorRecord {
            record: serde_json::to_value(MyWonderfulNavigatorRecord {}).unwrap(),
        })
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
        _network: &SharedRwLock<Network>,
        initial_time: f32,
    ) -> Box<dyn Navigator> {
        Box::new(MyWonderfulNavigator::from_config(
            serde_json::from_value(config.clone()).unwrap(),
            initial_time,
        ))
    }

    // Python state estimator
    fn get_state_estimator(
        &self,
        config: &serde_json::Value,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        network: &SharedRwLock<Network>,
        initial_time: f32,
    ) -> Box<dyn StateEstimator> {
        self.python_api.get_state_estimator(
            config,
            global_config,
            va_factory,
            network,
            initial_time,
        )
    }

    fn check_requests(&self) {
        self.python_api.check_requests();
    }
}

#[pymodule]
mod my_rust_plugin {
    #[pymodule_export]
    use super::start;
    #[pymodule_export]
    use simba::simba;
}

#[pyfunction]
fn start(python_api: Py<PyAny>) {
    // Initialize the environment
    Simulator::init_environment();

    let my_plugin = MyWonderfulPlugin {
        python_api: PythonAPI::new(python_api),
    };

    let my_plugin = Some(Arc::new(my_plugin) as Arc<dyn PluginAPI>);

    let mut simulator = match AsyncSimulator::from_config_path("config_plugin.yaml", &my_plugin) {
        Ok(simulator) => simulator,
        Err(e) => {
            panic!("Failed to create simulator: {}", e);
        }
    };
    simulator.run(&my_plugin, Some(20.), false);
    let _ = simulator.get_records(false);
    simulator.run(&my_plugin, Some(40.), false);
    let _ = simulator.get_records(false);
    simulator.compute_results();
    simulator.stop();
}
