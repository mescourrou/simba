use std::sync::{Arc, Mutex};

use log::debug;
use pyo3::prelude::*;
use serde_json::Value;

use crate::{
    controllers::{controller::Controller, pybinds::PythonController},
    logger::is_enabled,
    navigators::{go_to::GoToMessage, navigator::Navigator, pybinds::PythonNavigator},
    networking::{network::MessageFlag, MessageTypes},
    physics::{physics::Physics, pybinds::PythonPhysics},
    plugin_api::PluginAPI,
    pywrappers::{
        run_gui, CommandWrapper, ControllerErrorWrapper, GNSSObservationWrapper, NodeWrapper,
        ObservationWrapper, OdometryObservationWrapper, OrientedLandmarkObservationWrapper,
        OrientedRobotObservationWrapper, PluginAPIWrapper, SensorObservationWrapper,
        SimulatorWrapper, StateWrapper, WorldStateWrapper,
    },
    simulator::SimulatorConfig,
    state_estimators::{pybinds::PythonStateEstimator, state_estimator::StateEstimator},
    utils::determinist_random_variable::DeterministRandomVariableFactory,
};

pub fn make_python_bindings(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<SimulatorWrapper>()?;
    m.add_class::<PluginAPIWrapper>()?;
    m.add_class::<ControllerErrorWrapper>()?;
    m.add_class::<PythonPhysics>()?;
    m.add_class::<StateWrapper>()?;
    m.add_class::<WorldStateWrapper>()?;
    m.add_class::<PythonStateEstimator>()?;
    m.add_class::<ObservationWrapper>()?;
    m.add_class::<SensorObservationWrapper>()?;
    m.add_class::<GNSSObservationWrapper>()?;
    m.add_class::<OdometryObservationWrapper>()?;
    m.add_class::<OrientedLandmarkObservationWrapper>()?;
    m.add_class::<OrientedRobotObservationWrapper>()?;
    m.add_class::<PythonController>()?;
    m.add_class::<CommandWrapper>()?;
    m.add_class::<PythonNavigator>()?;
    m.add_class::<NodeWrapper>()?;
    m.add_class::<MessageFlag>()?;
    m.add_class::<MessageTypes>()?;
    m.add_class::<GoToMessage>()?;
    m.add_function(wrap_pyfunction!(run_gui, m)?)?;
    Ok(())
}

#[derive(Debug)]
pub struct PythonAPI {
    api: Py<PyAny>,
    state_estimators: Mutex<Vec<PythonStateEstimator>>,
    controllers: Mutex<Vec<PythonController>>,
    navigators: Mutex<Vec<PythonNavigator>>,
    physics: Mutex<Vec<PythonPhysics>>,
}

impl PythonAPI {
    pub fn new(m: Py<PyAny>) -> PythonAPI {
        PythonAPI {
            api: m,
            state_estimators: Mutex::new(Vec::new()),
            controllers: Mutex::new(Vec::new()),
            navigators: Mutex::new(Vec::new()),
            physics: Mutex::new(Vec::new()),
        }
    }
}

impl PluginAPI for PythonAPI {
    fn check_requests(&self) {
        for state_estimator in self.state_estimators.lock().unwrap().iter_mut() {
            state_estimator.check_requests();
        }
        for controller in self.controllers.lock().unwrap().iter_mut() {
            controller.check_requests();
        }
        for navigator in self.navigators.lock().unwrap().iter_mut() {
            navigator.check_requests();
        }
        for physics in self.physics.lock().unwrap().iter_mut() {
            physics.check_requests();
        }
    }

    fn get_state_estimator(
        &self,
        config: &Value,
        global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn StateEstimator> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling Python API");
        }
        self.state_estimators
            .lock()
            .unwrap()
            .push(PythonStateEstimator::new(Python::with_gil(|py| match self
                .api
                .bind(py)
                .call_method(
                    "get_state_estimator",
                    (
                        config.to_string(),
                        serde_json::to_string(global_config)
                            .expect("Failed to serialize global_config"),
                    ),
                    None,
                ) {
                Err(e) => {
                    e.display(py);
                    panic!("Error during execution of python method 'get_state_estimator'.");
                }
                Ok(r) => r
                    .extract()
                    .expect("Expecting function return of PythonStateEstimator but failed"),
            })));
        let st = Box::new(
            self.state_estimators
                .lock()
                .unwrap()
                .last()
                .unwrap()
                .get_client(),
        );
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Got api {:?}", st);
        }
        st
    }

    fn get_controller(
        &self,
        config: &Value,
        global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn Controller> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling Python API");
        }
        self.controllers
            .lock()
            .unwrap()
            .push(PythonController::new(Python::with_gil(|py| {
                match self.api.bind(py).call_method(
                    "get_controller",
                    (
                        config.to_string(),
                        serde_json::to_string(global_config)
                            .expect("Failed to serialize global_config"),
                    ),
                    None,
                ) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error during execution of python method 'get_controller'.");
                    }
                    Ok(r) => r
                        .extract()
                        .expect("Expecting function return of PythonController but failed"),
                }
            })));
        let st = Box::new(
            self.controllers
                .lock()
                .unwrap()
                .last()
                .unwrap()
                .get_client(),
        );
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Got api {:?}", st);
        }
        st
    }

    fn get_navigator(
        &self,
        config: &Value,
        global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn Navigator> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling Python API");
        }
        self.navigators
            .lock()
            .unwrap()
            .push(PythonNavigator::new(Python::with_gil(|py| {
                match self.api.bind(py).call_method(
                    "get_navigator",
                    (
                        config.to_string(),
                        serde_json::to_string(global_config)
                            .expect("Failed to serialize global_config"),
                    ),
                    None,
                ) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error during execution of python method 'get_navigator'.");
                    }
                    Ok(r) => r
                        .extract()
                        .expect("Expecting function return of Python?avigator but failed"),
                }
            })));
        let st = Box::new(self.navigators.lock().unwrap().last().unwrap().get_client());
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Got api {:?}", st);
        }
        st
    }

    fn get_physics(
        &self,
        config: &Value,
        global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn Physics> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling Python API");
        }
        self.physics
            .lock()
            .unwrap()
            .push(PythonPhysics::new(Python::with_gil(|py| {
                match self.api.bind(py).call_method(
                    "get_physics",
                    (
                        config.to_string(),
                        serde_json::to_string(global_config)
                            .expect("Failed to serialize global_config"),
                    ),
                    None,
                ) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error during execution of python method 'get_physics'.");
                    }
                    Ok(r) => r
                        .extract()
                        .expect("Expecting function return of PythonPhysics but failed"),
                }
            })));
        let st = Box::new(self.physics.lock().unwrap().last().unwrap().get_client());
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Got api {:?}", st);
        }
        st
    }
}
