use log::debug;
use pyo3::prelude::*;
use serde_json::Value;

use crate::{
    controllers::{controller::Controller, pybinds::PythonController}, logger::is_enabled, navigators::{navigator::Navigator, pybinds::PythonNavigator}, networking::network::MessageFlag, physics::{physics::Physics, pybinds::PythonPhysics}, pywrappers::{
        CommandWrapper, ControllerErrorWrapper, GNSSObservationWrapper, NodeWrapper, ObservationWrapper, OdometryObservationWrapper, OrientedLandmarkObservationWrapper, OrientedRobotObservationWrapper, PluginAPIWrapper, SensorObservationWrapper, SimulatorWrapper, StateWrapper, WorldStateWrapper, run_gui
    }, simulator::SimulatorConfig, state_estimators::{pybinds::PythonStateEstimator, state_estimator::StateEstimator}
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
    m.add_function(wrap_pyfunction!(run_gui, m)?)?;
    Ok(())
}

#[derive(Debug)]
pub struct PythonAPI {
    api: Py<PyAny>,
    state_estimators: Vec<PythonStateEstimator>,
    controllers: Vec<PythonController>,
    navigators: Vec<PythonNavigator>,
    physics: Vec<PythonPhysics>,
}

impl PythonAPI {
    pub fn new(m: Py<PyAny>) -> PythonAPI {
        PythonAPI {
            api: m,
            state_estimators: Vec::new(),
            controllers: Vec::new(),
            navigators: Vec::new(),
            physics: Vec::new(),
        }
    }
}

impl PythonAPI {
    pub fn check_requests(&mut self) {
        for state_estimator in &mut self.state_estimators {
            state_estimator.check_requests();
        }
        for controller in &mut self.controllers {
            controller.check_requests();
        }
        for navigator in &mut self.navigators {
            navigator.check_requests();
        }
        for physics in &mut self.physics {
            physics.check_requests();
        }
    }

    pub fn get_state_estimator(
        &mut self,
        config: &Value,
        global_config: &SimulatorConfig,
    ) -> Box<dyn StateEstimator> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling Python API");
        }
        self.state_estimators
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
        let st = Box::new(self.state_estimators.last().unwrap().get_client());
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Got api {:?}", st);
        }
        st
    }

    pub fn get_controller(
        &mut self,
        config: &Value,
        global_config: &SimulatorConfig,
    ) -> Box<dyn Controller> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling Python API");
        }
        self.controllers
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
        let st = Box::new(self.controllers.last().unwrap().get_client());
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Got api {:?}", st);
        }
        st
    }

    pub fn get_navigator(
        &mut self,
        config: &Value,
        global_config: &SimulatorConfig,
    ) -> Box<dyn Navigator> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling Python API");
        }
        self.navigators
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
        let st = Box::new(self.navigators.last().unwrap().get_client());
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Got api {:?}", st);
        }
        st
    }

    pub fn get_physics(
        &mut self,
        config: &Value,
        global_config: &SimulatorConfig,
    ) -> Box<dyn Physics> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling Python API");
        }
        self.physics.push(PythonPhysics::new(Python::with_gil(|py| {
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
        let st = Box::new(self.physics.last().unwrap().get_client());
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Got api {:?}", st);
        }
        st
    }
}
