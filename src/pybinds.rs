use log::debug;
use pyo3::prelude::*;
use serde_json::Value;

use crate::{
    controllers::{controller::Controller, pybinds::PythonController},
    navigators::{navigator::Navigator, pybinds::PythonNavigator},
    physics::{physic::Physic, pybinds::PythonPhysic},
    pywrappers::{
        CommandWrapper, ControllerErrorWrapper, GNSSObservationWrapper, ObservationWrapper,
        OdometryObservationWrapper, OrientedLandmarkObservationWrapper,
        OrientedRobotObservationWrapper, PluginAPIWrapper, SensorObservationWrapper,
        SimulatorWrapper, StateWrapper,
    },
    simulator::SimulatorConfig,
    state_estimators::{pybinds::PythonStateEstimator, state_estimator::StateEstimator},
};

pub fn make_python_bindings(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<SimulatorWrapper>()?;
    m.add_class::<PluginAPIWrapper>()?;
    m.add_class::<ControllerErrorWrapper>()?;
    m.add_class::<PythonPhysic>()?;
    m.add_class::<StateWrapper>()?;
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
    Ok(())
}

#[derive(Debug)]
pub struct PythonAPI {
    api: Py<PyAny>,
    state_estimators: Vec<PythonStateEstimator>,
    controllers: Vec<PythonController>,
    navigators: Vec<PythonNavigator>,
    physics: Vec<PythonPhysic>,
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
        for physic in &mut self.physics {
            physic.check_requests();
        }
    }

    pub fn get_state_estimator(
        &mut self,
        config: &Value,
        global_config: &SimulatorConfig,
    ) -> Box<dyn StateEstimator> {
        println!("Calling Python API");
        self.state_estimators
            .push(PythonStateEstimator::new(Python::with_gil(|py| {
                self.api
                    .bind(py)
                    .call_method(
                        "get_state_estimator",
                        (
                            config.to_string(),
                            serde_json::to_string(global_config)
                                .expect("Failed to serialize global_config"),
                        ),
                        None,
                    )
                    .expect("Error during execution of python method 'get_state_estimator'")
                    .extract()
                    .expect("Expecting function return of PythonStateEstimator but failed")
            })));
        let st = Box::new(self.state_estimators.last().unwrap().get_client());
        debug!("Got api {:?}", st);
        st
    }

    pub fn get_controller(
        &mut self,
        config: &Value,
        global_config: &SimulatorConfig,
    ) -> Box<dyn Controller> {
        println!("Calling Python API");
        self.controllers
            .push(PythonController::new(Python::with_gil(|py| {
                self.api
                    .bind(py)
                    .call_method(
                        "get_controller",
                        (
                            config.to_string(),
                            serde_json::to_string(global_config)
                                .expect("Failed to serialize global_config"),
                        ),
                        None,
                    )
                    .expect("Error during execution of python method 'get_controller'")
                    .extract()
                    .expect("Expecting function return of PythonController but failed")
            })));
        let st = Box::new(self.controllers.last().unwrap().get_client());
        debug!("Got api {:?}", st);
        st
    }

    pub fn get_navigator(
        &mut self,
        config: &Value,
        global_config: &SimulatorConfig,
    ) -> Box<dyn Navigator> {
        println!("Calling Python API");
        self.navigators
            .push(PythonNavigator::new(Python::with_gil(|py| {
                self.api
                    .bind(py)
                    .call_method(
                        "get_navigator",
                        (
                            config.to_string(),
                            serde_json::to_string(global_config)
                                .expect("Failed to serialize global_config"),
                        ),
                        None,
                    )
                    .expect("Error during execution of python method 'get_navigator'")
                    .extract()
                    .expect("Expecting function return of Python?avigator but failed")
            })));
        let st = Box::new(self.navigators.last().unwrap().get_client());
        debug!("Got api {:?}", st);
        st
    }

    pub fn get_physic(
        &mut self,
        config: &Value,
        global_config: &SimulatorConfig,
    ) -> Box<dyn Physic> {
        println!("Calling Python API");
        self.physics.push(PythonPhysic::new(Python::with_gil(|py| {
            self.api
                .bind(py)
                .call_method(
                    "get_physic",
                    (
                        config.to_string(),
                        serde_json::to_string(global_config)
                            .expect("Failed to serialize global_config"),
                    ),
                    None,
                )
                .expect("Error during execution of python method 'get_physic'")
                .extract()
                .expect("Expecting function return of PythonPhysic but failed")
        })));
        let st = Box::new(self.physics.last().unwrap().get_client());
        debug!("Got api {:?}", st);
        st
    }
}
