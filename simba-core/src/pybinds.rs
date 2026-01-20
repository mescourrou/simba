use std::sync::{Arc, Mutex};

use log::debug;
use pyo3::prelude::*;

use crate::{
    controllers::{
        Controller,
        pybinds::{ControllerWrapper, PythonController},
    },
    logger::is_enabled,
    navigators::{
        Navigator,
        go_to::GoToMessage,
        pybinds::{NavigatorWrapper, PythonNavigator},
    },
    networking::{MessageTypes, network::MessageFlag},
    physics::{
        Physics,
        pybinds::{PhysicsWrapper, PythonPhysics},
    },
    plugin_api::PluginAPI,
    pywrappers::{
        CommandWrapper, ControllerErrorWrapper, GNSSObservationWrapper, NodeWrapper, ObservationWrapper, OdometryObservationWrapper, OrientedLandmarkObservationWrapper, OrientedRobotObservationWrapper, PluginAPIWrapper, Pose, SensorObservationWrapper, SimulatorWrapper, SpeedObservationWrapper, StateWrapper, UnicycleCommandWrapper, Vec2, Vec3, WorldStateWrapper, run_gui
    },
    sensors::sensor_manager::SensorTriggerMessage,
    simulator::SimulatorConfig,
    state_estimators::{
        StateEstimator,
        pybinds::{PythonStateEstimator, StateEstimatorWrapper},
    },
    utils::{
        determinist_random_variable::DeterministRandomVariableFactory, python::call_py_method,
    },
};

pub fn make_python_bindings(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<SimulatorWrapper>()?;
    m.add_class::<PluginAPIWrapper>()?;
    m.add_class::<ControllerErrorWrapper>()?;
    m.add_class::<PhysicsWrapper>()?;
    m.add_class::<StateWrapper>()?;
    m.add_class::<Pose>()?;
    m.add_class::<Vec2>()?;
    m.add_class::<Vec3>()?;
    m.add_class::<WorldStateWrapper>()?;
    m.add_class::<StateEstimatorWrapper>()?;
    m.add_class::<ObservationWrapper>()?;
    m.add_class::<SensorObservationWrapper>()?;
    m.add_class::<GNSSObservationWrapper>()?;
    m.add_class::<SpeedObservationWrapper>()?;
    #[allow(deprecated)]
    m.add_class::<OdometryObservationWrapper>()?;
    m.add_class::<OrientedLandmarkObservationWrapper>()?;
    m.add_class::<OrientedRobotObservationWrapper>()?;
    m.add_class::<ControllerWrapper>()?;
    m.add_class::<CommandWrapper>()?;
    m.add_class::<UnicycleCommandWrapper>()?;
    m.add_class::<NavigatorWrapper>()?;
    m.add_class::<NodeWrapper>()?;
    m.add_class::<MessageFlag>()?;
    m.add_class::<MessageTypes>()?;
    m.add_class::<GoToMessage>()?;
    m.add_class::<SensorTriggerMessage>()?;
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
        config: &serde_json::Value,
        global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> Box<dyn StateEstimator> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling Python API");
        }
        self.state_estimators
            .lock()
            .unwrap()
            .push(PythonStateEstimator::new(call_py_method!(
                self.api,
                "get_state_estimator",
                Py<PyAny>,
                config.to_string(),
                serde_json::to_string(global_config).expect("Failed to serialize global_config"),
                initial_time
            )));
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
        config: &serde_json::Value,
        global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> Box<dyn Controller> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling Python API");
        }
        self.controllers
            .lock()
            .unwrap()
            .push(PythonController::new(call_py_method!(
                self.api,
                "get_controller",
                Py<PyAny>,
                config.to_string(),
                serde_json::to_string(global_config).expect("Failed to serialize global_config"),
                initial_time
            )));
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
        config: &serde_json::Value,
        global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> Box<dyn Navigator> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling Python API");
        }
        self.navigators
            .lock()
            .unwrap()
            .push(PythonNavigator::new(call_py_method!(
                self.api,
                "get_navigator",
                Py<PyAny>,
                config.to_string(),
                serde_json::to_string(global_config).expect("Failed to serialize global_config"),
                initial_time
            )));
        let st = Box::new(self.navigators.lock().unwrap().last().unwrap().get_client());
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Got api {:?}", st);
        }
        st
    }

    fn get_physics(
        &self,
        config: &serde_json::Value,
        global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> Box<dyn Physics> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling Python API");
        }
        self.physics
            .lock()
            .unwrap()
            .push(PythonPhysics::new(call_py_method!(
                self.api,
                "get_physics",
                Py<PyAny>,
                config.to_string(),
                serde_json::to_string(global_config).expect("Failed to serialize global_config"),
                initial_time
            )));
        let st = Box::new(self.physics.lock().unwrap().last().unwrap().get_client());
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Got api {:?}", st);
        }
        st
    }
}
