use std::sync::{Arc, Mutex};

use nalgebra::{SVector, Vector2};
use pyo3::prelude::*;

use crate::{
    api::async_api::{AsyncApi, AsyncApiRunner, PluginAsyncAPI},
    controllers::{
        controller::{Controller, ControllerError},
        pybinds::PythonController,
    },
    navigators::{navigator::Navigator, pybinds::PythonNavigator},
    physics::{
        physic::{Command, Physic},
        pybinds::PythonPhysic,
    },
    plugin_api::PluginAPI,
    pybinds::PythonAPI,
    sensors::{
        gnss_sensor::GNSSObservation, odometry_sensor::OdometryObservation,
        oriented_landmark_sensor::OrientedLandmarkObservation,
        robot_sensor::OrientedRobotObservation, sensor::Observation,
    },
    simulator::{Simulator, SimulatorConfig},
    state_estimators::{
        pybinds::PythonStateEstimator,
        state_estimator::{State, StateEstimator},
    },
};

#[derive(Clone)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "ControllerError")]
pub struct ControllerErrorWrapper {
    /// Lateral error.
    pub lateral: f32,
    /// Orientation error.
    pub theta: f32,
    /// Velocity error.
    pub velocity: f32,
}

#[pymethods]
impl ControllerErrorWrapper {
    #[new]
    pub fn new() -> Self {
        Self {
            lateral: 0.,
            theta: 0.,
            velocity: 0.,
        }
    }
}
impl ControllerErrorWrapper {
    pub fn from_ros(ce: &ControllerError) -> Self {
        Self {
            lateral: ce.lateral,
            theta: ce.theta,
            velocity: ce.velocity,
        }
    }

    pub fn to_ros(&self) -> ControllerError {
        ControllerError {
            lateral: self.lateral,
            theta: self.theta,
            velocity: self.velocity,
        }
    }
}

#[pyclass(get_all, set_all)]
#[pyo3(name = "Pose")]
#[derive(Clone)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

#[derive(Clone)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "State")]
pub struct StateWrapper {
    /// Position and orientation of the robot
    pub pose: Pose,
    /// Linear velocity.
    pub velocity: f32,
}

#[pymethods]
impl StateWrapper {
    #[new]
    pub fn new() -> Self {
        Self {
            pose: Pose {
                x: 0.,
                y: 0.,
                theta: 0.,
            },
            velocity: 0.,
        }
    }
}

impl StateWrapper {
    pub fn from_ros(s: &State) -> Self {
        Self {
            pose: Pose {
                x: s.pose[0],
                y: s.pose[1],
                theta: s.pose[2],
            },
            velocity: s.velocity,
        }
    }
    pub fn to_ros(&self) -> State {
        State {
            pose: SVector::from_vec(vec![self.pose.x, self.pose.y, self.pose.theta]),
            velocity: self.velocity,
        }
    }
}

#[derive(Clone)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "OrientedLandmarkObservation")]
pub struct OrientedLandmarkObservationWrapper {
    /// Id of the landmark
    pub id: i32,
    /// Pose of the landmark
    pub pose: Pose,
}

#[pymethods]
impl OrientedLandmarkObservationWrapper {
    #[new]
    pub fn new() -> Self {
        Self {
            id: 0,
            pose: Pose {
                x: 0.,
                y: 0.,
                theta: 0.,
            },
        }
    }
}

impl OrientedLandmarkObservationWrapper {
    pub fn from_ros(s: &OrientedLandmarkObservation) -> Self {
        Self {
            id: s.id,
            pose: Pose {
                x: s.pose[0],
                y: s.pose[1],
                theta: s.pose[2],
            },
        }
    }
    pub fn to_ros(&self) -> OrientedLandmarkObservation {
        OrientedLandmarkObservation {
            id: self.id,
            pose: SVector::from_vec(vec![self.pose.x, self.pose.y, self.pose.theta]),
        }
    }
}

#[derive(Clone)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "OdometryObservation")]
pub struct OdometryObservationWrapper {
    pub linear_velocity: f32,
    pub angular_velocity: f32,
}

#[pymethods]
impl OdometryObservationWrapper {
    #[new]
    pub fn new() -> Self {
        Self {
            linear_velocity: 0.,
            angular_velocity: 0.,
        }
    }
}

impl OdometryObservationWrapper {
    pub fn from_ros(s: &OdometryObservation) -> Self {
        Self {
            linear_velocity: s.linear_velocity,
            angular_velocity: s.angular_velocity,
        }
    }
    pub fn to_ros(&self) -> OdometryObservation {
        OdometryObservation {
            linear_velocity: self.linear_velocity,
            angular_velocity: self.angular_velocity,
        }
    }
}

#[derive(Clone)]
#[pyclass(get_all)]
#[pyo3(name = "GNSSObservation")]
pub struct GNSSObservationWrapper {
    pub position: [f32; 2],
    pub velocity: [f32; 2],
}

#[pymethods]
impl GNSSObservationWrapper {
    #[new]
    pub fn new() -> Self {
        Self {
            position: [0., 0.],
            velocity: [0., 0.],
        }
    }
}

impl GNSSObservationWrapper {
    pub fn from_ros(s: &GNSSObservation) -> Self {
        Self {
            position: s.position.into(),
            velocity: s.velocity.into(),
        }
    }
    pub fn to_ros(&self) -> GNSSObservation {
        GNSSObservation {
            position: Vector2::from(self.position),
            velocity: Vector2::from(self.velocity),
        }
    }
}

#[derive(Clone)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "OrientedRobotObservation")]
pub struct OrientedRobotObservationWrapper {
    /// Name of the Robot
    pub name: String,
    /// Pose of the Robot
    pub pose: Pose,
}

#[pymethods]
impl OrientedRobotObservationWrapper {
    #[new]
    pub fn new() -> Self {
        Self {
            name: "NoName".to_string(),
            pose: Pose {
                x: 0.,
                y: 0.,
                theta: 0.,
            },
        }
    }
}

impl OrientedRobotObservationWrapper {
    pub fn from_ros(s: &OrientedRobotObservation) -> Self {
        Self {
            name: s.name.clone(),
            pose: Pose {
                x: s.pose[0],
                y: s.pose[1],
                theta: s.pose[2],
            },
        }
    }
    pub fn to_ros(&self) -> OrientedRobotObservation {
        OrientedRobotObservation {
            name: self.name.clone(),
            pose: SVector::from_vec(vec![self.pose.x, self.pose.y, self.pose.theta]),
        }
    }
}

#[derive(Clone)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "Observation")]
pub enum ObservationWrapper {
    OrientedLandmark(OrientedLandmarkObservationWrapper),
    Odometry(OdometryObservationWrapper),
    GNSS(GNSSObservationWrapper),
    OrientedRobot(OrientedRobotObservationWrapper),
}

#[pymethods]
impl ObservationWrapper {
    #[new]
    pub fn new() -> Self {
        ObservationWrapper::GNSS(GNSSObservationWrapper::new())
    }
}

impl ObservationWrapper {
    pub fn from_ros(s: &Observation) -> Self {
        match s {
            Observation::GNSS(o) => ObservationWrapper::GNSS(GNSSObservationWrapper::from_ros(o)),
            Observation::Odometry(o) => {
                ObservationWrapper::Odometry(OdometryObservationWrapper::from_ros(o))
            }
            Observation::OrientedLandmark(o) => ObservationWrapper::OrientedLandmark(
                OrientedLandmarkObservationWrapper::from_ros(o),
            ),
            Observation::OrientedRobot(o) => {
                ObservationWrapper::OrientedRobot(OrientedRobotObservationWrapper::from_ros(o))
            }
        }
    }
    pub fn to_ros(&self) -> Observation {
        match self {
            ObservationWrapper::GNSS(o) => Observation::GNSS(o.to_ros()),
            ObservationWrapper::Odometry(o) => Observation::Odometry(o.to_ros()),
            ObservationWrapper::OrientedLandmark(o) => Observation::OrientedLandmark(o.to_ros()),
            ObservationWrapper::OrientedRobot(o) => Observation::OrientedRobot(o.to_ros()),
        }
    }
}

#[derive(Clone)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "Command")]
pub struct CommandWrapper {
    /// Left wheel speed.
    pub left_wheel_speed: f32,
    /// Right wheel speed.
    pub right_wheel_speed: f32,
}

#[pymethods]
impl CommandWrapper {
    #[new]
    pub fn new() -> CommandWrapper {
        Self {
            left_wheel_speed: 0.,
            right_wheel_speed: 0.,
        }
    }
}

impl CommandWrapper {
    pub fn from_ros(s: &Command) -> Self {
        Self {
            left_wheel_speed: s.left_wheel_speed,
            right_wheel_speed: s.right_wheel_speed,
        }
    }
    pub fn to_ros(&self) -> Command {
        Command {
            left_wheel_speed: self.left_wheel_speed,
            right_wheel_speed: self.right_wheel_speed,
        }
    }
}

#[pyclass(subclass)]
#[pyo3(name = "PluginAPI")]
pub struct PluginAPIWrapper {}

#[pymethods]
impl PluginAPIWrapper {
    #[new]
    pub fn new() -> Self {
        Self {}
    }
    /// Return the [`StateEstimator`] to be used by the
    /// [`ExternalEstimator`](`crate::state_estimators::external_estimator::ExternalEstimator`).
    ///
    /// # Arguments
    /// * `config` - Config for the external state estimator. The configuration
    /// is given using [`serde_json::Value`]. It should be converted by the
    /// external plugin to the specific configuration.
    /// * `global_config` - Full configuration of the simulator.
    ///
    /// # Return
    ///
    /// Returns the [`StateEstimator`] to use.
    pub fn get_state_estimator(
        &self,
        _config: Py<PyAny>,
        _global_config: Py<PyAny>,
    ) -> PythonStateEstimator {
        panic!("The given PluginAPI does not provide a state estimator");
    }

    /// Return the [`Controller`] to be used by the
    /// [`ExternalController`](`crate::controllers::external_controller::ExternalController`).
    ///
    /// # Arguments
    /// * `config` - Config for the external controller. The configuration
    /// is given using [`serde_json::Value`]. It should be converted by the
    /// external plugin to the specific configuration.
    /// * `global_config` - Full configuration of the simulator.
    ///
    /// # Return
    ///
    /// Returns the [`Controller`] to use.
    pub fn get_controller(
        &self,
        _config: Py<PyAny>,
        _global_config: Py<PyAny>,
    ) -> PythonController {
        panic!("The given PluginAPI does not provide a controller");
    }

    /// Return the [`Navigator`] to be used by the
    /// [`ExternalNavigator`](`crate::navigators::external_navigator::ExternalNavigator`).
    ///
    /// # Arguments
    /// * `config` - Config for the external navigator. The configuration
    /// is given using [`serde_json::Value`]. It should be converted by the
    /// external plugin to the specific configuration.
    /// * `global_config` - Full configuration of the simulator.
    ///
    /// # Return
    ///
    /// Returns the [`Navigator`] to use.
    pub fn get_navigator(&self, _config: Py<PyAny>, _global_config: Py<PyAny>) -> PythonNavigator {
        panic!("The given PluginAPI does not provide a navigator");
    }

    /// Return the [`Physic`] to be used by the
    /// [`ExternalPhysic`](`crate::physcs::external_physic::ExternalPhysic`).
    ///
    /// # Arguments
    /// * `config` - Config for the external physic. The configuration
    /// is given using [`serde_json::Value`]. It should be converted by the
    /// external plugin to the specific configuration.
    /// * `global_config` - Full configuration of the simulator.
    ///
    /// # Return
    ///
    /// Returns the [`Physic`] to use.
    pub fn get_physic(&self, _config: Py<PyAny>, _global_config: Py<PyAny>) -> PythonPhysic {
        panic!("The given PluginAPI does not provide a physic");
    }
}

#[pyclass]
#[pyo3(name = "Simulator")]
pub struct SimulatorWrapper {
    server: Arc<Mutex<AsyncApiRunner>>,
    api: AsyncApi,
    async_plugin_api: Option<PluginAsyncAPI>,
    python_api: Option<PythonAPI>,
}

#[pymethods]
impl SimulatorWrapper {
    #[staticmethod]
    #[pyo3(signature = (config_path, plugin_api=None, loglevel="off"))]
    pub fn from_config(
        config_path: String,
        plugin_api: Option<Py<PyAny>>,
        loglevel: &str,
    ) -> SimulatorWrapper {
        Simulator::init_environment(
            match loglevel.to_lowercase().as_str() {
                "debug" => log::LevelFilter::Debug,
                "info" => log::LevelFilter::Info,
                "warn" => log::LevelFilter::Warn,
                "error" => log::LevelFilter::Error,
                "off" => log::LevelFilter::Off,
                &_ => log::LevelFilter::Off,
            },
            Vec::new(),
            Vec::new(),
        );

        let server = Arc::new(Mutex::new(AsyncApiRunner::new()));
        let api = server.lock().unwrap().get_api();
        let mut wrapper = SimulatorWrapper {
            server,
            api,
            async_plugin_api: match &plugin_api {
                Some(_) => Some(PluginAsyncAPI::new()),
                None => None,
            },
            python_api: match plugin_api {
                Some(a) => Some(PythonAPI::new(a)),
                None => None,
            },
        };

        // Unsafe use because wrapper is a python object, which should be used until the end. But PyO3 does not support lifetimes to force the behaviour
        wrapper
            .server
            .lock()
            .unwrap()
            .run(match &wrapper.async_plugin_api {
                Some(api) => Some(Box::<&dyn PluginAPI>::new(unsafe {
                    std::mem::transmute::<&dyn PluginAPI, &'static dyn PluginAPI>(api)
                })),
                None => None,
            });
        wrapper.api.load_config.send(config_path).unwrap();

        if let Some(unwrapped_async_api) = &wrapper.async_plugin_api {
            let api_client = &unwrapped_async_api.client;
            let python_api = wrapper.python_api.as_mut().unwrap();
            while wrapper
                .api
                .load_config_end
                .lock()
                .unwrap()
                .try_recv()
                .is_err()
            {
                if let Ok((config, simulator_config)) = api_client
                    .get_state_estimator_request
                    .lock()
                    .unwrap()
                    .try_recv()
                {
                    let state_estimator =
                        python_api.get_state_estimator(&config, &simulator_config);
                    api_client
                        .get_state_estimator_response
                        .send(state_estimator)
                        .unwrap();
                }
                if let Ok((config, simulator_config)) =
                    api_client.get_controller_request.lock().unwrap().try_recv()
                {
                    let controller = python_api.get_controller(&config, &simulator_config);
                    api_client.get_controller_response.send(controller).unwrap();
                }
                if let Ok((config, simulator_config)) =
                    api_client.get_navigator_request.lock().unwrap().try_recv()
                {
                    let navigator = python_api.get_navigator(&config, &simulator_config);
                    api_client.get_navigator_response.send(navigator).unwrap();
                }
                if let Ok((config, simulator_config)) =
                    api_client.get_physic_request.lock().unwrap().try_recv()
                {
                    let physic = python_api.get_physic(&config, &simulator_config);
                    api_client.get_physic_response.send(physic).unwrap();
                }
                python_api.check_requests();
            }
        } else {
            wrapper.api.load_config_end.lock().unwrap().recv().unwrap();
        }

        wrapper
    }

    pub fn run(&mut self) {
        self.api
            .run
            .send(None)
            .expect("Error while sending 'run' request");
        if let Some(python_api) = &mut self.python_api {
            while self.api.run_end.lock().unwrap().try_recv().is_err() {
                python_api.check_requests();
                if Python::with_gil(|py| py.check_signals()).is_err() {
                    break;
                }
            }
        } else {
            while self.api.run_end.lock().unwrap().try_recv().is_err() {
                if Python::with_gil(|py| py.check_signals()).is_err() {
                    break;
                }
            }
        }
        // Stop server thread
        self.server.lock().unwrap().stop();
        // Calling directly the simulator to keep python in one thread
        self.server
            .lock()
            .unwrap()
            .get_simulator()
            .lock()
            .unwrap()
            .compute_results();
    }
}
