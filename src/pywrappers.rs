use std::{
    collections::BTreeMap,
    sync::{Arc, Mutex},
};

use log::debug;
use nalgebra::{SVector, Vector2, Vector3};
use pyo3::prelude::*;

use crate::{
    api::async_api::{AsyncApi, AsyncApiRunner, PluginAsyncAPI},
    controllers::{controller::ControllerError, pybinds::PythonController},
    logger::is_enabled,
    navigators::pybinds::PythonNavigator,
    physics::{physics::Command, pybinds::PythonPhysics},
    plugin_api::PluginAPI,
    pybinds::PythonAPI,
    sensors::{
        gnss_sensor::GNSSObservation,
        odometry_sensor::OdometryObservation,
        oriented_landmark_sensor::OrientedLandmarkObservation,
        robot_sensor::OrientedRobotObservation,
        sensor::{Observation, SensorObservation},
    },
    simulator::Simulator,
    state_estimators::{
        pybinds::PythonStateEstimator,
        state_estimator::{State, WorldState},
    },
    utils::occupancy_grid::OccupancyGrid,
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
    pub fn from_rust(ce: &ControllerError) -> Self {
        Self {
            lateral: ce.lateral,
            theta: ce.theta,
            velocity: ce.velocity,
        }
    }

    pub fn to_rust(&self) -> ControllerError {
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
    pub fn from_rust(s: &State) -> Self {
        Self {
            pose: Pose {
                x: s.pose[0],
                y: s.pose[1],
                theta: s.pose[2],
            },
            velocity: s.velocity,
        }
    }
    pub fn to_rust(&self) -> State {
        State {
            pose: SVector::from_vec(vec![self.pose.x, self.pose.y, self.pose.theta]),
            velocity: self.velocity,
        }
    }
}

#[derive(Clone)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "WorldState")]
pub struct WorldStateWrapper {
    pub ego: Option<StateWrapper>,
    pub objects: BTreeMap<String, StateWrapper>,
    pub landmarks: BTreeMap<i32, StateWrapper>,
    pub occupancy_grid: Option<OccupancyGridWrapper>,
}

#[pymethods]
impl WorldStateWrapper {
    #[new]
    pub fn new() -> Self {
        Self {
            ego: None,
            objects: BTreeMap::new(),
            landmarks: BTreeMap::new(),
            occupancy_grid: None,
        }
    }
}

impl WorldStateWrapper {
    pub fn from_rust(s: &WorldState) -> Self {
        Self {
            ego: match &s.ego {
                Some(st) => Some(StateWrapper::from_rust(st)),
                None => None,
            },
            landmarks: BTreeMap::from_iter(
                s.landmarks
                    .iter()
                    .map(|(id, s)| (id.clone(), StateWrapper::from_rust(s))),
            ),
            objects: BTreeMap::from_iter(
                s.objects
                    .iter()
                    .map(|(id, s)| (id.clone(), StateWrapper::from_rust(s))),
            ),
            occupancy_grid: match &s.occupancy_grid {
                Some(og) => Some(OccupancyGridWrapper::from_rust(og)),
                None => None,
            },
        }
    }
    pub fn to_rust(&self) -> WorldState {
        WorldState {
            ego: match &self.ego {
                Some(st) => Some(StateWrapper::to_rust(st)),
                None => None,
            },
            landmarks: BTreeMap::from_iter(
                self.landmarks
                    .iter()
                    .map(|(id, s)| (id.clone(), StateWrapper::to_rust(s))),
            ),
            objects: BTreeMap::from_iter(
                self.objects
                    .iter()
                    .map(|(id, s)| (id.clone(), StateWrapper::to_rust(s))),
            ),
            occupancy_grid: match &self.occupancy_grid {
                Some(og) => Some(OccupancyGridWrapper::to_rust(og)),
                None => None,
            },
        }
    }
}

#[derive(Clone)]
#[pyclass]
#[pyo3(name = "OccupancyGrid")]
pub struct OccupancyGridWrapper {
    grid: OccupancyGrid,
}

#[pymethods]
impl OccupancyGridWrapper {
    #[new]
    pub fn new(
        center: [f32; 3],
        cell_height: f32,
        cell_width: f32,
        nb_rows: usize,
        nb_cols: usize,
    ) -> Self {
        Self {
            grid: OccupancyGrid::new(
                Vector3::from(center),
                cell_height,
                cell_width,
                nb_rows,
                nb_cols,
            ),
        }
    }

    pub fn get_idx(&self, row: usize, col: usize) -> Option<f32> {
        self.grid.get_idx(row, col).cloned()
    }

    pub fn set_idx(&mut self, row: usize, col: usize, value: f32) -> bool {
        if let Some(v) = self.grid.get_idx_mut(row, col) {
            *v = value;
            true
        } else {
            false
        }
    }

    pub fn get_pos(&self, position: [f32; 2]) -> Option<f32> {
        self.grid.get_pos(Vector2::from(position)).cloned()
    }

    pub fn set_pos(&mut self, position: [f32; 2], value: f32) -> bool {
        if let Some(v) = self.grid.get_pos_mut(Vector2::from(position)) {
            *v = value;
            true
        } else {
            false
        }
    }
}

impl OccupancyGridWrapper {
    pub fn from_rust(s: &OccupancyGrid) -> Self {
        Self { grid: s.clone() }
    }
    pub fn to_rust(&self) -> OccupancyGrid {
        self.grid.clone()
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
    pub fn from_rust(s: &OrientedLandmarkObservation) -> Self {
        Self {
            id: s.id,
            pose: Pose {
                x: s.pose[0],
                y: s.pose[1],
                theta: s.pose[2],
            },
        }
    }
    pub fn to_rust(&self) -> OrientedLandmarkObservation {
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
    pub fn from_rust(s: &OdometryObservation) -> Self {
        Self {
            linear_velocity: s.linear_velocity,
            angular_velocity: s.angular_velocity,
        }
    }
    pub fn to_rust(&self) -> OdometryObservation {
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
    pub fn from_rust(s: &GNSSObservation) -> Self {
        Self {
            position: s.position.into(),
            velocity: s.velocity.into(),
        }
    }
    pub fn to_rust(&self) -> GNSSObservation {
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
    pub fn from_rust(s: &OrientedRobotObservation) -> Self {
        Self {
            name: s.name.clone(),
            pose: Pose {
                x: s.pose[0],
                y: s.pose[1],
                theta: s.pose[2],
            },
        }
    }
    pub fn to_rust(&self) -> OrientedRobotObservation {
        OrientedRobotObservation {
            name: self.name.clone(),
            pose: SVector::from_vec(vec![self.pose.x, self.pose.y, self.pose.theta]),
        }
    }
}

#[derive(Clone)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "SensorObservation")]
pub enum SensorObservationWrapper {
    OrientedLandmark(OrientedLandmarkObservationWrapper),
    Odometry(OdometryObservationWrapper),
    GNSS(GNSSObservationWrapper),
    OrientedRobot(OrientedRobotObservationWrapper),
}

#[pymethods]
impl SensorObservationWrapper {
    #[new]
    pub fn new() -> Self {
        SensorObservationWrapper::GNSS(GNSSObservationWrapper::new())
    }
}

impl SensorObservationWrapper {
    pub fn from_rust(s: &SensorObservation) -> Self {
        match s {
            SensorObservation::GNSS(o) => {
                SensorObservationWrapper::GNSS(GNSSObservationWrapper::from_rust(o))
            }
            SensorObservation::Odometry(o) => {
                SensorObservationWrapper::Odometry(OdometryObservationWrapper::from_rust(o))
            }
            SensorObservation::OrientedLandmark(o) => SensorObservationWrapper::OrientedLandmark(
                OrientedLandmarkObservationWrapper::from_rust(o),
            ),
            SensorObservation::OrientedRobot(o) => SensorObservationWrapper::OrientedRobot(
                OrientedRobotObservationWrapper::from_rust(o),
            ),
        }
    }
    pub fn to_rust(&self) -> SensorObservation {
        match self {
            SensorObservationWrapper::GNSS(o) => SensorObservation::GNSS(o.to_rust()),
            SensorObservationWrapper::Odometry(o) => SensorObservation::Odometry(o.to_rust()),
            SensorObservationWrapper::OrientedLandmark(o) => {
                SensorObservation::OrientedLandmark(o.to_rust())
            }
            SensorObservationWrapper::OrientedRobot(o) => {
                SensorObservation::OrientedRobot(o.to_rust())
            }
        }
    }
}

#[derive(Clone)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "Observation")]
pub struct ObservationWrapper {
    pub sensor_name: String,
    pub observer: String,
    pub time: f32,
    pub sensor_observation: SensorObservationWrapper,
}

#[pymethods]
impl ObservationWrapper {
    #[new]
    pub fn new() -> Self {
        Self {
            sensor_name: "some_sensor".to_string(),
            observer: "someone".to_string(),
            time: 0.,
            sensor_observation: SensorObservationWrapper::new(),
        }
    }
}

impl ObservationWrapper {
    pub fn from_rust(s: &Observation) -> Self {
        Self {
            sensor_name: s.sensor_name.clone(),
            observer: s.observer.clone(),
            time: s.time,
            sensor_observation: SensorObservationWrapper::from_rust(&s.sensor_observation),
        }
    }
    pub fn to_rust(&self) -> Observation {
        Observation {
            sensor_name: self.sensor_name.clone(),
            observer: self.observer.clone(),
            time: self.time,
            sensor_observation: self.sensor_observation.to_rust(),
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
    pub fn from_rust(s: &Command) -> Self {
        Self {
            left_wheel_speed: s.left_wheel_speed,
            right_wheel_speed: s.right_wheel_speed,
        }
    }
    pub fn to_rust(&self) -> Command {
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
    /// Return the [`StateEstimator`](crate::state_estimators::state_estimator::StateEstimator) to be used by the
    /// [`ExternalEstimator`](crate::state_estimators::external_estimator::ExternalEstimator).
    ///
    /// # Arguments
    /// * `config` - Config for the external state estimator. The configuration
    /// is given using [`serde_json::Value`]. It should be converted by the
    /// external plugin to the specific configuration.
    /// * `global_config` - Full configuration of the simulator.
    ///
    /// # Return
    ///
    /// Returns the [`StateEstimator`](crate::state_estimators::state_estimator::StateEstimator) to use.
    pub fn get_state_estimator(
        &self,
        _config: Py<PyAny>,
        _global_config: Py<PyAny>,
    ) -> PythonStateEstimator {
        panic!("The given PluginAPI does not provide a state estimator");
    }

    /// Return the [`Controller`](crate::controllers::controller::Controller) to be used by the
    /// [`ExternalController`](crate::controllers::external_controller::ExternalController).
    ///
    /// # Arguments
    /// * `config` - Config for the external controller. The configuration
    /// is given using [`serde_json::Value`]. It should be converted by the
    /// external plugin to the specific configuration.
    /// * `global_config` - Full configuration of the simulator.
    ///
    /// # Return
    ///
    /// Returns the [`Controller`](crate::controllers::controller::Controller) to use.
    pub fn get_controller(
        &self,
        _config: Py<PyAny>,
        _global_config: Py<PyAny>,
    ) -> PythonController {
        panic!("The given PluginAPI does not provide a controller");
    }

    /// Return the [`Navigator`](crate::navigators::navigator::Navigator) to be used by the
    /// [`ExternalNavigator`](crate::navigators::external_navigator::ExternalNavigator).
    ///
    /// # Arguments
    /// * `config` - Config for the external navigator. The configuration
    /// is given using [`serde_json::Value`]. It should be converted by the
    /// external plugin to the specific configuration.
    /// * `global_config` - Full configuration of the simulator.
    ///
    /// # Return
    ///
    /// Returns the [`Navigator`](crate::navigators::navigator::Navigator) to use.
    pub fn get_navigator(&self, _config: Py<PyAny>, _global_config: Py<PyAny>) -> PythonNavigator {
        panic!("The given PluginAPI does not provide a navigator");
    }

    /// Return the [`Physics`](crate::physics::physics::Physics) to be used by the
    /// [`ExternalPhysics`](crate::physics::external_physics::ExternalPhysics).
    ///
    /// # Arguments
    /// * `config` - Config for the external physics. The configuration
    /// is given using [`serde_json::Value`]. It should be converted by the
    /// external plugin to the specific configuration.
    /// * `global_config` - Full configuration of the simulator.
    ///
    /// # Return
    ///
    /// Returns the [`Physics`](crate::physics::physics::Physics) to use.
    pub fn get_physics(&self, _config: Py<PyAny>, _global_config: Py<PyAny>) -> PythonPhysics {
        panic!("The given PluginAPI does not provide physics");
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
    #[pyo3(signature = (config_path, plugin_api=None))]
    pub fn from_config(config_path: String, plugin_api: Option<Py<PyAny>>) -> SimulatorWrapper {
        Simulator::init_environment();

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
        wrapper.api.load_config.async_call(config_path);

        if let Some(unwrapped_async_api) = &wrapper.async_plugin_api {
            let api_client = &unwrapped_async_api.client;
            let python_api = wrapper.python_api.as_mut().unwrap();
            while wrapper.api.load_config.try_get_result().is_none() {
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
                    api_client.get_physics_request.lock().unwrap().try_recv()
                {
                    let physic = python_api.get_physics(&config, &simulator_config);
                    api_client.get_physics_response.send(physic).unwrap();
                }
                python_api.check_requests();
            }
        } else {
            wrapper.api.load_config.wait_result().unwrap().unwrap();
        }

        wrapper
    }

    pub fn run(&mut self) {
        self.api.run.async_call(None);
        if let Some(python_api) = &mut self.python_api {
            while self.api.run.try_get_result().is_none() {
                python_api.check_requests();
                if Python::with_gil(|py| py.check_signals()).is_err() {
                    break;
                }
            }
        } else {
            while self.api.run.try_get_result().is_none() {
                if Python::with_gil(|py| py.check_signals()).is_err() {
                    break;
                }
            }
        }
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Stop server");
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
            .compute_results()
            .unwrap();
    }
}

#[cfg(feature = "gui")]
#[pyfunction]
#[pyo3(signature = (plugin_api=None))]
pub fn run_gui(py: Python, plugin_api: Option<Py<PyAny>>) {
    use std::{sync::RwLock, thread};

    use crate::gui;
    let async_plugin_api = match &plugin_api {
        Some(_) => Some(PluginAsyncAPI::new()),
        None => None,
    };
    let mut python_api = match plugin_api {
        Some(a) => Some(PythonAPI::new(a)),
        None => None,
    };

    let api_client = match &async_plugin_api {
        Some(api) => Some(api.client.clone()),
        None => None,
    };

    let running = Arc::new(RwLock::new(true));
    let local_running = running.clone();

    let thread_handle = thread::spawn(move || {
        while *running.read().unwrap() {
            if let Some(api_client) = &api_client {
                let python_api = python_api.as_mut().unwrap();
                // TODO: Multiple wait can be optimized the same way than AsyncAPI runner (or maybe not as it's Python)
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
                    api_client.get_physics_request.lock().unwrap().try_recv()
                {
                    let physic = python_api.get_physics(&config, &simulator_config);
                    api_client.get_physics_response.send(physic).unwrap();
                }
                python_api.check_requests();
            }
        }
    });

    // egui needs to be run in the main thread. However, Python get the GIL. So we need to free the GIL so that the thread spawn before can call Python functions.
    // allow_threads reacquire the GIL when it exits the function. At this point, we are sure that the other thread is finished.
    py.allow_threads(|| {
        gui::run_gui(match &async_plugin_api {
            Some(api) => Some(Box::<&dyn PluginAPI>::new(unsafe {
                std::mem::transmute::<&dyn PluginAPI, &'static dyn PluginAPI>(api)
            })),
            None => None,
        });
        *local_running.write().unwrap() = false;
        thread_handle.join().unwrap();
    });
}

#[cfg(not(feature = "gui"))]
#[pyfunction]
#[pyo3(signature = (_plugin_api=None))]
pub fn run_gui(_py: Python, _plugin_api: Option<Py<PyAny>>) {
    unimplemented!("run_gui not available. Compile python package with 'gui' feature");
}
