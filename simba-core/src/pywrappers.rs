#![allow(clippy::useless_conversion)]
use std::{collections::BTreeMap, str::FromStr, sync::Arc};

use nalgebra::{SVector, Vector2, Vector3};
use pyo3::{exceptions::PyTypeError, prelude::*};
use simba_com::pub_sub::{MultiClientTrait, PathKey};
use simba_macros::EnumToString;

#[cfg(feature = "gui")]
use std::path::Path;

#[cfg(feature = "gui")]
use crate::api::async_api::PluginAsyncAPI;

use crate::{
    controllers::{ControllerError, pybinds::ControllerWrapper},
    navigators::pybinds::NavigatorWrapper,
    networking::{
        MessageTypes,
        network::{Envelope, MessageFlag},
    },
    node::Node,
    physics::{
        pybinds::PhysicsWrapper,
        robot_models::{Command, holonomic::HolonomicCommand, unicycle::UnicycleCommand},
    },
    plugin_api::PluginAPI,
    pybinds::PythonAPI,
    sensors::{
        Observation, SensorObservation, displacement_sensor::DisplacementObservation,
        gnss_sensor::GNSSObservation, oriented_landmark_sensor::OrientedLandmarkObservation,
        robot_sensor::OrientedRobotObservation, speed_sensor::SpeedObservation,
    },
    simulator::{AsyncSimulator, SimbaBrokerMultiClient, Simulator},
    state_estimators::{State, WorldState, pybinds::StateEstimatorWrapper},
    utils::occupancy_grid::OccupancyGrid,
};

#[derive(Clone, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "ControllerError")]
pub struct ControllerErrorWrapper {
    /// Lateral error.
    pub lateral: f32,
    /// Longitudinal error.
    pub longitudinal: f32,
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
            longitudinal: 0.,
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
            longitudinal: ce.longitudinal,
        }
    }

    pub fn to_rust(&self) -> ControllerError {
        ControllerError {
            lateral: self.lateral,
            longitudinal: self.longitudinal,
            theta: self.theta,
            velocity: self.velocity,
        }
    }
}

impl Default for ControllerErrorWrapper {
    fn default() -> Self {
        Self::new()
    }
}

#[pyclass(get_all, set_all)]
#[pyo3(name = "Pose")]
#[derive(Clone, Debug)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

#[pyclass(get_all, set_all)]
#[pyo3(name = "Vec2")]
#[derive(Clone, Debug)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

#[pyclass(get_all, set_all)]
#[pyo3(name = "Vec3")]
#[derive(Clone, Debug)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Clone, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "State")]
pub struct StateWrapper {
    /// Position and orientation of the robot
    pub pose: Pose,
    /// Linear velocity.
    pub velocity: Vec3,
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
            velocity: Vec3 {
                x: 0.,
                y: 0.,
                z: 0.,
            },
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
            velocity: Vec3 {
                x: s.velocity[0],
                y: s.velocity[1],
                z: s.velocity[2],
            },
        }
    }
    pub fn to_rust(&self) -> State {
        State {
            pose: SVector::from_vec(vec![self.pose.x, self.pose.y, self.pose.theta]),
            velocity: SVector::from_vec(vec![self.velocity.x, self.velocity.y, self.velocity.z]),
        }
    }
}

impl Default for StateWrapper {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Debug)]
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
            ego: s.ego.as_ref().map(StateWrapper::from_rust),
            landmarks: BTreeMap::from_iter(
                s.landmarks
                    .iter()
                    .map(|(id, s)| (*id, StateWrapper::from_rust(s))),
            ),
            objects: BTreeMap::from_iter(
                s.objects
                    .iter()
                    .map(|(id, s)| (id.clone(), StateWrapper::from_rust(s))),
            ),
            occupancy_grid: s
                .occupancy_grid
                .as_ref()
                .map(OccupancyGridWrapper::from_rust),
        }
    }
    pub fn to_rust(&self) -> WorldState {
        WorldState {
            ego: self.ego.as_ref().map(StateWrapper::to_rust),
            landmarks: BTreeMap::from_iter(
                self.landmarks
                    .iter()
                    .map(|(id, s)| (*id, StateWrapper::to_rust(s))),
            ),
            objects: BTreeMap::from_iter(
                self.objects
                    .iter()
                    .map(|(id, s)| (id.clone(), StateWrapper::to_rust(s))),
            ),
            occupancy_grid: self
                .occupancy_grid
                .as_ref()
                .map(OccupancyGridWrapper::to_rust),
        }
    }
}

impl Default for WorldStateWrapper {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Debug)]
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

#[derive(Clone, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "OrientedLandmarkObservation")]
pub struct OrientedLandmarkObservationWrapper {
    /// Id of the landmark
    pub id: i32,
    /// Labels of the landmark
    pub labels: Vec<String>,
    /// Pose of the landmark
    pub pose: Pose,
    /// Applied fault in JSON format
    pub applied_faults: String,
    /// Width of the landmark
    pub width: f32,
    /// Height of the landmark
    pub height: f32,
}

#[pymethods]
impl OrientedLandmarkObservationWrapper {
    #[new]
    pub fn new() -> Self {
        Self {
            id: 0,
            labels: Vec::new(),
            pose: Pose {
                x: 0.,
                y: 0.,
                theta: 0.,
            },
            width: 0.,
            height: 1.,
            applied_faults: "[]".to_string(),
        }
    }
}

impl OrientedLandmarkObservationWrapper {
    pub fn from_rust(s: &OrientedLandmarkObservation) -> Self {
        Self {
            id: s.id,
            labels: s.labels.clone(),
            pose: Pose {
                x: s.pose[0],
                y: s.pose[1],
                theta: s.pose[2],
            },
            applied_faults: serde_json::to_string(&s.applied_faults).unwrap(),
            width: s.width,
            height: s.height,
        }
    }
    pub fn to_rust(&self) -> OrientedLandmarkObservation {
        OrientedLandmarkObservation {
            id: self.id,
            labels: self.labels.clone(),
            pose: SVector::from_vec(vec![self.pose.x, self.pose.y, self.pose.theta]),
            applied_faults: serde_json::from_str(&self.applied_faults).unwrap(),
            width: self.width,
            height: self.height,
        }
    }
}

impl Default for OrientedLandmarkObservationWrapper {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "SpeedObservation")]
pub struct SpeedObservationWrapper {
    pub linear_velocity: f32,
    pub lateral_velocity: f32,
    pub angular_velocity: f32,
    /// Applied faults in JSON format
    pub applied_faults: String,
}

#[pymethods]
impl SpeedObservationWrapper {
    #[new]
    pub fn new() -> Self {
        Self {
            linear_velocity: 0.,
            lateral_velocity: 0.,
            angular_velocity: 0.,
            applied_faults: "[]".to_string(),
        }
    }
}

impl SpeedObservationWrapper {
    pub fn from_rust(s: &SpeedObservation) -> Self {
        Self {
            linear_velocity: s.linear_velocity,
            lateral_velocity: s.lateral_velocity,
            angular_velocity: s.angular_velocity,
            applied_faults: serde_json::to_string(&s.applied_faults).unwrap(),
        }
    }
    pub fn to_rust(&self) -> SpeedObservation {
        SpeedObservation {
            linear_velocity: self.linear_velocity,
            lateral_velocity: self.lateral_velocity,
            angular_velocity: self.angular_velocity,
            applied_faults: serde_json::from_str(&self.applied_faults).unwrap(),
        }
    }
}

impl Default for SpeedObservationWrapper {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Debug)]
#[pyclass(get_all)]
#[pyo3(name = "GNSSObservation")]
pub struct GNSSObservationWrapper {
    pub pose: Vec3,
    pub velocity: Vec2,
    /// Applied faults in JSON format
    pub applied_faults: String,
}

#[pymethods]
impl GNSSObservationWrapper {
    #[new]
    pub fn new() -> Self {
        Self {
            pose: Vec3 {
                x: 0.,
                y: 0.,
                z: 0.,
            },
            velocity: Vec2 { x: 0., y: 0. },
            applied_faults: "[]".to_string(),
        }
    }
}

impl GNSSObservationWrapper {
    pub fn from_rust(s: &GNSSObservation) -> Self {
        Self {
            pose: Vec3 {
                x: s.pose[0],
                y: s.pose[1],
                z: s.pose[2],
            },
            velocity: Vec2 {
                x: s.velocity[0],
                y: s.velocity[1],
            },
            applied_faults: serde_json::to_string(&s.applied_faults).unwrap(),
        }
    }
    pub fn to_rust(&self) -> GNSSObservation {
        GNSSObservation {
            pose: Vector3::from_vec(vec![self.pose.x, self.pose.y, self.pose.z]),
            velocity: Vector2::from_vec(vec![self.velocity.x, self.velocity.y]),
            applied_faults: serde_json::from_str(&self.applied_faults).unwrap(),
        }
    }
}

impl Default for GNSSObservationWrapper {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Debug)]
#[pyclass(get_all)]
#[pyo3(name = "DisplacementObservation")]
pub struct DisplacementObservationWrapper {
    pub translation: Vec2,
    pub rotation: f32,
    /// Applied faults in JSON format
    pub applied_faults: String,
}

#[pymethods]
impl DisplacementObservationWrapper {
    #[new]
    pub fn new() -> Self {
        Self {
            translation: Vec2 { x: 0., y: 0. },
            rotation: 0.,
            applied_faults: "[]".to_string(),
        }
    }
}

impl DisplacementObservationWrapper {
    pub fn from_rust(s: &DisplacementObservation) -> Self {
        Self {
            translation: Vec2 {
                x: s.translation[0],
                y: s.translation[1],
            },
            rotation: s.rotation,
            applied_faults: serde_json::to_string(&s.applied_faults).unwrap(),
        }
    }
    pub fn to_rust(&self) -> DisplacementObservation {
        DisplacementObservation {
            translation: Vector2::from_vec(vec![self.translation.x, self.translation.y]),
            rotation: self.rotation,
            applied_faults: serde_json::from_str(&self.applied_faults).unwrap(),
        }
    }
}

impl Default for DisplacementObservationWrapper {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "OrientedRobotObservation")]
pub struct OrientedRobotObservationWrapper {
    /// Name of the Robot
    pub name: String,
    /// Labels of the Robot
    pub labels: Vec<String>,
    /// Pose of the Robot
    pub pose: Pose,
    /// Applied faults in JSON format
    pub applied_faults: String,
}

#[pymethods]
impl OrientedRobotObservationWrapper {
    #[new]
    pub fn new() -> Self {
        Self {
            name: "NoName".to_string(),
            labels: Vec::new(),
            pose: Pose {
                x: 0.,
                y: 0.,
                theta: 0.,
            },
            applied_faults: "[]".to_string(),
        }
    }
}

impl OrientedRobotObservationWrapper {
    pub fn from_rust(s: &OrientedRobotObservation) -> Self {
        Self {
            name: s.name.clone(),
            labels: s.labels.clone(),
            pose: Pose {
                x: s.pose[0],
                y: s.pose[1],
                theta: s.pose[2],
            },
            applied_faults: serde_json::to_string(&s.applied_faults).unwrap(),
        }
    }
    pub fn to_rust(&self) -> OrientedRobotObservation {
        OrientedRobotObservation {
            name: self.name.clone(),
            labels: self.labels.clone(),
            pose: SVector::from_vec(vec![self.pose.x, self.pose.y, self.pose.theta]),
            applied_faults: serde_json::from_str(&self.applied_faults).unwrap(),
        }
    }
}

impl Default for OrientedRobotObservationWrapper {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, EnumToString, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "SensorObservation")]
pub enum SensorObservationWrapper {
    OrientedLandmark(OrientedLandmarkObservationWrapper),
    Speed(SpeedObservationWrapper),
    GNSS(GNSSObservationWrapper),
    OrientedRobot(OrientedRobotObservationWrapper),
}

#[pymethods]
impl SensorObservationWrapper {
    #[new]
    pub fn new() -> Self {
        SensorObservationWrapper::GNSS(GNSSObservationWrapper::new())
    }

    pub fn as_oriented_landmark(&self) -> PyResult<OrientedLandmarkObservationWrapper> {
        if let Self::OrientedLandmark(o) = self {
            Ok(o.clone())
        } else {
            Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
                "Impossible to convert this observation to an OrientedLandmarkObservation",
            ))
        }
    }

    pub fn as_speed(&self) -> PyResult<SpeedObservationWrapper> {
        if let Self::Speed(o) = self {
            Ok(o.clone())
        } else {
            Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
                "Impossible to convert this observation to a SpeedObservation",
            ))
        }
    }

    pub fn as_gnss(&self) -> PyResult<GNSSObservationWrapper> {
        if let Self::GNSS(o) = self {
            Ok(o.clone())
        } else {
            Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
                "Impossible to convert this observation to an GNSSObservation",
            ))
        }
    }

    pub fn as_oriented_robot(&self) -> PyResult<OrientedRobotObservationWrapper> {
        if let Self::OrientedRobot(o) = self {
            Ok(o.clone())
        } else {
            Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
                "Impossible to convert this observation to an OrientedRobotObservation",
            ))
        }
    }

    #[getter]
    pub fn kind(&self) -> String {
        self.to_string()
    }
}

impl SensorObservationWrapper {
    pub fn from_rust(s: &SensorObservation) -> Self {
        match s {
            SensorObservation::GNSS(o) => {
                SensorObservationWrapper::GNSS(GNSSObservationWrapper::from_rust(o))
            }
            SensorObservation::Speed(o) => {
                SensorObservationWrapper::Speed(SpeedObservationWrapper::from_rust(o))
            }
            SensorObservation::OrientedLandmark(o) => SensorObservationWrapper::OrientedLandmark(
                OrientedLandmarkObservationWrapper::from_rust(o),
            ),
            SensorObservation::OrientedRobot(o) => SensorObservationWrapper::OrientedRobot(
                OrientedRobotObservationWrapper::from_rust(o),
            ),
            _ => {
                panic!("ExternalObservation cannot be converted to SensorObservationWrapper yet");
            }
        }
    }
    pub fn to_rust(&self) -> SensorObservation {
        match self {
            SensorObservationWrapper::GNSS(o) => SensorObservation::GNSS(o.to_rust()),
            SensorObservationWrapper::Speed(o) => SensorObservation::Speed(o.to_rust()),
            SensorObservationWrapper::OrientedLandmark(o) => {
                SensorObservation::OrientedLandmark(o.to_rust())
            }
            SensorObservationWrapper::OrientedRobot(o) => {
                SensorObservation::OrientedRobot(o.to_rust())
            }
        }
    }
}

impl Default for SensorObservationWrapper {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Debug)]
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

impl Default for ObservationWrapper {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, EnumToString, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "Command")]
pub enum CommandWrapper {
    Unicycle(UnicycleCommandWrapper),
    Holonomic(HolonomicCommandWrapper),
}

#[pymethods]
impl CommandWrapper {
    #[new]
    pub fn new() -> CommandWrapper {
        Self::Unicycle(UnicycleCommandWrapper::new())
    }

    pub fn as_unicycle_command(&self) -> PyResult<UnicycleCommandWrapper> {
        if let Self::Unicycle(o) = self {
            Ok(o.clone())
        } else {
            Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
                "Impossible to convert this command to a UnicycleCommand",
            ))
        }
    }

    pub fn as_holonomic_command(&self) -> PyResult<HolonomicCommandWrapper> {
        if let Self::Holonomic(o) = self {
            Ok(o.clone())
        } else {
            Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
                "Impossible to convert this command to a HolonomicCommand",
            ))
        }
    }

    #[getter]
    pub fn kind(&self) -> String {
        self.to_string()
    }

    #[staticmethod]
    pub fn from_unicycle_command(cmd: UnicycleCommandWrapper) -> CommandWrapper {
        Self::Unicycle(cmd)
    }

    #[staticmethod]
    pub fn from_holonomic_command(cmd: HolonomicCommandWrapper) -> CommandWrapper {
        Self::Holonomic(cmd)
    }
}

impl Default for CommandWrapper {
    fn default() -> Self {
        Self::new()
    }
}

impl CommandWrapper {
    pub fn from_rust(s: &Command) -> Self {
        match s {
            Command::Unicycle(cmd) => {
                CommandWrapper::Unicycle(UnicycleCommandWrapper::from_rust(cmd))
            }
            Command::Holonomic(cmd) => {
                CommandWrapper::Holonomic(HolonomicCommandWrapper::from_rust(cmd))
            }
        }
    }
    pub fn to_rust(&self) -> Command {
        match self {
            CommandWrapper::Unicycle(cmd) => {
                Command::Unicycle(UnicycleCommandWrapper::to_rust(cmd))
            }
            CommandWrapper::Holonomic(cmd) => {
                Command::Holonomic(HolonomicCommandWrapper::to_rust(cmd))
            }
        }
    }
}

#[derive(Clone, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "UnicycleCommand")]
pub struct UnicycleCommandWrapper {
    /// Left wheel speed.
    pub left_wheel_speed: f32,
    /// Right wheel speed.
    pub right_wheel_speed: f32,
}

#[pymethods]
impl UnicycleCommandWrapper {
    #[new]
    pub fn new() -> UnicycleCommandWrapper {
        Self {
            left_wheel_speed: 0.,
            right_wheel_speed: 0.,
        }
    }
}

impl Default for UnicycleCommandWrapper {
    fn default() -> Self {
        Self::new()
    }
}

impl UnicycleCommandWrapper {
    pub fn from_rust(s: &UnicycleCommand) -> Self {
        Self {
            left_wheel_speed: s.left_wheel_speed,
            right_wheel_speed: s.right_wheel_speed,
        }
    }
    pub fn to_rust(&self) -> UnicycleCommand {
        UnicycleCommand {
            left_wheel_speed: self.left_wheel_speed,
            right_wheel_speed: self.right_wheel_speed,
        }
    }
}

#[derive(Clone, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "HolonomicCommand")]
pub struct HolonomicCommandWrapper {
    pub longitudinal_velocity: f32,
    pub lateral_velocity: f32,
    pub angular_velocity: f32,
}

#[pymethods]
impl HolonomicCommandWrapper {
    #[new]
    pub fn new() -> HolonomicCommandWrapper {
        Self {
            longitudinal_velocity: 0.,
            lateral_velocity: 0.,
            angular_velocity: 0.,
        }
    }
}

impl Default for HolonomicCommandWrapper {
    fn default() -> Self {
        Self::new()
    }
}

impl HolonomicCommandWrapper {
    pub fn from_rust(s: &HolonomicCommand) -> Self {
        Self {
            longitudinal_velocity: s.longitudinal_velocity,
            lateral_velocity: s.lateral_velocity,
            angular_velocity: s.angular_velocity,
        }
    }
    pub fn to_rust(&self) -> HolonomicCommand {
        HolonomicCommand {
            longitudinal_velocity: self.longitudinal_velocity,
            lateral_velocity: self.lateral_velocity,
            angular_velocity: self.angular_velocity,
        }
    }
}

#[derive(Clone, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "Envelope")]
pub struct EnvelopeWrapper {
    pub msg_from: String,
    pub message: MessageTypes,
    pub timestamp: f32,
}

#[derive(Debug, Clone)]
#[pyclass]
#[pyo3(name = "Node")]
pub struct NodeWrapper {
    node: Arc<Node>,
}

#[pymethods]
impl NodeWrapper {
    pub fn name(&self) -> String {
        self.node.name()
    }

    #[pyo3(signature = (to, message, time, flags=Vec::new()))]
    #[warn(clippy::useless_conversion)]
    pub fn send_message(
        &self,
        to: String,
        message: MessageTypes,
        time: f32,
        flags: Vec<MessageFlag>,
    ) -> PyResult<()> {
        if let Some(network) = &self.node.network() {
            let msg = match message {
                MessageTypes::String(s) => serde_json::to_value(s),
                MessageTypes::GoTo(m) => serde_json::to_value(m),
                MessageTypes::SensorTrigger(m) => serde_json::to_value(m),
            }
            .map_err(|e| PyErr::new::<PyTypeError, _>(format!("Conversion failed: {}", e)))?;
            let key = PathKey::from_str(to.as_str()).unwrap();
            let msg = Envelope {
                from: self.node.name(),
                message: msg,
                timestamp: time,
                message_flags: flags,
            };
            network.write().unwrap().send_to(key, msg, time);
            Ok(())
        } else {
            Err(PyErr::new::<PyTypeError, _>("No network on this node"))
        }
    }

    pub fn subscribe(&self, topics: Vec<String>) -> PyResult<MultiClientWrapper> {
        if let Some(network) = &self.node.network() {
            let keys = topics
                .iter()
                .map(|t| PathKey::from_str(t.as_str()).unwrap())
                .collect::<Vec<PathKey>>();
            let client = network.write().unwrap().subscribe_to(&keys, None);
            Ok(MultiClientWrapper::from_rust(client))
        } else {
            Err(PyErr::new::<PyTypeError, _>("No network on this node"))
        }
    }

    pub fn make_channel(&self, channel_name: String) -> PyResult<()> {
        if let Some(network) = &self.node.network() {
            let key = PathKey::from_str(channel_name.as_str()).unwrap();
            network.write().unwrap().make_channel(key);
            Ok(())
        } else {
            Err(PyErr::new::<PyTypeError, _>("No network on this node"))
        }
    }
}

impl NodeWrapper {
    pub fn from_rust(n: &Node) -> Self {
        Self {
            // I did not find another solution.
            // Relatively safe as Nodes lives very long, almost all the time
            // For API usage
            #[allow(clippy::borrow_deref_ref)]
            node: unsafe { Arc::from_raw(&*n) },
        }
    }
}

#[derive(Debug)]
#[pyclass]
#[pyo3(name = "Client")]
pub struct MultiClientWrapper {
    client: SimbaBrokerMultiClient,
}

#[pymethods]
impl MultiClientWrapper {
    pub fn subscribe(&mut self, key: String) {
        let key = PathKey::from_str(key.as_str()).unwrap();
        self.client.subscribe(&key);
    }

    pub fn subscribe_instantaneous(&mut self, key: String) {
        let key = PathKey::from_str(key.as_str()).unwrap();
        self.client.subscribe_instantaneous(&key);
    }

    pub fn send(
        &self,
        to: String,
        message: MessageTypes,
        time: f32,
        flags: Vec<MessageFlag>,
    ) -> PyResult<()> {
        let msg = match message {
            MessageTypes::String(s) => serde_json::to_value(s),
            MessageTypes::GoTo(m) => serde_json::to_value(m),
            MessageTypes::SensorTrigger(m) => serde_json::to_value(m),
        }
        .map_err(|e| PyErr::new::<PyTypeError, _>(format!("Conversion failed: {}", e)))?;
        let key = PathKey::from_str(to.as_str()).unwrap();
        let msg = Envelope {
            from: self.client.node_id().to_string(),
            message: msg,
            timestamp: time,
            message_flags: flags,
        };
        self.client.send(&key, msg, time);
        Ok(())
    }

    pub fn try_receive(&self, time: f32) -> Option<(String, EnvelopeWrapper)> {
        if let Some((path, envelope)) = self.client.try_receive(time) {
            let msg = match serde_json::from_value(envelope.message.clone()) {
                Err(_) => MessageTypes::String(serde_json::to_string(&envelope.message).unwrap()),
                Ok(m) => m,
            };
            Some((
                path.to_string(),
                EnvelopeWrapper {
                    msg_from: envelope.from,
                    message: msg,
                    timestamp: envelope.timestamp,
                },
            ))
        } else {
            None
        }
    }

    pub fn next_message_time(&self) -> Option<f32> {
        self.client.next_message_time()
    }

    pub fn subscribed_keys(&self) -> Vec<String> {
        self.client
            .subscribed_keys()
            .iter()
            .map(|k| k.to_string())
            .collect()
    }

    pub fn node_id(&self) -> String {
        self.client.node_id().to_string()
    }
}

impl MultiClientWrapper {
    pub fn from_rust(client: SimbaBrokerMultiClient) -> Self {
        Self { client }
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
        _initial_time: f32,
    ) -> StateEstimatorWrapper {
        panic!("The given PluginAPI does not provide a state estimator");
    }

    /// Return the [`Controller`](crate::controllers::Controller) to be used by the
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
    /// Returns the [`Controller`](crate::controllers::Controller) to use.
    pub fn get_controller(
        &self,
        _config: Py<PyAny>,
        _global_config: Py<PyAny>,
        _initial_time: f32,
    ) -> ControllerWrapper {
        panic!("The given PluginAPI does not provide a controller");
    }

    /// Return the [`Navigator`](crate::navigators::Navigator) to be used by the
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
    /// Returns the [`Navigator`](crate::navigators::Navigator) to use.
    pub fn get_navigator(
        &self,
        _config: Py<PyAny>,
        _global_config: Py<PyAny>,
        _initial_time: f32,
    ) -> NavigatorWrapper {
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
    pub fn get_physics(
        &self,
        _config: Py<PyAny>,
        _global_config: Py<PyAny>,
        _initial_time: f32,
    ) -> PhysicsWrapper {
        panic!("The given PluginAPI does not provide physics");
    }
}

impl Default for PluginAPIWrapper {
    fn default() -> Self {
        Self::new()
    }
}

#[pyclass]
#[pyo3(name = "Simulator")]
pub struct SimulatorWrapper {
    simulator: AsyncSimulator,
    python_api: Option<Arc<dyn PluginAPI>>,
}

#[pymethods]
impl SimulatorWrapper {
    #[staticmethod]
    #[pyo3(signature = (config_path, plugin_api=None))]
    pub fn from_config(
        config_path: String,
        plugin_api: Option<Py<PyAny>>,
    ) -> PyResult<SimulatorWrapper> {
        Simulator::init_environment();

        let python_api = plugin_api.map(|api| Arc::new(PythonAPI::new(api)) as Arc<dyn PluginAPI>);

        let simulator =
            AsyncSimulator::from_config_path(&config_path, &python_api).map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!(
                    "Failed to create simulator from config: {}",
                    e.detailed_error()
                ))
            })?;
        Ok(SimulatorWrapper {
            simulator,
            python_api,
        })
    }

    pub fn run(&mut self) {
        self.simulator.run(&self.python_api, None, false);
        self.simulator.compute_results();
        self.simulator.stop();
    }
}

#[cfg(feature = "gui")]
#[pyfunction]
#[pyo3(signature = (default_config_path=None, plugin_api=None, load_results=false))]
pub fn run_gui(
    py: Python,
    default_config_path: Option<String>,
    plugin_api: Option<Py<PyAny>>,
    load_results: bool,
) {
    use std::{sync::RwLock, thread};

    use crate::gui;
    let async_plugin_api = plugin_api.as_ref().map(|_| Arc::new(PluginAsyncAPI::new()));
    let mut python_api = plugin_api.map(PythonAPI::new);

    let api_client = async_plugin_api.as_ref().map(|api| api.get_client());

    let running = Arc::new(RwLock::new(true));
    let local_running = running.clone();

    let thread_handle = thread::spawn(move || {
        while *running.read().unwrap() {
            if let Some(api_client) = &api_client {
                let python_api = python_api.as_mut().unwrap();
                // TODO: Multiple wait can be optimized the same way than AsyncAPI runner (or maybe not as it's Python)
                api_client.get_state_estimator.try_recv_closure(|request| {
                    python_api.get_state_estimator(
                        &request.config,
                        &request.global_config,
                        &request.va_factory,
                        &request.network,
                        0.,
                    )
                });
                api_client.get_controller.try_recv_closure(|request| {
                    python_api.get_controller(
                        &request.config,
                        &request.global_config,
                        &request.va_factory,
                        &request.network,
                        0.,
                    )
                });
                api_client.get_navigator.try_recv_closure(|request| {
                    python_api.get_navigator(
                        &request.config,
                        &request.global_config,
                        &request.va_factory,
                        &request.network,
                        0.,
                    )
                });
                api_client.get_physics.try_recv_closure(|request| {
                    python_api.get_physics(
                        &request.config,
                        &request.global_config,
                        &request.va_factory,
                        &request.network,
                        0.,
                    )
                });
                python_api.check_requests();
            }
        }
    });

    // egui needs to be run in the main thread. However, Python get the GIL. So we need to free the GIL so that the thread spawn before can call Python functions.
    // allow_threads reacquire the GIL when it exits the function. At this point, we are sure that the other thread is finished.
    py.detach(|| {
        gui::run_gui(
            match &default_config_path {
                Some(path_str) => {
                    let p = Path::new(path_str);
                    Some(unsafe { std::mem::transmute::<&Path, &'static Path>(p) })
                }
                None => None,
            },
            async_plugin_api.map(|api| api as Arc<dyn PluginAPI>),
            load_results,
        );
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
