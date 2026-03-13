#![allow(clippy::useless_conversion)]
//! Python wrapper layer for Simba core types exposed through PyO3.
//!
//! This module contains thin wrapper types used to:
//! - expose Rust data structures and APIs to Python,
//! - convert values between Python-facing wrappers and internal Rust models,
//! - provide Python entry points for simulation and networking.
//!
//! Most wrappers follow the same pattern:
//! - a `#[pyclass]` with Python-friendly fields,
//! - `from_rust` and `to_rust` conversion helpers,
//! - a default constructor for easy Python-side instantiation.

use std::{
    collections::BTreeMap,
    str::FromStr,
    sync::{Arc, RwLock, Weak},
};

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
        network::{Envelope, MessageFlag, Network},
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
/// Python wrapper around controller tracking error values.
pub struct ControllerErrorWrapper {
    /// Lateral error (y axis).
    pub lateral: f32,
    /// Longitudinal error (x axis).
    pub longitudinal: f32,
    /// Orientation error.
    pub theta: f32,
    /// Velocity error.
    pub velocity: f32,
}

#[pymethods]
impl ControllerErrorWrapper {
    /// Create a zero-initialized controller error.
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
    /// Convert from the Rust [`ControllerError`] type.
    pub fn from_rust(ce: &ControllerError) -> Self {
        Self {
            lateral: ce.lateral,
            theta: ce.theta,
            velocity: ce.velocity,
            longitudinal: ce.longitudinal,
        }
    }

    /// Convert this wrapper to the Rust [`ControllerError`] type.
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
/// 2D pose with heading angle used by Python APIs.
pub struct Pose {
    /// Position on x axis.
    pub x: f32,
    /// Position on y axis.
    pub y: f32,
    /// Heading angle (radians).
    pub theta: f32,
}

#[pyclass(get_all, set_all)]
#[pyo3(name = "Vec2")]
#[derive(Clone, Debug)]
/// Generic 2D vector used in Python wrappers.
pub struct Vec2 {
    /// x component.
    pub x: f32,
    /// y component.
    pub y: f32,
}

#[pyclass(get_all, set_all)]
#[pyo3(name = "Vec3")]
#[derive(Clone, Debug)]
/// Generic 3D vector used in Python wrappers.
pub struct Vec3 {
    /// x component.
    pub x: f32,
    /// y component.
    pub y: f32,
    /// z component.
    pub z: f32,
}

#[derive(Clone, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "State")]
/// Python wrapper around a robot [`State`].
pub struct StateWrapper {
    /// Position and orientation of the robot
    pub pose: Pose,
    /// Linear velocity.
    pub velocity: Vec3,
}

#[pymethods]
impl StateWrapper {
    /// Create a zero-initialized state.
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
    /// Convert from the Rust [`State`] type.
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
    /// Convert this wrapper to the Rust [`State`] type.
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
/// Python wrapper around a full [`WorldState`].
pub struct WorldStateWrapper {
    /// Optional state of the ego robot.
    pub ego: Option<StateWrapper>,
    /// States of named dynamic objects (other nodes).
    pub objects: BTreeMap<String, StateWrapper>,
    /// States of known landmarks indexed by id.
    pub landmarks: BTreeMap<i32, StateWrapper>,
    /// Optional occupancy grid map.
    pub occupancy_grid: Option<OccupancyGridWrapper>,
}

#[pymethods]
impl WorldStateWrapper {
    /// Create an empty world state.
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
    /// Convert from the Rust [`WorldState`] type.
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
    /// Convert this wrapper to the Rust [`WorldState`] type.
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
/// Python wrapper around [`OccupancyGrid`].
pub struct OccupancyGridWrapper {
    grid: OccupancyGrid,
}

#[pymethods]
impl OccupancyGridWrapper {
    /// Create a grid centered at `center` with fixed cell geometry.
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

    /// Get a cell value by `(row, col)` index.
    pub fn get_idx(&self, row: usize, col: usize) -> Option<f32> {
        self.grid.get_idx(row, col).cloned()
    }

    /// Set a cell value by `(row, col)` index.
    ///
    /// Returns `true` if the index exists, `false` otherwise.
    pub fn set_idx(&mut self, row: usize, col: usize, value: f32) -> bool {
        if let Some(v) = self.grid.get_idx_mut(row, col) {
            *v = value;
            true
        } else {
            false
        }
    }

    /// Get a cell value using world coordinates `[x, y]`.
    pub fn get_pos(&self, position: [f32; 2]) -> Option<f32> {
        self.grid.get_pos(Vector2::from(position)).cloned()
    }

    /// Set a cell value using world coordinates `[x, y]`.
    ///
    /// Returns `true` if the position maps to a valid cell, `false` otherwise.
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
    /// Clone a Rust [`OccupancyGrid`] into its Python wrapper.
    pub fn from_rust(s: &OccupancyGrid) -> Self {
        Self { grid: s.clone() }
    }

    /// Clone this wrapper back to a Rust [`OccupancyGrid`].
    pub fn to_rust(&self) -> OccupancyGrid {
        self.grid.clone()
    }
}

#[derive(Clone, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "OrientedLandmarkObservation")]
/// Python wrapper around an oriented landmark sensor observation.
pub struct OrientedLandmarkObservationWrapper {
    /// Id of the landmark
    pub id: i32,
    /// Labels of the landmark
    pub labels: Vec<String>,
    /// Relative pose of the landmark
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
    /// Create a default-oriented landmark observation.
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
    /// Convert from the Rust [`OrientedLandmarkObservation`] type.
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
    /// Convert this wrapper to the Rust [`OrientedLandmarkObservation`] type.
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
/// Python wrapper around speed sensor observation values.
pub struct SpeedObservationWrapper {
    /// Linear velocity along robot forward axis.
    pub linear_velocity: f32,
    /// Linear velocity along robot lateral axis.
    pub lateral_velocity: f32,
    /// Angular velocity around vertical axis.
    pub angular_velocity: f32,
    /// Applied faults in JSON format
    pub applied_faults: String,
}

#[pymethods]
impl SpeedObservationWrapper {
    /// Create a default speed observation.
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
    /// Convert from the Rust [`SpeedObservation`] type.
    pub fn from_rust(s: &SpeedObservation) -> Self {
        Self {
            linear_velocity: s.linear_velocity,
            lateral_velocity: s.lateral_velocity,
            angular_velocity: s.angular_velocity,
            applied_faults: serde_json::to_string(&s.applied_faults).unwrap(),
        }
    }
    /// Convert this wrapper to the Rust [`SpeedObservation`] type.
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
/// Python wrapper around GNSS-like pose and velocity observation.
pub struct GNSSObservationWrapper {
    /// Pose measurement `[x, y, theta]`.
    pub pose: Vec3,
    /// Velocity measurement `[vx, vy]`.
    pub velocity: Vec2,
    /// Applied faults in JSON format
    pub applied_faults: String,
}

#[pymethods]
impl GNSSObservationWrapper {
    /// Create a default GNSS observation.
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
    /// Convert from the Rust [`GNSSObservation`] type.
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
    /// Convert this wrapper to the Rust [`GNSSObservation`] type.
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
/// Python wrapper around incremental displacement measurement.
pub struct DisplacementObservationWrapper {
    /// Translation increment on the xy plane.
    pub translation: Vec2,
    /// Rotation increment (radians).
    pub rotation: f32,
    /// Applied faults in JSON format
    pub applied_faults: String,
}

#[pymethods]
impl DisplacementObservationWrapper {
    /// Create a default displacement observation.
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
    /// Convert from the Rust [`DisplacementObservation`] type.
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
    /// Convert this wrapper to the Rust [`DisplacementObservation`] type.
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
/// Python wrapper around oriented robot observation.
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
    /// Create a default-oriented robot observation.
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
    /// Convert from the Rust [`OrientedRobotObservation`] type.
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
    /// Convert this wrapper to the Rust [`OrientedRobotObservation`] type.
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
/// Tagged union of supported sensor observations exposed to Python.
pub enum SensorObservationWrapper {
    /// Observation from an oriented-landmark sensor.
    OrientedLandmark(OrientedLandmarkObservationWrapper),
    /// Observation from a speed sensor.
    Speed(SpeedObservationWrapper),
    /// Observation from a GNSS-like sensor.
    GNSS(GNSSObservationWrapper),
    /// Observation from an oriented-robot sensor.
    OrientedRobot(OrientedRobotObservationWrapper),
    /// Observation from a displacement sensor.
    Displacement(DisplacementObservationWrapper),
}

#[pymethods]
impl SensorObservationWrapper {
    #[new]
    /// Create a default observation (GNSS for placeholder purposes).
    pub fn new() -> Self {
        SensorObservationWrapper::GNSS(GNSSObservationWrapper::new())
    }

    /// Try to convert the observation to an [`OrientedLandmarkObservationWrapper`].
    pub fn as_oriented_landmark(&self) -> PyResult<OrientedLandmarkObservationWrapper> {
        if let Self::OrientedLandmark(o) = self {
            Ok(o.clone())
        } else {
            Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
                "Impossible to convert this observation to an OrientedLandmarkObservation",
            ))
        }
    }

    /// Try to convert the observation to a [`SpeedObservationWrapper`].
    pub fn as_speed(&self) -> PyResult<SpeedObservationWrapper> {
        if let Self::Speed(o) = self {
            Ok(o.clone())
        } else {
            Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
                "Impossible to convert this observation to a SpeedObservation",
            ))
        }
    }

    /// Try to convert the observation to a [`GNSSObservationWrapper`].
    pub fn as_gnss(&self) -> PyResult<GNSSObservationWrapper> {
        if let Self::GNSS(o) = self {
            Ok(o.clone())
        } else {
            Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
                "Impossible to convert this observation to an GNSSObservation",
            ))
        }
    }

    /// Try to convert the observation to an [`OrientedRobotObservationWrapper`].
    pub fn as_oriented_robot(&self) -> PyResult<OrientedRobotObservationWrapper> {
        if let Self::OrientedRobot(o) = self {
            Ok(o.clone())
        } else {
            Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
                "Impossible to convert this observation to an OrientedRobotObservation",
            ))
        }
    }

    /// Try to convert the observation to a [`DisplacementObservationWrapper`].
    pub fn as_displacement(&self) -> PyResult<DisplacementObservationWrapper> {
        if let Self::Displacement(o) = self {
            Ok(o.clone())
        } else {
            Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
                "Impossible to convert this observation to a DisplacementObservation",
            ))
        }
    }

    #[getter]
    /// Return the variant name as a string.
    pub fn kind(&self) -> String {
        self.to_string()
    }
}

impl SensorObservationWrapper {
    /// Convert from the Rust [`SensorObservation`] type.
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
            SensorObservation::Displacement(o) => SensorObservationWrapper::Displacement(
                DisplacementObservationWrapper::from_rust(o),
            ),
            SensorObservation::Scan(_) => {
                panic!("ScanObservation cannot be converted to SensorObservationWrapper yet");
            }
            SensorObservation::External(_) => {
                panic!("ExternalObservation cannot be converted to SensorObservationWrapper yet");
            }
        }
    }
    /// Convert this wrapper to the Rust [`SensorObservation`] type.
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
            SensorObservationWrapper::Displacement(o) => {
                SensorObservation::Displacement(o.to_rust())
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
/// Generic observation wrapper carrying metadata and observation from one sensor.
pub struct ObservationWrapper {
    /// Sensor identifier.
    pub sensor_name: String,
    /// Name of the node that observed.
    pub observer: String,
    /// Observation timestamp.
    pub time: f32,
    /// Concrete sensor observation payload.
    pub sensor_observation: SensorObservationWrapper,
}

#[pymethods]
impl ObservationWrapper {
    /// Create a default observation with placeholder metadata.
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
    /// Convert from the Rust [`Observation`] type.
    pub fn from_rust(s: &Observation) -> Self {
        Self {
            sensor_name: s.sensor_name.clone(),
            observer: s.observer.clone(),
            time: s.time,
            sensor_observation: SensorObservationWrapper::from_rust(&s.sensor_observation),
        }
    }
    /// Convert this wrapper to the Rust [`Observation`] type.
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

/// Wrapper around the [`Command`] to be used in Python.
/// It can be either a [`UnicycleCommandWrapper`] or a [`HolonomicCommandWrapper`].
#[derive(Clone, EnumToString, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "Command")]
pub enum CommandWrapper {
    /// Command for unicycle robot models, containing the left and right wheel speeds.
    Unicycle(UnicycleCommandWrapper),
    /// Command for holonomic robot models, containing the longitudinal, lateral and angular velocities.
    Holonomic(HolonomicCommandWrapper),
}

#[pymethods]
impl CommandWrapper {
    /// Creates a new [`CommandWrapper`] with a default [`UnicycleCommandWrapper`]. It can be changed later using the setter or the static methods.
    #[new]
    pub fn new() -> CommandWrapper {
        Self::Unicycle(UnicycleCommandWrapper::new())
    }

    /// Tries to convert the command to a [`UnicycleCommandWrapper`]. It returns an error if the command is not a [`UnicycleCommandWrapper`].
    pub fn as_unicycle_command(&self) -> PyResult<UnicycleCommandWrapper> {
        if let Self::Unicycle(o) = self {
            Ok(o.clone())
        } else {
            Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
                "Impossible to convert this command to a UnicycleCommand",
            ))
        }
    }

    /// Tries to convert the command to a [`HolonomicCommandWrapper`]. It returns an error if the command is not a [`HolonomicCommandWrapper`].
    pub fn as_holonomic_command(&self) -> PyResult<HolonomicCommandWrapper> {
        if let Self::Holonomic(o) = self {
            Ok(o.clone())
        } else {
            Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
                "Impossible to convert this command to a HolonomicCommand",
            ))
        }
    }

    /// Returns the kind of the command as a string. It can be either "Unicycle" or "Holonomic".
    #[getter]
    pub fn kind(&self) -> String {
        self.to_string()
    }

    /// Creates a new [`CommandWrapper`] from a [`UnicycleCommandWrapper`].
    #[staticmethod]
    pub fn from_unicycle_command(cmd: UnicycleCommandWrapper) -> CommandWrapper {
        Self::Unicycle(cmd)
    }

    /// Creates a new [`CommandWrapper`] from a [`HolonomicCommandWrapper`].
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
    /// Creates a [`CommandWrapper`] from its Rust counterpart [`Command`]
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

    /// Converts the Python wrapper back to its Rust counterpart [`Command`]
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

/// Wrapper around the [`UnicycleCommand`] to be used in Python.
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
    /// Creates a new [`UnicycleCommandWrapper`] with all wheel speeds set to 0.
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
    /// Creates an [`UnicycleCommandWrapper`] from its Rust counterpart [`UnicycleCommand`]
    pub fn from_rust(s: &UnicycleCommand) -> Self {
        Self {
            left_wheel_speed: s.left_wheel_speed,
            right_wheel_speed: s.right_wheel_speed,
        }
    }

    /// Converts the Python wrapper back to its Rust counterpart [`UnicycleCommand`]
    pub fn to_rust(&self) -> UnicycleCommand {
        UnicycleCommand {
            left_wheel_speed: self.left_wheel_speed,
            right_wheel_speed: self.right_wheel_speed,
        }
    }
}

/// Wrapper around the [`HolonomicCommand`] to be used in Python.
#[derive(Clone, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "HolonomicCommand")]
pub struct HolonomicCommandWrapper {
    /// Longitudinal velocity (following x axis of the robot).
    pub longitudinal_velocity: f32,
    /// Lateral velocity (following y axis of the robot).
    pub lateral_velocity: f32,
    /// Angular velocity (radians per second).
    pub angular_velocity: f32,
}

#[pymethods]
impl HolonomicCommandWrapper {
    /// Creates a new [`HolonomicCommandWrapper`] with all velocities set to 0.
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
    /// Creates an [`HolonomicCommandWrapper`] from its Rust counterpart [`HolonomicCommand`]
    pub fn from_rust(s: &HolonomicCommand) -> Self {
        Self {
            longitudinal_velocity: s.longitudinal_velocity,
            lateral_velocity: s.lateral_velocity,
            angular_velocity: s.angular_velocity,
        }
    }

    /// Converts the Python wrapper back to its Rust counterpart [`HolonomicCommand`]
    pub fn to_rust(&self) -> HolonomicCommand {
        HolonomicCommand {
            longitudinal_velocity: self.longitudinal_velocity,
            lateral_velocity: self.lateral_velocity,
            angular_velocity: self.angular_velocity,
        }
    }
}

/// Wrapper for the [`Envelope`]
#[derive(Clone, Debug)]
#[pyclass(get_all, set_all)]
#[pyo3(name = "Envelope")]
pub struct EnvelopeWrapper {
    /// Expeditor
    pub msg_from: String,
    /// Message sent, see [`MessageTypes`] for supported messages
    pub message: MessageTypes,
    /// Time of the message
    pub timestamp: f32,
}

/// Wrapper around the [`Node`] to be used in Python.
/// It provides methods around the [`Network`].
#[derive(Debug, Clone)]
#[pyclass]
#[pyo3(name = "Node")]
pub struct NodeWrapper {
    name: String,
    network: Option<Weak<RwLock<Network>>>,
}

#[pymethods]
impl NodeWrapper {
    /// Get the unique name of the node.
    pub fn name(&self) -> String {
        self.name.clone()
    }

    /// Send a message to the given channel using the node [`Network`]. It returns an error if the node is not connected to any network.
    /// 
    /// Should only be used for one-shot messages as it creates a new client every times. For more persistent communication, it is better to use a [`MultiClientWrapper`] obtained with the [`subscribe`](Self::subscribe) method.
    /// 
    /// # Arguments
    /// * `to` - Channel key to send the message to. It can be a relative key (e.g. "channel1") or an absolute key (e.g. "/my_channels/channel1").
    /// * `message` - Message to send. See [`MessageTypes`] for the supported message types in Python.
    /// * `time` - Time at which the message is sent. It should be the current time of the simulator or in the future.
    /// * `flags` - Message flags. See [`MessageFlag`] for the supported flags.
    #[pyo3(signature = (to, message, time, flags=Vec::new()))]
    #[warn(clippy::useless_conversion)]
    pub fn send_message(
        &self,
        to: String,
        message: MessageTypes,
        time: f32,
        flags: Vec<MessageFlag>,
    ) -> PyResult<()> {
        if let Some(network) = self.network.as_ref().and_then(|n| n.upgrade()) {
            let msg = match message {
                MessageTypes::String(s) => serde_json::to_value(s),
                MessageTypes::GoTo(m) => serde_json::to_value(m),
                MessageTypes::SensorTrigger(m) => serde_json::to_value(m),
            }
            .map_err(|e| PyErr::new::<PyTypeError, _>(format!("Conversion failed: {}", e)))?;
            let key = PathKey::from_str(to.as_str()).unwrap();
            let msg = Envelope {
                from: self.name.clone(),
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

    /// Subscribe to a channel using the node [`Network`] and get a [`MultiClientWrapper`] to send and receive messages on this channel.
    /// It returns an error if the node is not connected to any network.
    pub fn subscribe(&self, channels: Vec<String>) -> PyResult<MultiClientWrapper> {
        if let Some(network) = &self.network.as_ref().and_then(|n| n.upgrade()) {
            let keys = channels
                .iter()
                .map(|t| PathKey::from_str(t.as_str()).unwrap())
                .collect::<Vec<PathKey>>();
            let client = network.write().unwrap().subscribe_to(&keys, None);
            Ok(MultiClientWrapper::from_rust(client))
        } else {
            Err(PyErr::new::<PyTypeError, _>("No network on this node"))
        }
    }

    /// Create a new channel in the network using [`Network::make_channel`]. It returns an error if the node is not connected to any network.
    pub fn make_channel(&self, channel_name: String) -> PyResult<()> {
        if let Some(network) = &self.network.as_ref().and_then(|n| n.upgrade()) {
            let key = PathKey::from_str(channel_name.as_str()).unwrap();
            network.write().unwrap().make_channel(key);
            Ok(())
        } else {
            Err(PyErr::new::<PyTypeError, _>("No network on this node"))
        }
    }
}

impl NodeWrapper {
    /// Create a new NodeWrapper from a Node.
    pub fn from_rust(n: &Node) -> Self {
        Self {
            name: n.name(),
            network: n.network().as_ref().map(Arc::downgrade),
        }
    }
}

/// Wrapper around the [`SimbaBrokerMultiClient`] to be used in Python.
/// It provides methods to subscribe to channels, send messages and receive messages.
#[derive(Debug)]
#[pyclass]
#[pyo3(name = "Client")]
pub struct MultiClientWrapper {
    client: SimbaBrokerMultiClient,
}

#[pymethods]
impl MultiClientWrapper {
    /// Subscribe to a channel. The client will receive messages sent to this channel.
    /// 
    /// If the channel key is relative (does not start with a "/"), the subscription will be made to the channel using
    /// the node id as prefix. For example, if the node id is "node1" and the channel key is "channel1", the subscription will be made to "/simba/nodes/node1/channel1".
    /// 
    /// # Arguments
    /// * `key` - Channel key to subscribe to. It can be a relative key (e.g. "channel1") or an absolute key (e.g. "/my_channels/channel1").
    pub fn subscribe(&mut self, key: String) {
        let key = PathKey::from_str(key.as_str()).unwrap();
        self.client.subscribe(&key);
    }

    /// Similar to [`subscribe`](Self::subscribe) but it overrides the network delays.
    /// To be used for meta messages, or virtual messages that would not exists in a real world
    pub fn subscribe_instantaneous(&mut self, key: String) {
        let key = PathKey::from_str(key.as_str()).unwrap();
        self.client.subscribe_instantaneous(&key);
    }

    /// Send a message to the given channel. The message will be received by all the clients subscribed to this channel which are in range.
    /// 
    /// # Arguments
    /// * `to` - Channel key to send the message to. It can be a relative key (e.g. "channel1") or an absolute key (e.g. "/my_channels/channel1").
    /// * `message` - Message to send. See [`MessageTypes`] for the supported message types in Python.
    /// * `time` - Time at which the message is sent. It should be the current time of the simulator or in the future.
    /// * `flags` - Message flags. See [`MessageFlag`] for the supported flags.
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

    /// Try to receive a message from the subscribed channels. It returns the first message received or None if no message is received within the given time.
    /// 
    /// It can return messages older than the given `time`.
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

    /// Get the time of the next message to be received. It returns None if no message is expected to be received in the future.
    pub fn next_message_time(&self) -> Option<f32> {
        self.client.next_message_time()
    }

    /// Get the list of channels the client is subscribed to.
    pub fn subscribed_keys(&self) -> Vec<String> {
        self.client
            .subscribed_keys()
            .iter()
            .map(|k| k.to_string())
            .collect()
    }

    /// Get the node id of the client. It is the same as the name of the node the client is attached to.
    pub fn node_id(&self) -> String {
        self.client.node_id().to_string()
    }
}

impl MultiClientWrapper {
    /// Create a new MultiClientWrapper from a SimbaBrokerMultiClient.
    pub fn from_rust(client: SimbaBrokerMultiClient) -> Self {
        Self { client }
    }
}

/// Trait to be implemented by the Python API provided by the user. It provides methods to get the state estimator, controller, navigator and physics to be used by the simulator.
/// The user can implement this trait in Python and provide it to the simulator to use custom implementations of these components.
/// 
/// # Example
/// ```python
/// import simba
/// import json
/// 
/// class Controller(simba.Controller):
///     # implementation
/// 
/// class SimulatorAPI(simba.PluginAPI):
///     def get_controller(self, config: dict, global_config: dict, initial_time: float):
///         config = json.loads(config)
///         print(f"Config received by python: {type(config)} {config}")
///         return Controller(config, initial_time)
/// 
/// def main():
///     simulator_api = SimulatorAPI()
/// 
///     simulator = simba.Simulator.from_config(
///         "config/config_controller.yaml", simulator_api
///     )
///     simulator.run()
/// ```
#[pyclass(subclass)]
#[pyo3(name = "PluginAPI")]
pub struct PluginAPIWrapper {}

#[pymethods]
impl PluginAPIWrapper {
    /// Constructor for the PluginAPI. It does not take any argument and returns an instance of the PluginAPI.
    #[new]
    pub fn new() -> Self {
        Self {}
    }
    /// Return the [`StateEstimator`](crate::state_estimators::StateEstimator) to be used by the
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
    /// Returns the [`StateEstimator`](crate::state_estimators::StateEstimator) to use.
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

    /// Return the [`Physics`](crate::physics::Physics) to be used by the
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
    /// Returns the [`Physics`](crate::physics::Physics) to use.
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

/// Simulator wrapper to be used in Python. It provides a method to create a simulator from a configuration file and a method to run the simulator.
#[pyclass]
#[pyo3(name = "Simulator")]
pub struct SimulatorWrapper {
    simulator: AsyncSimulator,
    python_api: Option<Arc<dyn PluginAPI>>,
}

#[pymethods]
impl SimulatorWrapper {
    /// Constructor for the simulator.
    /// 
    /// # Arguments
    /// * `config_path` - Path to the configuration file. The configuration file should be in YAML format and follow the structure defined in the documentation. See the [`SimulatorConfig`](crate::simulator::SimulatorConfig) for more details on the configuration structure.
    /// * `plugin_api` - Optional Python object that implements the [`PluginAPI`] interface. This object will be used to provide custom implementations of the state estimator, controller, navigator and physics if required by the configuration.
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

    /// Run the simulator. This function will block until the simulation is finished.
    /// It will call the given state estimator, controller, navigator and physics at each simulation step.
    pub fn run(&mut self) {
        self.simulator.run(&self.python_api, None, false);
        self.simulator.compute_results();
        self.simulator.stop();
    }
}

/// Run the GUI of the simulator. This function will block until the GUI is closed but
/// will allow communications between Python thread and GUI to be able to use [`PythonAPI`] in the GUI.
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
