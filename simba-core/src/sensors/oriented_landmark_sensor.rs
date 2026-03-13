//! Oriented landmark sensor implementation.
//!
//! This module provides a [`Sensor`] that observes oriented landmarks
//! in the robot frame.
//! It supports configurable filters through
//! [`SensorFilterConfig`] and fault model pipelines configured by [`OrientedLandmarkSensorFaultModelConfig`].

use super::fault_models::fault_model::FaultModel;
use super::{Sensor, SensorObservation, SensorRecord};

use crate::constants::TIME_ROUND;
use crate::errors::SimbaResult;
#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::logger::is_enabled;
use crate::plugin_api::PluginAPI;
use crate::recordable::Recordable;
use crate::sensors::fault_models::additive::{AdditiveFault, AdditiveFaultConfig};
use crate::sensors::fault_models::clutter::{ClutterFault, ClutterFaultConfig};
use crate::sensors::fault_models::external_fault::{ExternalFault, ExternalFaultConfig};
use crate::sensors::fault_models::misassociation::{
    MisassociationFault, MisassociationFaultConfig,
};
use crate::sensors::fault_models::misdetection::{MisdetectionFault, MisdetectionFaultConfig};
use crate::sensors::fault_models::python_fault_model::{PythonFaultModel, PythonFaultModelConfig};
use crate::sensors::sensor_filters::{
    SensorFilter, SensorFilterConfig, SensorFilterType, make_sensor_filter_from_config,
};
use crate::simulator::SimulatorConfig;
use crate::state_estimators::State;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::enum_tools::EnumVariables;
use crate::utils::periodicity::{Periodicity, PeriodicityConfig};
use serde_derive::{Deserialize, Serialize};

use log::debug;
extern crate nalgebra as na;
use na::Vector3;
use simba_macros::{EnumToString, UIComponent, config_derives, enum_variables};

use std::sync::Arc;
use std::vec;

enum_variables!(
    "Variables used by oriented-landmark observations, filters, and fault models."
    OrientedLandmarkSensorVariables;
    "Variables accepted by sensor filters."
    Filter,
    "Variables modified by fault models."
    Faults:
    "Relative X coordinate."
    X, "x";
    Filter, Faults:
    "Relative Y coordinate."
    Y, "y";
    Filter, Faults: 
    "Relative orientation of the landmark with respect to the robot."
    Orientation, "orientation";
    Filter, Faults:
    "Relative distance to the landmark."
    R, "r", "range", "d", "distance";
    Filter, Faults:
    "Relative bearing of the landmark."
    Theta, "theta";
    Filter, Faults:
    "Width of the landmark."
    Width, "width";
    Filter, Faults: 
    "Absolute height of the landmark."
    Height, "height" ;
    Filter:
    "Absolute observer linear velocity"
    SelfVelocity, "self_velocity";
);

/// Configuration enum selecting oriented-landmark fault model strategies.
///
/// Default value: [`OrientedLandmarkSensorFaultModelConfig::AdditiveRobotCentered`] with
/// [`AdditiveFaultConfig::default`].
#[config_derives]
#[derive(UIComponent)]
#[show_all = "Faults"]
pub enum OrientedLandmarkSensorFaultModelConfig {
    /// Additive fault model in robot-centered coordinates.
    AdditiveRobotCentered(
        AdditiveFaultConfig<OrientedLandmarkSensorVariablesFaults, OrientedLandmarkSensorVariables>,
    ),
    /// Additive fault model in observation-centered coordinates.
    AdditiveObservationCentered(
        AdditiveFaultConfig<OrientedLandmarkSensorVariablesFaults, OrientedLandmarkSensorVariables>,
    ),
    /// Clutter fault model.
    Clutter(ClutterFaultConfig<OrientedLandmarkSensorVariablesFaults>),
    /// Misdetection fault model.
    Misdetection(MisdetectionFaultConfig),
    /// Misassociation fault model.
    Misassociation(MisassociationFaultConfig),
    /// Plugin-provided external fault model.
    External(ExternalFaultConfig),
    /// Python-implemented fault model.
    Python(PythonFaultModelConfig),
}

impl Default for OrientedLandmarkSensorFaultModelConfig {
    fn default() -> Self {
        Self::AdditiveRobotCentered(AdditiveFaultConfig::default())
    }
}

/// Runtime enum containing instantiated oriented-landmark fault models.
#[derive(Debug, EnumToString)]
pub enum OrientedLandmarkSensorFaultModelType {
    /// Instantiated additive robot-centered fault model.
    AdditiveRobotCentered(
        AdditiveFault<OrientedLandmarkSensorVariablesFaults, OrientedLandmarkSensorVariables>,
    ),
    /// Instantiated additive observation-centered fault model.
    AdditiveObservationCentered(
        AdditiveFault<OrientedLandmarkSensorVariablesFaults, OrientedLandmarkSensorVariables>,
    ),
    /// Instantiated clutter fault model.
    Clutter(ClutterFault<OrientedLandmarkSensorVariablesFaults>),
    /// Instantiated misdetection fault model.
    Misdetection(MisdetectionFault),
    /// Instantiated misassociation fault model.
    Misassociation(MisassociationFault),
    /// Instantiated external fault model.
    External(ExternalFault),
    /// Instantiated Python fault model.
    Python(PythonFaultModel),
}

impl OrientedLandmarkSensorFaultModelType {
    /// Initializes fault models that require runtime node context.
    pub fn post_init(
        &mut self,
        node: &mut crate::node::Node,
        initial_time: f32,
    ) -> crate::errors::SimbaResult<()> {
        match self {
            Self::Python(f) => f.post_init(node, initial_time),
            Self::External(f) => f.post_init(node, initial_time),
            Self::AdditiveRobotCentered(_)
            | Self::AdditiveObservationCentered(_)
            | Self::Clutter(_)
            | Self::Misdetection(_)
            | Self::Misassociation(_) => Ok(()),
        }
    }
}

/// Configuration of the [`OrientedLandmarkSensor`].
/// 
/// The occlusions are defined geometrically using all the landmarks and their height. If occlusion occurs on a
/// widthed landmark, the landmark is not observed at all and the observed width is reduced to the observable part.
///
/// Default values:
/// - `detection_distance`: `5.0`
/// - `activation_time`: `Some(PeriodicityConfig { period: 0.1, offset: None, table: None })`
/// - `faults`: empty vector
/// - `filters`: empty vector
/// - `xray`: `false`
#[config_derives]
pub struct OrientedLandmarkSensorConfig {
    /// Max distance of detection.
    pub detection_distance: f32,
    /// Periodicity configuration of the sensor.
    #[check]
    pub activation_time: Option<PeriodicityConfig>,
    #[check]
    /// Fault model configurations applied after filtering.
    #[check]
    pub faults: Vec<OrientedLandmarkSensorFaultModelConfig>,
    /// Filter configurations applied before fault injection.
    #[check]
    pub filters: Vec<SensorFilterConfig<OrientedLandmarkSensorVariablesFilter>>,
    /// If true, will detect all landmarks, even if they are behind obstacles (no raycasting).
    pub xray: bool,
}

impl Check for OrientedLandmarkSensorConfig {
    fn do_check(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.detection_distance < 0. {
            errors.push(format!(
                "Detection distance should be positive, got {}",
                self.detection_distance
            ));
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

impl Default for OrientedLandmarkSensorConfig {
    fn default() -> Self {
        Self {
            detection_distance: 5.0,
            activation_time: Some(PeriodicityConfig {
                period: crate::config::NumberConfig::Num(0.1),
                offset: None,
                table: None,
            }),
            faults: Vec::new(),
            filters: Vec::new(),
            xray: false,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for OrientedLandmarkSensorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Oriented Landmark sensor")
            .id_salt(format!("oriented-landmark-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Detection distance:");
                    if self.detection_distance < 0. {
                        self.detection_distance = 0.;
                    }
                    ui.add(egui::DragValue::new(&mut self.detection_distance));
                });

                ui.horizontal(|ui| {
                    if let Some(p) = &mut self.activation_time {
                        p.show_mut(
                            ui,
                            ctx,
                            buffer_stack,
                            global_config,
                            current_node_name,
                            unique_id,
                        );
                        if ui.button("Remove activation").clicked() {
                            self.activation_time = None;
                        }
                    } else if ui.button("Add activation").clicked() {
                        self.activation_time = Self::default().activation_time;
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("X-Ray mode:");
                    ui.checkbox(&mut self.xray, "");
                });

                SensorFilterConfig::show_filters_mut(
                    &mut self.filters,
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );

                OrientedLandmarkSensorFaultModelConfig::show_all_mut(
                    &mut self.faults,
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );
            });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new("Oriented Landmark sensor")
            .id_salt(format!("oriented-landmark-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(format!("Detection distance: {}", self.detection_distance));
                });

                ui.horizontal(|ui| {
                    if let Some(p) = &self.activation_time {
                        p.show(ui, ctx, unique_id);
                    } else {
                        ui.label("No activation");
                    }
                });

                ui.horizontal(|ui| {
                    ui.label(format!("X-Ray mode: {}", self.xray));
                });

                SensorFilterConfig::show_filters(&self.filters, ui, ctx, unique_id);

                OrientedLandmarkSensorFaultModelConfig::show_all(&self.faults, ui, ctx, unique_id);
            });
    }
}

/// Record of the [`OrientedLandmarkSensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct OrientedLandmarkSensorRecord {
    last_time: Option<f32>,
}

#[cfg(feature = "gui")]
impl UIComponent for OrientedLandmarkSensorRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.label(format!(
            "Last time: {}",
            match self.last_time {
                Some(t) => t.to_string(),
                None => "None".to_string(),
            }
        ));
    }
}

/// Observation of an [`OrientedLandmark`](crate::environment::oriented_landmark::OrientedLandmark).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OrientedLandmarkObservation {
    /// Id of the landmark
    pub id: i32,
    /// Labels associated with the observed landmark.
    pub labels: Vec<String>,
    /// Pose of the landmark
    pub pose: Vector3<f32>,
    /// Height of the landmark, used for obstruction checks
    pub height: f32,
    /// Can be 0 for ponctual landmarks
    pub width: f32,
    /// Fault models applied to this observation.
    pub applied_faults: Vec<OrientedLandmarkSensorFaultModelConfig>,
}

impl Default for OrientedLandmarkObservation {
    fn default() -> Self {
        Self {
            id: 0,
            labels: Vec::new(),
            pose: Vector3::new(0., 0., 0.),
            height: 1.,
            width: 0.,
            applied_faults: Vec::new(),
        }
    }
}

impl Recordable<OrientedLandmarkObservationRecord> for OrientedLandmarkObservation {
    fn record(&self) -> OrientedLandmarkObservationRecord {
        OrientedLandmarkObservationRecord {
            id: self.id,
            labels: self.labels.clone(),
            pose: self.pose.into(),
            height: self.height,
            width: self.width,
            applied_faults: self.applied_faults.clone(),
        }
    }
}

/// Serializable record representation of [`OrientedLandmarkObservation`].
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct OrientedLandmarkObservationRecord {
    /// Id of the landmark
    pub id: i32,
    /// Labels associated with the observed landmark.
    pub labels: Vec<String>,
    /// Pose of the landmark
    pub pose: [f32; 3],
    /// Landmark height.
    pub height: f32,
    /// Landmark width.
    pub width: f32,
    /// Fault models applied at observation generation time.
    pub applied_faults: Vec<OrientedLandmarkSensorFaultModelConfig>,
}

#[cfg(feature = "gui")]
impl UIComponent for OrientedLandmarkObservationRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!("Id: {}", self.id));
            ui.label("Labels:");
            for label in &self.labels {
                ui.label(format!("- {}", label));
            }
            ui.label(format!(
                "Pose: ({}, {}, {})",
                self.pose[0], self.pose[1], self.pose[2]
            ));
            if self.height <= 0. {
                ui.label(format!("Height: {}", self.height));
            }
            if self.width <= 0. {
                ui.label(format!("Width: {}", self.width));
            }
            if self.applied_faults.is_empty() {
                ui.label("No applied faults.");
            } else {
                ui.label("Applied faults:");
                for fault in &self.applied_faults {
                    ui.label(format!("{:?}", fault));
                }
            }
        });
    }
}

/// Sensor which observe the map landmarks.
#[derive(Debug)]
pub struct OrientedLandmarkSensor {
    /// Detection distance
    detection_distance: f32,
    /// Observation period
    activation_time: Option<Periodicity>,
    /// Last observation time.
    last_time: Option<f32>,
    faults: Vec<OrientedLandmarkSensorFaultModelType>,
    filters: Vec<SensorFilterType<OrientedLandmarkSensorVariablesFilter>>,
    /// If true, will detect all landmarks, even if they are behind obstacles (no raycasting).
    xray: bool,
}

impl OrientedLandmarkSensor {
    /// Makes a new [`OrientedLandmarkSensor`] from the given config.
    ///
    /// The map path is relative to the config path of the simulator.
    pub fn from_config(
        config: &OrientedLandmarkSensorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> SimbaResult<Self> {
        let mut fault_models = Vec::new();
        for fault_config in &config.faults {
            fault_models.push(match &fault_config {
                OrientedLandmarkSensorFaultModelConfig::AdditiveRobotCentered(c) => {
                    OrientedLandmarkSensorFaultModelType::AdditiveRobotCentered(
                        AdditiveFault::from_config(c, va_factory, initial_time),
                    )
                }
                OrientedLandmarkSensorFaultModelConfig::AdditiveObservationCentered(c) => {
                    OrientedLandmarkSensorFaultModelType::AdditiveObservationCentered(
                        AdditiveFault::from_config(c, va_factory, initial_time),
                    )
                }
                OrientedLandmarkSensorFaultModelConfig::Clutter(c) => {
                    OrientedLandmarkSensorFaultModelType::Clutter(ClutterFault::from_config(
                        c,
                        va_factory,
                        initial_time,
                    ))
                }
                OrientedLandmarkSensorFaultModelConfig::Misdetection(c) => {
                    OrientedLandmarkSensorFaultModelType::Misdetection(
                        MisdetectionFault::from_config(c, va_factory, initial_time),
                    )
                }
                OrientedLandmarkSensorFaultModelConfig::Misassociation(c) => {
                    OrientedLandmarkSensorFaultModelType::Misassociation(
                        MisassociationFault::from_config(c, va_factory),
                    )
                }
                OrientedLandmarkSensorFaultModelConfig::External(c) => {
                    OrientedLandmarkSensorFaultModelType::External(ExternalFault::from_config(
                        c,
                        plugin_api,
                        global_config,
                        va_factory,
                        initial_time,
                    )?)
                }
                OrientedLandmarkSensorFaultModelConfig::Python(c) => {
                    OrientedLandmarkSensorFaultModelType::Python(PythonFaultModel::from_config(
                        c,
                        global_config,
                        initial_time,
                    )?)
                }
            });
        }

        let mut filters = Vec::new();
        for filter_config in &config.filters {
            filters.push(make_sensor_filter_from_config(
                filter_config,
                plugin_api,
                global_config,
                va_factory,
                initial_time,
            )?);
        }

        let activation_time = config
            .activation_time
            .as_ref()
            .map(|p| Periodicity::from_config(p, va_factory, initial_time));

        Ok(Self {
            detection_distance: config.detection_distance,
            activation_time,
            last_time: None,
            faults: fault_models,
            filters,
            xray: config.xray,
        })
    }
}

use crate::node::Node;

impl Sensor for OrientedLandmarkSensor {
    fn post_init(&mut self, node: &mut Node, initial_time: f32) -> crate::errors::SimbaResult<()> {
        for filter in self.filters.iter_mut() {
            filter.post_init(node, initial_time)?;
        }
        for fault_model in self.faults.iter_mut() {
            fault_model.post_init(node, initial_time)?;
        }
        Ok(())
    }

    fn get_observations(&mut self, node: &mut Node, time: f32) -> Vec<SensorObservation> {
        let mut observation_list = Vec::<SensorObservation>::new();
        if let Some(last_time) = self.last_time
            && (time - last_time).abs() < TIME_ROUND
        {
            return observation_list;
        }
        let state = if let Some(arc_physic) = node.physics() {
            let physic = arc_physic.read().unwrap();
            physic.state(time).clone()
        } else {
            State::new() // 0
        };

        let rotation_matrix =
            nalgebra::geometry::Rotation3::from_euler_angles(0., 0., state.pose.z);

        let observable_landmarks = node.environment().get_observable_landmarks(
            &state.pose.fixed_rows::<2>(0).clone_owned(),
            if self.xray { None } else { Some(0.) },
            self.detection_distance,
            Some(node.name()),
        );

        for (i, landmark) in observable_landmarks.iter().enumerate() {
            let landmark_seed = (i + 1) as f32 / (100. * (time - self.last_time.unwrap_or(-1.)))
                * ((landmark.id + 1) as f32);
            let pose = rotation_matrix.transpose() * (landmark.pose - state.pose);
            let obs = SensorObservation::OrientedLandmark(OrientedLandmarkObservation {
                id: landmark.id,
                labels: landmark.labels.clone(),
                pose,
                applied_faults: Vec::new(),
                height: landmark.height,
                width: landmark.width,
            });

            let mut keep_observation = Some(obs);

            for filter in self.filters.iter() {
                if let Some(obs) = keep_observation {
                    keep_observation = match filter {
                        SensorFilterType::External(f) => f.filter(time, obs, &state, None),
                        SensorFilterType::PythonFilter(f) => f.filter(time, obs, &state, None),
                        SensorFilterType::RangeFilter(f) => {
                            if let SensorObservation::OrientedLandmark(obs) = obs {
                                if f.match_exclusion(
                                    &OrientedLandmarkSensorVariablesFilter::mapped_values(
                                        |variant| match variant {
                                            OrientedLandmarkSensorVariablesFilter::X => obs.pose.x,
                                            OrientedLandmarkSensorVariablesFilter::Y => obs.pose.y,
                                            OrientedLandmarkSensorVariablesFilter::Orientation => {
                                                obs.pose.z
                                            }
                                            OrientedLandmarkSensorVariablesFilter::R => {
                                                obs.pose.fixed_rows::<2>(0).norm()
                                            }
                                            OrientedLandmarkSensorVariablesFilter::Theta => {
                                                obs.pose.y.atan2(obs.pose.x)
                                            }
                                            OrientedLandmarkSensorVariablesFilter::SelfVelocity => {
                                                state.velocity.fixed_rows::<2>(0).norm()
                                            }
                                            OrientedLandmarkSensorVariablesFilter::Width => {
                                                obs.width
                                            }
                                            OrientedLandmarkSensorVariablesFilter::Height => {
                                                obs.height
                                            }
                                        },
                                    ),
                                ) {
                                    None
                                } else {
                                    Some(SensorObservation::OrientedLandmark(obs))
                                }
                            } else {
                                unreachable!()
                            }
                        }
                        SensorFilterType::IdFilter(f) => {
                            if let SensorObservation::OrientedLandmark(obs) = obs {
                                if f.match_exclusion(&[obs.id.to_string()]) {
                                    None
                                } else {
                                    Some(SensorObservation::OrientedLandmark(obs))
                                }
                            } else {
                                unreachable!()
                            }
                        }
                        SensorFilterType::LabelFilter(f) => {
                            if let SensorObservation::OrientedLandmark(obs) = obs {
                                if f.match_exclusion(&obs.labels) {
                                    None
                                } else {
                                    Some(SensorObservation::OrientedLandmark(obs))
                                }
                            } else {
                                unreachable!()
                            }
                        }
                    }
                } else {
                    break;
                }
            }

            let mut new_obs = Vec::new();
            if let Some(observation) = keep_observation {
                new_obs.push(observation); // Not adding directly to observation_list to apply faults only once
                for fault_model in self.faults.iter_mut() {
                    match fault_model {
                        OrientedLandmarkSensorFaultModelType::Python(f) => f.add_faults(
                            time,
                            time + landmark_seed,
                            &mut new_obs,
                            SensorObservation::OrientedLandmark(
                                OrientedLandmarkObservation::default(),
                            ),
                            node.environment(),
                        ),
                        OrientedLandmarkSensorFaultModelType::External(f) => f.add_faults(
                            time,
                            time + landmark_seed,
                            &mut new_obs,
                            SensorObservation::OrientedLandmark(
                                OrientedLandmarkObservation::default(),
                            ),
                            node.environment(),
                        ),
                        OrientedLandmarkSensorFaultModelType::AdditiveObservationCentered(f) => {
                            let obs_list_len = new_obs.len();
                            for (i, obs) in new_obs
                                .iter_mut()
                                .map(|o| {
                                    if let SensorObservation::OrientedLandmark(observation) = o {
                                        observation
                                    } else {
                                        unreachable!()
                                    }
                                })
                                .enumerate()
                            {
                                let seed = time + i as f32 / (100. * obs_list_len as f32);
                                let new_values = f.add_faults(
                                    seed,
                                    OrientedLandmarkSensorVariablesFaults::mapped_values(
                                        |variant| match variant {
                                            OrientedLandmarkSensorVariablesFaults::X => obs.pose.x,
                                            OrientedLandmarkSensorVariablesFaults::Y => obs.pose.y,
                                            OrientedLandmarkSensorVariablesFaults::Orientation => {
                                                obs.pose.z
                                            }
                                            OrientedLandmarkSensorVariablesFaults::R => 0.,
                                            OrientedLandmarkSensorVariablesFaults::Theta => {
                                                obs.pose.z
                                            }
                                            OrientedLandmarkSensorVariablesFaults::Height => {
                                                obs.height
                                            }
                                            OrientedLandmarkSensorVariablesFaults::Width => {
                                                obs.width
                                            }
                                        },
                                    ),
                                    &OrientedLandmarkSensorVariables::mapped_values(|variant| {
                                        match variant {
                                            OrientedLandmarkSensorVariables::Orientation => {
                                                obs.pose.z
                                            }
                                            OrientedLandmarkSensorVariables::R => {
                                                obs.pose.fixed_rows::<2>(0).norm()
                                            }
                                            OrientedLandmarkSensorVariables::Theta => obs.pose.z,
                                            OrientedLandmarkSensorVariables::X => obs.pose.x,
                                            OrientedLandmarkSensorVariables::Y => obs.pose.y,
                                            OrientedLandmarkSensorVariables::SelfVelocity => {
                                                state.velocity.fixed_rows::<2>(0).norm()
                                            }
                                            OrientedLandmarkSensorVariables::Height => obs.height,
                                            OrientedLandmarkSensorVariables::Width => obs.width,
                                        }
                                    }),
                                );
                                if let Some(new_x) =
                                    new_values.get(&OrientedLandmarkSensorVariablesFaults::X)
                                {
                                    obs.pose.x = *new_x;
                                }
                                if let Some(new_y) =
                                    new_values.get(&OrientedLandmarkSensorVariablesFaults::Y)
                                {
                                    obs.pose.y = *new_y;
                                }
                                let new_r = if let Some(new_r) =
                                    new_values.get(&OrientedLandmarkSensorVariablesFaults::R)
                                {
                                    *new_r
                                } else {
                                    0.
                                };
                                let new_theta = if let Some(new_theta) =
                                    new_values.get(&OrientedLandmarkSensorVariablesFaults::Theta)
                                {
                                    *new_theta
                                } else {
                                    obs.pose.z
                                };
                                obs.pose.x += new_r * new_theta.cos();
                                obs.pose.y += new_r * new_theta.sin();
                                if let Some(new_orientation) = new_values
                                    .get(&OrientedLandmarkSensorVariablesFaults::Orientation)
                                {
                                    obs.pose.z = *new_orientation;
                                }
                                if let Some(new_height) =
                                    new_values.get(&OrientedLandmarkSensorVariablesFaults::Height)
                                {
                                    obs.height = *new_height;
                                }
                                if let Some(new_width) =
                                    new_values.get(&OrientedLandmarkSensorVariablesFaults::Width)
                                {
                                    obs.width = *new_width;
                                }
                            }
                        }
                        OrientedLandmarkSensorFaultModelType::AdditiveRobotCentered(f) => {
                            let obs_list_len = new_obs.len();
                            for (i, obs) in new_obs
                                .iter_mut()
                                .map(|o| {
                                    if let SensorObservation::OrientedLandmark(observation) = o {
                                        observation
                                    } else {
                                        unreachable!()
                                    }
                                })
                                .enumerate()
                            {
                                let seed = time + i as f32 / (100. * obs_list_len as f32);
                                let new_values = f.add_faults(
                                    seed,
                                    OrientedLandmarkSensorVariablesFaults::mapped_values(
                                        |variant| match variant {
                                            OrientedLandmarkSensorVariablesFaults::X => obs.pose.x,
                                            OrientedLandmarkSensorVariablesFaults::Y => obs.pose.y,
                                            OrientedLandmarkSensorVariablesFaults::Orientation => {
                                                obs.pose.z
                                            }
                                            OrientedLandmarkSensorVariablesFaults::R => {
                                                obs.pose.fixed_rows::<2>(0).norm()
                                            }
                                            OrientedLandmarkSensorVariablesFaults::Theta => {
                                                obs.pose.y.atan2(obs.pose.x)
                                            }
                                            OrientedLandmarkSensorVariablesFaults::Height => {
                                                obs.height
                                            }
                                            OrientedLandmarkSensorVariablesFaults::Width => {
                                                obs.width
                                            }
                                        },
                                    ),
                                    &OrientedLandmarkSensorVariables::mapped_values(|variant| {
                                        match variant {
                                            OrientedLandmarkSensorVariables::Orientation => {
                                                obs.pose.z
                                            }
                                            OrientedLandmarkSensorVariables::R => {
                                                obs.pose.fixed_rows::<2>(0).norm()
                                            }
                                            OrientedLandmarkSensorVariables::Theta => {
                                                obs.pose.y.atan2(obs.pose.x)
                                            }
                                            OrientedLandmarkSensorVariables::X => obs.pose.x,
                                            OrientedLandmarkSensorVariables::Y => obs.pose.y,
                                            OrientedLandmarkSensorVariables::SelfVelocity => {
                                                state.velocity.fixed_rows::<2>(0).norm()
                                            }
                                            OrientedLandmarkSensorVariables::Height => obs.height,
                                            OrientedLandmarkSensorVariables::Width => obs.width,
                                        }
                                    }),
                                );
                                if let Some(new_x) =
                                    new_values.get(&OrientedLandmarkSensorVariablesFaults::X)
                                {
                                    obs.pose.x = *new_x;
                                }
                                if let Some(new_y) =
                                    new_values.get(&OrientedLandmarkSensorVariablesFaults::Y)
                                {
                                    obs.pose.y = *new_y;
                                }
                                let new_r = if let Some(new_r) =
                                    new_values.get(&OrientedLandmarkSensorVariablesFaults::R)
                                {
                                    *new_r
                                } else {
                                    obs.pose.fixed_rows::<2>(0).norm()
                                };
                                let new_theta = if let Some(new_theta) =
                                    new_values.get(&OrientedLandmarkSensorVariablesFaults::Theta)
                                {
                                    *new_theta
                                } else {
                                    obs.pose.y.atan2(obs.pose.x)
                                };
                                obs.pose.x = new_r * new_theta.cos();
                                obs.pose.y = new_r * new_theta.sin();
                                if let Some(new_orientation) = new_values
                                    .get(&OrientedLandmarkSensorVariablesFaults::Orientation)
                                {
                                    obs.pose.z = *new_orientation;
                                }
                                if let Some(new_height) =
                                    new_values.get(&OrientedLandmarkSensorVariablesFaults::Height)
                                {
                                    obs.height = *new_height;
                                }
                                if let Some(new_width) =
                                    new_values.get(&OrientedLandmarkSensorVariablesFaults::Width)
                                {
                                    obs.width = *new_width;
                                }
                            }
                        }
                        OrientedLandmarkSensorFaultModelType::Clutter(f) => {
                            let new_obs_from_clutter =
                                f.add_faults(time + landmark_seed, landmark_seed / 100.);
                            for (obs_id, obs_params) in new_obs_from_clutter {
                                let mut x = obs_params
                                    .get(&OrientedLandmarkSensorVariablesFaults::X)
                                    .cloned()
                                    .unwrap_or(0.);
                                let mut y = obs_params
                                    .get(&OrientedLandmarkSensorVariablesFaults::Y)
                                    .cloned()
                                    .unwrap_or(0.);
                                let orientation = obs_params
                                    .get(&OrientedLandmarkSensorVariablesFaults::Orientation)
                                    .cloned()
                                    .unwrap_or(0.);

                                let r = obs_params
                                    .get(&OrientedLandmarkSensorVariablesFaults::R)
                                    .cloned()
                                    .unwrap_or(0.);
                                let theta = obs_params
                                    .get(&OrientedLandmarkSensorVariablesFaults::Theta)
                                    .cloned()
                                    .unwrap_or(0.);
                                x += r * theta.cos();
                                y += r * theta.sin();

                                let obs = SensorObservation::OrientedLandmark(
                                    OrientedLandmarkObservation {
                                        id: obs_id.parse().unwrap_or(0),
                                        height: obs_params
                                            .get(&OrientedLandmarkSensorVariablesFaults::Height)
                                            .cloned()
                                            .unwrap_or(0.),
                                        width: obs_params
                                            .get(&OrientedLandmarkSensorVariablesFaults::Width)
                                            .cloned()
                                            .unwrap_or(0.),
                                        labels: Vec::new(),
                                        pose: Vector3::new(x, y, orientation),
                                        applied_faults: vec![
                                            OrientedLandmarkSensorFaultModelConfig::Clutter(
                                                f.config().clone(),
                                            ),
                                        ],
                                    },
                                );
                                new_obs.push(obs);
                            }
                        }
                        OrientedLandmarkSensorFaultModelType::Misassociation(f) => {
                            for (i, obs) in new_obs.iter_mut().enumerate() {
                                if let SensorObservation::OrientedLandmark(observation) = obs {
                                    let new_label = f.new_label(
                                        time + landmark_seed + (i as f32) / 1000.,
                                        observation.id.to_string(),
                                        observation.pose.fixed_rows::<2>(0).clone_owned(),
                                        node.environment(),
                                    );
                                    observation.id = new_label.parse().unwrap_or(observation.id);
                                } else {
                                    unreachable!()
                                }
                            }
                        }
                        OrientedLandmarkSensorFaultModelType::Misdetection(f) => {
                            new_obs = new_obs
                                .iter()
                                .enumerate()
                                .filter_map(|(i, obs)| {
                                    if let SensorObservation::OrientedLandmark(observation) = obs {
                                        if f.detected(time + landmark_seed + (i as f32) / 1000.) {
                                            Some(SensorObservation::OrientedLandmark(
                                                observation.clone(),
                                            ))
                                        } else {
                                            None
                                        }
                                    } else {
                                        unreachable!()
                                    }
                                })
                                .collect();
                        }
                    }
                }
            } else if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
                debug!(
                    "Observation {i} of landmark {} was filtered out",
                    landmark.id
                );
            }
            observation_list.extend(new_obs);
        }
        if let Some(p) = self.activation_time.as_mut() {
            p.update(time);
        }
        self.last_time = Some(time);
        observation_list
    }

    /// Get the next observation time.
    fn next_time_step(&self) -> f32 {
        if let Some(activation) = &self.activation_time {
            activation.next_time()
        } else {
            f32::INFINITY
        }
    }
}

impl Recordable<SensorRecord> for OrientedLandmarkSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::OrientedLandmarkSensor(OrientedLandmarkSensorRecord {
            last_time: self.last_time,
        })
    }
}
