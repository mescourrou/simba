/*!
Provides a [`Sensor`] which can observe the other nodes in the frame of the ego node.
*/

use super::fault_models::fault_model::FaultModel;
use super::{Sensor, SensorObservation, SensorRecord};

use crate::constants::TIME_ROUND;

use crate::errors::{SimbaErrorTypes, SimbaResult};
#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::logger::is_enabled;
use crate::networking::service_manager::ServiceError;
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
use crate::utils::SharedMutex;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::enum_tools::EnumVariables;
use crate::utils::periodicity::{Periodicity, PeriodicityConfig};
use serde_derive::{Deserialize, Serialize};

use log::debug;
extern crate nalgebra as na;
use na::Vector3;
use simba_macros::{EnumToString, ToVec, UIComponent, config_derives, enum_variables};

use std::fmt;
use std::sync::{Arc, Mutex};

enum_variables!(
    RobotSensorVariables;
    Filter, Faults: X, "x";
    Filter, Faults: Y, "y";
    Filter, Faults: Orientation, "orientation", "z";
    Filter, Faults: R, "r";
    Filter, Faults: Theta, "theta" ;
    Filter: SelfVelocity, "self_velocity";
    Filter: TargetVelocity, "target_velocity";
);

#[config_derives]
#[derive(UIComponent)]
#[show_all = "Faults"]
pub enum RobotSensorFaultModelConfig {
    #[check]
    AdditiveRobotCentered(AdditiveFaultConfig<RobotSensorVariablesFaults, RobotSensorVariables>),
    AdditiveObservationCentered(
        AdditiveFaultConfig<RobotSensorVariablesFaults, RobotSensorVariables>,
    ),
    #[check]
    Clutter(ClutterFaultConfig<RobotSensorVariablesFaults>),
    #[check]
    Misdetection(MisdetectionFaultConfig),
    #[check]
    Misassociation(MisassociationFaultConfig),
    #[check]
    Python(PythonFaultModelConfig),
    #[check]
    External(ExternalFaultConfig),
}

impl Default for RobotSensorFaultModelConfig {
    fn default() -> Self {
        Self::AdditiveRobotCentered(AdditiveFaultConfig::default())
    }
}

#[derive(Debug, EnumToString)]
pub enum RobotSensorFaultModelType {
    AdditiveRobotCentered(AdditiveFault<RobotSensorVariablesFaults, RobotSensorVariables>),
    AdditiveObservationCentered(AdditiveFault<RobotSensorVariablesFaults, RobotSensorVariables>),
    Clutter(ClutterFault<RobotSensorVariablesFaults>),
    Misdetection(MisdetectionFault),
    Misassociation(MisassociationFault),
    Python(PythonFaultModel),
    External(ExternalFault),
}

impl RobotSensorFaultModelType {
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

/// Configuration of the [`RobotSensor`].
#[config_derives]
pub struct RobotSensorConfig {
    /// Max distance of detection.
    pub detection_distance: f32,
    /// Observation period of the sensor.
    #[check]
    pub activation_time: Option<PeriodicityConfig>,
    #[check]
    pub faults: Vec<RobotSensorFaultModelConfig>,
    #[check]
    pub filters: Vec<SensorFilterConfig<RobotSensorVariablesFilter>>,
    pub xray: bool,
}

impl Check for RobotSensorConfig {
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

impl Default for RobotSensorConfig {
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
impl UIComponent for RobotSensorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Robot sensor")
            .id_salt(format!("robot-sensor-{}", unique_id))
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
                        self.activation_time = Some(Self::default().activation_time.unwrap());
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

                RobotSensorFaultModelConfig::show_all_mut(
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
        egui::CollapsingHeader::new("Robot sensor")
            .id_salt(format!("robot-sensor-{}", unique_id))
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

                RobotSensorFaultModelConfig::show_all(&self.faults, ui, ctx, unique_id);
            });
    }
}

/// Record of the [`RobotSensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct RobotSensorRecord {
    last_time: Option<f32>,
}

#[cfg(feature = "gui")]
impl UIComponent for RobotSensorRecord {
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

/// Landmark struct, with an `id` and a `pose`, used to read the map file.
#[derive(Debug)]
pub struct OrientedRobot {
    pub name: String,
    pub pose: Vector3<f32>,
}

use serde::ser::{Serialize, SerializeStruct, Serializer};

impl Serialize for OrientedRobot {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // 3 is the number of fields in the struct.
        let mut state = serializer.serialize_struct("OrientedRobot", 4)?;
        state.serialize_field("name", self.name.as_str())?;
        state.serialize_field("x", &self.pose.x)?;
        state.serialize_field("y", &self.pose.y)?;
        state.serialize_field("theta", &self.pose.z)?;
        state.end()
    }
}

use serde::de::{self, Deserialize, Deserializer, MapAccess, SeqAccess, Visitor};

impl<'de> Deserialize<'de> for OrientedRobot {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        enum Field {
            Name,
            X,
            Y,
            Theta,
            Unknown,
        }

        // This part could also be generated independently by:
        //
        //    #[derive(Deserialize)]
        //    #[serde(field_identifier, rename_all = "lowercase")]
        //    enum Field { Secs, Nanos }
        impl<'de> Deserialize<'de> for Field {
            fn deserialize<D>(deserializer: D) -> Result<Field, D::Error>
            where
                D: Deserializer<'de>,
            {
                struct FieldVisitor;

                impl<'de> Visitor<'de> for FieldVisitor {
                    type Value = Field;

                    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                        formatter.write_str("`name` or `x` or `y` or `theta`")
                    }

                    fn visit_str<E>(self, value: &str) -> Result<Field, E>
                    where
                        E: de::Error,
                    {
                        match value {
                            "name" => Ok(Field::Name),
                            "x" => Ok(Field::X),
                            "y" => Ok(Field::Y),
                            "theta" => Ok(Field::Theta),
                            _ => Ok(Field::Unknown),
                        }
                    }
                }

                deserializer.deserialize_identifier(FieldVisitor)
            }
        }

        struct OrientedRobotVisitor;

        impl<'de> Visitor<'de> for OrientedRobotVisitor {
            type Value = OrientedRobot;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct OrientedRobot")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<OrientedRobot, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let name: &str = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                let x: f32 = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(1, &self))?;
                let y: f32 = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(2, &self))?;
                let theta: f32 = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(3, &self))?;
                Ok(OrientedRobot {
                    name: name.to_string(),
                    pose: Vector3::from_vec(vec![x, y, theta]),
                })
            }

            fn visit_map<V>(self, mut map: V) -> Result<OrientedRobot, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut name = None;
                let mut x = None;
                let mut y = None;
                let mut theta = None;

                while let Some(key) = map.next_key()? {
                    match key {
                        Field::Name => {
                            if name.is_some() {
                                return Err(de::Error::duplicate_field("name"));
                            }
                            name = Some(map.next_value()?);
                        }
                        Field::X => {
                            if x.is_some() {
                                return Err(de::Error::duplicate_field("x"));
                            }
                            x = Some(map.next_value()?);
                        }
                        Field::Y => {
                            if y.is_some() {
                                return Err(de::Error::duplicate_field("y"));
                            }
                            y = Some(map.next_value()?);
                        }
                        Field::Theta => {
                            if theta.is_some() {
                                return Err(de::Error::duplicate_field("theta"));
                            }
                            theta = Some(map.next_value()?);
                        }
                        Field::Unknown => {}
                    }
                }
                let name = name.ok_or_else(|| de::Error::missing_field("name"))?;
                let x = x.ok_or_else(|| de::Error::missing_field("x"))?;
                let y = y.ok_or_else(|| de::Error::missing_field("y"))?;
                let theta = theta.ok_or_else(|| de::Error::missing_field("theta"))?;
                Ok(OrientedRobot {
                    name,
                    pose: Vector3::from_vec(vec![x, y, theta]),
                })
            }
        }

        const FIELDS: &[&str] = &["name", "x", "y", "theta"];
        deserializer.deserialize_struct("OrientedRobot", FIELDS, OrientedRobotVisitor)
    }
}

/// Observation of an [`OrientedRobot`].
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct OrientedRobotObservation {
    /// Name of the Robot
    pub name: String,
    pub labels: Vec<String>,
    /// Pose of the Robot
    pub pose: Vector3<f32>,
    pub applied_faults: Vec<RobotSensorFaultModelConfig>,
}

impl Recordable<OrientedRobotObservationRecord> for OrientedRobotObservation {
    fn record(&self) -> OrientedRobotObservationRecord {
        OrientedRobotObservationRecord {
            name: self.name.clone(),
            labels: self.labels.clone(),
            pose: self.pose.to_owned().into(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct OrientedRobotObservationRecord {
    /// Name of the Robot
    pub name: String,
    pub labels: Vec<String>,
    /// Pose of the Robot
    pub pose: [f32; 3],
}

#[cfg(feature = "gui")]
impl UIComponent for OrientedRobotObservationRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!("Name: {}", self.name));
            ui.label("Labels:");
            for label in &self.labels {
                ui.label(format!("- {}", label));
            }
            ui.label(format!(
                "Pose: ({}, {}, {})",
                self.pose[0], self.pose[1], self.pose[2]
            ));
        });
    }
}

/// Sensor which observe the other Robots.
#[derive(Debug)]
pub struct RobotSensor {
    /// Detection distance
    detection_distance: f32,
    /// Observation period
    activation_time: Option<Periodicity>,
    /// Last observation time.
    last_time: Option<f32>,
    xray: bool,
    faults: Vec<RobotSensorFaultModelType>,
    filters: Vec<SensorFilterType<RobotSensorVariablesFilter>>,
}

impl RobotSensor {
    /// Makes a new [`RobotSensor`] from the given config.
    ///
    /// The map path is relative to the config path of the simulator.
    pub fn from_config(
        config: &RobotSensorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        node_name: &str,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> SimbaResult<Self> {
        let mut fault_models = Vec::new();
        for fault_config in &config.faults {
            fault_models.push(match &fault_config {
                &RobotSensorFaultModelConfig::AdditiveRobotCentered(cfg) => {
                    RobotSensorFaultModelType::AdditiveRobotCentered(AdditiveFault::from_config(
                        cfg,
                        va_factory,
                        initial_time,
                    ))
                }
                &RobotSensorFaultModelConfig::AdditiveObservationCentered(cfg) => {
                    RobotSensorFaultModelType::AdditiveObservationCentered(
                        AdditiveFault::from_config(cfg, va_factory, initial_time),
                    )
                }
                &RobotSensorFaultModelConfig::Clutter(cfg) => RobotSensorFaultModelType::Clutter(
                    ClutterFault::from_config(cfg, va_factory, initial_time),
                ),
                &RobotSensorFaultModelConfig::Misdetection(cfg) => {
                    RobotSensorFaultModelType::Misdetection(MisdetectionFault::from_config(
                        cfg,
                        va_factory,
                        initial_time,
                    ))
                }
                &RobotSensorFaultModelConfig::Misassociation(cfg) => {
                    RobotSensorFaultModelType::Misassociation(MisassociationFault::from_config(
                        cfg, va_factory,
                    ))
                }
                &RobotSensorFaultModelConfig::Python(cfg) => RobotSensorFaultModelType::Python(
                    PythonFaultModel::from_config(cfg, global_config, initial_time)
                        .expect("Failed to create Python Fault Model"),
                ),
                &RobotSensorFaultModelConfig::External(cfg) => RobotSensorFaultModelType::External(
                    ExternalFault::from_config(
                        cfg,
                        plugin_api,
                        global_config,
                        va_factory,
                        initial_time,
                    )
                    .expect("Failed to create External Fault Model"),
                ),
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

        let period = config
            .activation_time
            .as_ref()
            .map(|p| Periodicity::from_config(p, va_factory, initial_time));
        Ok(Self {
            detection_distance: config.detection_distance,
            activation_time: period,
            last_time: None,
            faults: fault_models,
            xray: config.xray,
            filters,
        })
    }
}

use crate::node::Node;

impl Sensor for RobotSensor {
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
        if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
            debug!("Start looking for nodes");
        }
        let state = if let Some(arc_physics) = node.physics() {
            let physics = arc_physics.read().unwrap();
            physics.state(time).clone()
        } else {
            State::new() // 0
        };

        let rotation_matrix =
            nalgebra::geometry::Rotation3::from_euler_angles(0., 0., state.pose.z);
        if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
            debug!("Rotation matrix: {}", rotation_matrix);
        }

        for (i, other_node_name) in node.other_node_names().iter().enumerate() {
            if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
                debug!("Sensing node {}", other_node_name);
            }
            assert!(*other_node_name != node.name());

            let service_manager = node.service_manager();
            match service_manager.read().unwrap().get_real_state(
                &other_node_name.to_string(),
                node,
                time,
            ) {
                Ok(other_state) => {
                    if node.environment().is_target_observable(
                        &other_state.pose.fixed_rows::<2>(0).clone_owned(),
                        Some(0.),
                        &state.pose.fixed_rows::<2>(0).clone_owned(),
                        if self.xray { None } else { Some(0.) },
                        self.detection_distance,
                        Some(node.name().clone()),
                    ) {
                        let robot_seed =
                            (i as f32) / (100. * (time - self.last_time.unwrap_or(-1.)));
                        let pose = rotation_matrix.transpose() * (other_state.pose - state.pose);
                        let labels = node
                            .meta_data_list()
                            .unwrap()
                            .read()
                            .unwrap()
                            .get(other_node_name)
                            .map_or(Vec::new(), |md| md.read().unwrap().labels.clone());
                        let obs = SensorObservation::OrientedRobot(OrientedRobotObservation {
                            name: other_node_name.clone(),
                            labels,
                            pose,
                            applied_faults: Vec::new(),
                        });

                        let mut keep_observation = Some(obs);

                        for filter in self.filters.iter() {
                            if let Some(obs) = keep_observation {
                                keep_observation = match filter {
                                    SensorFilterType::PythonFilter(f) => {
                                        f.filter(time, obs, &state, Some(&other_state))
                                    }
                                    SensorFilterType::External(f) => {
                                        f.filter(time, obs, &state, Some(&other_state))
                                    }
                                    SensorFilterType::IdFilter(f) => {
                                        if let SensorObservation::OrientedRobot(obs) = obs {
                                            if f.match_exclusion(&[obs.name.clone()]) {
                                                Some(SensorObservation::OrientedRobot(obs))
                                            } else {
                                                None
                                            }
                                        } else {
                                            unreachable!()
                                        }
                                    }
                                    SensorFilterType::LabelFilter(f) => {
                                        if let SensorObservation::OrientedRobot(obs) = obs {
                                            if f.match_exclusion(&obs.labels) {
                                                Some(SensorObservation::OrientedRobot(obs))
                                            } else {
                                                None
                                            }
                                        } else {
                                            unreachable!()
                                        }
                                    }
                                    SensorFilterType::RangeFilter(f) => {
                                        if let SensorObservation::OrientedRobot(obs) = obs {
                                            if f.match_exclusion(
                                                &RobotSensorVariablesFilter::mapped_values(|variant| {
                                                    match variant {
                                                        RobotSensorVariablesFilter::X => obs.pose.x,
                                                        RobotSensorVariablesFilter::Y => obs.pose.y,
                                                        RobotSensorVariablesFilter::Orientation => {
                                                            obs.pose.z
                                                        }
                                                        RobotSensorVariablesFilter::R => {
                                                            obs.pose.fixed_rows::<2>(0).norm()
                                                        }
                                                        RobotSensorVariablesFilter::Theta => {
                                                            obs.pose.y.atan2(obs.pose.x)
                                                        }
                                                        RobotSensorVariablesFilter::SelfVelocity => {
                                                            state.velocity.fixed_rows::<2>(0).norm()
                                                        }
                                                        RobotSensorVariablesFilter::TargetVelocity => {
                                                            other_state
                                                                .velocity
                                                                .fixed_rows::<2>(0)
                                                                .norm()
                                                        }
                                                    }
                                                }),
                                            ) {
                                                Some(SensorObservation::OrientedRobot(obs))
                                            } else {
                                                None
                                            }
                                        } else {
                                            unreachable!()
                                        }
                                    }
                                };
                            } else {
                                break;
                            }
                        }

                        let mut new_obs = Vec::new();
                        if let Some(observation) = keep_observation {
                            new_obs.push(observation); // Not adding directly to observation_list to apply faults only once
                            for fault_model in self.faults.iter_mut() {
                                match fault_model {
                                    RobotSensorFaultModelType::Python(f) => f.add_faults(
                                        time,
                                        time + robot_seed,
                                        &mut new_obs,
                                        SensorObservation::OrientedRobot(
                                            OrientedRobotObservation::default(),
                                        ),
                                        node.environment(),
                                    ),
                                    RobotSensorFaultModelType::External(f) => f.add_faults(
                                        time,
                                        time + robot_seed,
                                        &mut new_obs,
                                        SensorObservation::OrientedRobot(
                                            OrientedRobotObservation::default(),
                                        ),
                                        node.environment(),
                                    ),
                                    RobotSensorFaultModelType::AdditiveObservationCentered(f) => {
                                        let obs_list_len = new_obs.len();
                                        for (i, obs) in new_obs
                                            .iter_mut()
                                            .map(|o| {
                                                if let SensorObservation::OrientedRobot(
                                                    observation,
                                                ) = o
                                                {
                                                    observation
                                                } else {
                                                    unreachable!()
                                                }
                                            })
                                            .enumerate()
                                        {
                                            let seed =
                                                time + i as f32 / (100. * obs_list_len as f32);
                                            let new_values = f.add_faults(
                                                seed,
                                                RobotSensorVariablesFaults::mapped_values(
                                                    |variant| match variant {
                                                        RobotSensorVariablesFaults::X => obs.pose.x,
                                                        RobotSensorVariablesFaults::Y => obs.pose.y,
                                                        RobotSensorVariablesFaults::Orientation => {
                                                            obs.pose.z
                                                        }
                                                        RobotSensorVariablesFaults::R => 0.,
                                                        RobotSensorVariablesFaults::Theta => {
                                                            obs.pose.z
                                                        }
                                                    },
                                                ),
                                                &RobotSensorVariables::mapped_values(|variant| {
                                                    match variant {
                                                        RobotSensorVariables::Orientation => {
                                                            obs.pose.z
                                                        }
                                                        RobotSensorVariables::R => {
                                                            obs.pose.fixed_rows::<2>(0).norm()
                                                        }
                                                        RobotSensorVariables::Theta => obs.pose.z,
                                                        RobotSensorVariables::X => obs.pose.x,
                                                        RobotSensorVariables::Y => obs.pose.y,
                                                        RobotSensorVariables::SelfVelocity => {
                                                            state.velocity.fixed_rows::<2>(0).norm()
                                                        }
                                                        RobotSensorVariables::TargetVelocity => {
                                                            other_state
                                                                .velocity
                                                                .fixed_rows::<2>(0)
                                                                .norm()
                                                        }
                                                    }
                                                }),
                                            );
                                            if let Some(new_x) =
                                                new_values.get(&RobotSensorVariablesFaults::X)
                                            {
                                                obs.pose.x = *new_x;
                                            }
                                            if let Some(new_y) =
                                                new_values.get(&RobotSensorVariablesFaults::Y)
                                            {
                                                obs.pose.y = *new_y;
                                            }
                                            let new_r = if let Some(new_r) =
                                                new_values.get(&RobotSensorVariablesFaults::R)
                                            {
                                                *new_r
                                            } else {
                                                0.
                                            };
                                            let new_theta = if let Some(new_theta) =
                                                new_values.get(&RobotSensorVariablesFaults::Theta)
                                            {
                                                *new_theta
                                            } else {
                                                obs.pose.z
                                            };
                                            obs.pose.x += new_r * new_theta.cos();
                                            obs.pose.y += new_r * new_theta.sin();
                                            if let Some(new_orientation) = new_values
                                                .get(&RobotSensorVariablesFaults::Orientation)
                                            {
                                                obs.pose.z = *new_orientation;
                                            }
                                        }
                                    }
                                    RobotSensorFaultModelType::AdditiveRobotCentered(f) => {
                                        let obs_list_len = new_obs.len();
                                        for (i, obs) in new_obs
                                            .iter_mut()
                                            .map(|o| {
                                                if let SensorObservation::OrientedRobot(
                                                    observation,
                                                ) = o
                                                {
                                                    observation
                                                } else {
                                                    unreachable!()
                                                }
                                            })
                                            .enumerate()
                                        {
                                            let seed =
                                                time + i as f32 / (100. * obs_list_len as f32);
                                            let new_values = f.add_faults(
                                                seed,
                                                RobotSensorVariablesFaults::mapped_values(
                                                    |variant| match variant {
                                                        RobotSensorVariablesFaults::X => obs.pose.x,
                                                        RobotSensorVariablesFaults::Y => obs.pose.y,
                                                        RobotSensorVariablesFaults::Orientation => {
                                                            obs.pose.z
                                                        }
                                                        RobotSensorVariablesFaults::R => {
                                                            obs.pose.fixed_rows::<2>(0).norm()
                                                        }
                                                        RobotSensorVariablesFaults::Theta => {
                                                            obs.pose.y.atan2(obs.pose.x)
                                                        }
                                                    },
                                                ),
                                                &RobotSensorVariables::mapped_values(|variant| {
                                                    match variant {
                                                        RobotSensorVariables::Orientation => {
                                                            obs.pose.z
                                                        }
                                                        RobotSensorVariables::R => {
                                                            obs.pose.fixed_rows::<2>(0).norm()
                                                        }
                                                        RobotSensorVariables::Theta => {
                                                            obs.pose.y.atan2(obs.pose.x)
                                                        }
                                                        RobotSensorVariables::X => obs.pose.x,
                                                        RobotSensorVariables::Y => obs.pose.y,
                                                        RobotSensorVariables::SelfVelocity => {
                                                            state.velocity.fixed_rows::<2>(0).norm()
                                                        }
                                                        RobotSensorVariables::TargetVelocity => {
                                                            other_state
                                                                .velocity
                                                                .fixed_rows::<2>(0)
                                                                .norm()
                                                        }
                                                    }
                                                }),
                                            );
                                            if let Some(new_x) =
                                                new_values.get(&RobotSensorVariablesFaults::X)
                                            {
                                                obs.pose.x = *new_x;
                                            }
                                            if let Some(new_y) =
                                                new_values.get(&RobotSensorVariablesFaults::Y)
                                            {
                                                obs.pose.y = *new_y;
                                            }
                                            let new_r = if let Some(new_r) =
                                                new_values.get(&RobotSensorVariablesFaults::R)
                                            {
                                                *new_r
                                            } else {
                                                obs.pose.fixed_rows::<2>(0).norm()
                                            };
                                            let new_theta = if let Some(new_theta) =
                                                new_values.get(&RobotSensorVariablesFaults::Theta)
                                            {
                                                *new_theta
                                            } else {
                                                obs.pose.y.atan2(obs.pose.x)
                                            };
                                            obs.pose.x = new_r * new_theta.cos();
                                            obs.pose.y = new_r * new_theta.sin();
                                            if let Some(new_orientation) = new_values
                                                .get(&RobotSensorVariablesFaults::Orientation)
                                            {
                                                obs.pose.z = *new_orientation;
                                            }
                                        }
                                    }
                                    RobotSensorFaultModelType::Clutter(f) => {
                                        let new_obs_from_clutter =
                                            f.add_faults(time + robot_seed, robot_seed / 100.);
                                        for (obs_id, obs_params) in new_obs_from_clutter {
                                            let mut x = obs_params
                                                .get(&RobotSensorVariablesFaults::X)
                                                .cloned()
                                                .unwrap_or(0.);
                                            let mut y = obs_params
                                                .get(&RobotSensorVariablesFaults::Y)
                                                .cloned()
                                                .unwrap_or(0.);
                                            let orientation = obs_params
                                                .get(&RobotSensorVariablesFaults::Orientation)
                                                .cloned()
                                                .unwrap_or(0.);

                                            let r = obs_params
                                                .get(&RobotSensorVariablesFaults::R)
                                                .cloned()
                                                .unwrap_or(0.);
                                            let theta = obs_params
                                                .get(&RobotSensorVariablesFaults::Theta)
                                                .cloned()
                                                .unwrap_or(0.);
                                            x += r * theta.cos();
                                            y += r * theta.sin();

                                            let obs = SensorObservation::OrientedRobot(
                                                OrientedRobotObservation {
                                                    name: obs_id,
                                                    labels: Vec::new(),
                                                    pose: Vector3::new(x, y, orientation),
                                                    applied_faults: vec![
                                                        RobotSensorFaultModelConfig::Clutter(
                                                            f.config().clone(),
                                                        ),
                                                    ],
                                                },
                                            );
                                            new_obs.push(obs);
                                        }
                                    }
                                    RobotSensorFaultModelType::Misassociation(f) => {
                                        for (i, obs) in new_obs.iter_mut().enumerate() {
                                            if let SensorObservation::OrientedRobot(observation) =
                                                obs
                                            {
                                                let new_label = f.new_label(
                                                    time + robot_seed + (i as f32) / 1000.,
                                                    observation.name.clone(),
                                                    observation
                                                        .pose
                                                        .fixed_rows::<2>(0)
                                                        .clone_owned(),
                                                    node.environment(),
                                                );
                                                observation.name = new_label;
                                            } else {
                                                unreachable!()
                                            }
                                        }
                                    }
                                    RobotSensorFaultModelType::Misdetection(f) => {
                                        new_obs = new_obs
                                            .iter()
                                            .enumerate()
                                            .filter_map(|(i, obs)| {
                                                if let SensorObservation::OrientedRobot(
                                                    observation,
                                                ) = obs
                                                {
                                                    if f.detected(
                                                        time + robot_seed + (i as f32) / 1000.,
                                                    ) {
                                                        Some(SensorObservation::OrientedRobot(
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
                                "Observation of node {} was filtered out",
                                &other_node_name.to_string()
                            );
                        }
                        observation_list.extend(new_obs);
                    };
                }
                Err(e) => {
                    match e.error_type() {
                        SimbaErrorTypes::ServiceError(
                            ServiceError::Unavailable | ServiceError::Closed,
                        ) => (),
                        _ => log::error!(
                            "Error trying to get real state of node {}: {}",
                            &other_node_name.to_string(),
                            e.detailed_error()
                        ),
                    };
                }
            };
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

impl Recordable<SensorRecord> for RobotSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::RobotSensor(RobotSensorRecord {
            last_time: self.last_time,
        })
    }
}
