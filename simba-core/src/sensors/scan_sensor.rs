use std::sync::Arc;

use log::debug;
use nalgebra::{Rotation2, Vector2, Vector3};
use serde::{Deserialize, Serialize};
use simba_macros::{EnumToString, UIComponent, config_derives, enum_variables};

use crate::{
    config::NumberConfig,
    constants::TIME_ROUND,
    errors::SimbaResult,
    logger::is_enabled,
    node::Node,
    plugin_api::PluginAPI,
    recordable::Recordable,
    sensors::{
        Sensor, SensorObservation, SensorRecord,
        fault_models::{
            additive::{AdditiveFault, AdditiveFaultConfig},
            clutter::{ClutterFault, ClutterFaultConfig},
            external_fault::{ExternalFault, ExternalFaultConfig},
            fault_model::FaultModel,
            misdetection::{MisdetectionFault, MisdetectionFaultConfig},
            python_fault_model::{PythonFaultModel, PythonFaultModelConfig},
        },
        sensor_filters::{self, SensorFilter, SensorFilterConfig, SensorFilterType},
    },
    simulator::SimulatorConfig,
    state_estimators::State,
    utils::{
        determinist_random_variable::DeterministRandomVariableFactory,
        enum_tools::EnumVariables,
        geometry::{is_angle_inside, segments_intersection},
        periodicity::{Periodicity, PeriodicityConfig},
    },
};
#[cfg(feature = "gui")]
use crate::{
    gui::{UIComponent, utils::string_combobox},
    utils::enum_tools::ToVec,
};

enum_variables!(
    ScanSensorVariables;
    Filter, PointFaults, GlobalFaults, GlobalProp, PointProp: X, "x";
    Filter, PointFaults, GlobalFaults, GlobalProp, PointProp: Y, "y" ;
    Filter, PointFaults, PointProp: R, "r";
    Filter, PointFaults, PointProp: Theta, "theta";
    Filter, PointFaults, GlobalFaults, PointProp: RadialVelocity, "radial_velocity", "v";
    GlobalFaults, GlobalProp: Orientation, "orientation", "z";
    Filter, GlobalProp, PointProp: SelfVelocity, "self_velocity";
);

#[config_derives]
#[derive(UIComponent)]
#[show_all = "Faults"]
pub enum ScanSensorFaultModelConfig {
    #[check]
    AdditiveRobotCentered(
        AdditiveFaultConfig<ScanSensorVariablesGlobalFaults, ScanSensorVariablesGlobalProp>,
    ),
    #[check]
    PointAdditiveRobotCentered(
        AdditiveFaultConfig<ScanSensorVariablesPointFaults, ScanSensorVariablesPointProp>,
    ),
    #[check]
    PointAdditiveObservationCentered(
        AdditiveFaultConfig<ScanSensorVariablesPointFaults, ScanSensorVariablesPointProp>,
    ),
    #[check]
    Clutter(ClutterFaultConfig<ScanSensorVariablesPointFaults>),
    #[check]
    Misdetection(MisdetectionFaultConfig),
    #[check]
    PointMisdetection(MisdetectionFaultConfig),
    #[check]
    Python(PythonFaultModelConfig),
    #[check]
    External(ExternalFaultConfig),
}

impl Default for ScanSensorFaultModelConfig {
    fn default() -> Self {
        Self::AdditiveRobotCentered(AdditiveFaultConfig::default())
    }
}

#[derive(Debug, EnumToString)]
pub enum FaultModelTypeScanSensor {
    AdditiveRobotCentered(
        AdditiveFault<ScanSensorVariablesGlobalFaults, ScanSensorVariablesGlobalProp>,
    ),
    PointAdditiveRobotCentered(
        AdditiveFault<ScanSensorVariablesPointFaults, ScanSensorVariablesPointProp>,
    ),
    PointAdditiveObservationCentered(
        AdditiveFault<ScanSensorVariablesPointFaults, ScanSensorVariablesPointProp>,
    ),
    Clutter(ClutterFault<ScanSensorVariablesPointFaults>),
    Misdetection(MisdetectionFault),
    PointMisdetection(MisdetectionFault),
    Python(PythonFaultModel),
    External(ExternalFault),
}

impl FaultModelTypeScanSensor {
    pub fn post_init(
        &mut self,
        node: &mut crate::node::Node,
        initial_time: f32,
    ) -> crate::errors::SimbaResult<()> {
        match self {
            Self::Python(f) => f.post_init(node, initial_time),
            Self::External(f) => f.post_init(node, initial_time),
            Self::AdditiveRobotCentered(_)
            | Self::PointAdditiveRobotCentered(_)
            | Self::PointAdditiveObservationCentered(_)
            | Self::Clutter(_)
            | Self::Misdetection(_)
            | Self::PointMisdetection(_) => Ok(()),
        }
    }
}

#[config_derives(tag_content)]
pub enum RayConfig {
    Regular(usize),
    DegreeTable(Vec<f32>),
    RadianTable(Vec<f32>),
}

#[config_derives]
pub struct ScanSensorConfig {
    /// Max distance of detection.
    pub detection_distance: f32,
    #[check]
    pub rays: RayConfig,
    /// Height of the sensor: will only detect objects at this height or higher.
    pub height: f32,
    /// Observation period of the sensor.
    #[check]
    pub activation_time: Option<PeriodicityConfig>,
    #[check]
    pub faults: Vec<ScanSensorFaultModelConfig>,
    #[check]
    pub filters: Vec<SensorFilterConfig<ScanSensorVariablesFilter>>,
}

impl Check for ScanSensorConfig {
    fn do_check(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.detection_distance < 0. {
            errors.push(format!(
                "Detection distance should be positive, got {}",
                self.detection_distance
            ));
        }
        if self.height < 0. {
            errors.push(format!("Height should be positive, got {}", self.height));
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

impl Default for ScanSensorConfig {
    fn default() -> Self {
        Self {
            detection_distance: 0.,
            rays: RayConfig::Regular(360),
            height: 0.,
            activation_time: Some(PeriodicityConfig {
                period: NumberConfig::Num(0.1),
                ..Default::default()
            }),
            faults: vec![],
            filters: vec![],
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ScanSensorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &crate::simulator::SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Scan sensor")
            .id_salt(format!("scan-sensor-{}", unique_id))
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
                    ui.label("Height:");
                    ui.add(egui::DragValue::new(&mut self.height));
                });

                ui.horizontal(|ui| {
                    ui.label("Rays:");
                    let mut current_str = self.rays.to_string();
                    string_combobox(
                        ui,
                        &RayConfig::to_vec(),
                        &mut current_str,
                        format!("scan-ray-choice-{}", unique_id),
                    );
                    if current_str != self.rays.to_string() {
                        match current_str.as_str() {
                            "Regular" => {
                                self.rays = RayConfig::Regular(360);
                            }
                            "DegreeTable" => {
                                self.rays = RayConfig::DegreeTable(
                                    (0..10).map(|i| (i * 36) as f32).collect(),
                                );
                            }
                            "RadianTable" => {
                                self.rays = RayConfig::RadianTable(
                                    (0..4)
                                        .map(|i| i as f32 * std::f32::consts::PI / 2.0)
                                        .collect(),
                                );
                            }
                            _ => panic!("Where did you find this value?"),
                        };
                    }
                });
                ui.vertical(|ui| match &mut self.rays {
                    RayConfig::Regular(n) => {
                        ui.label("Regular: number of rays: ");
                        ui.add(egui::DragValue::new(n));
                    }
                    RayConfig::DegreeTable(table) | RayConfig::RadianTable(table) => {
                        let mut to_remove = Vec::new();
                        for (i, angle) in table.iter_mut().enumerate() {
                            ui.horizontal(|ui| {
                                ui.add(egui::DragValue::new(angle));
                                if ui.button("X").clicked() {
                                    to_remove.push(i);
                                }
                            });
                        }
                        if !to_remove.is_empty() {
                            for i in to_remove.into_iter().rev() {
                                table.remove(i);
                            }
                        }
                        if ui.button("+").clicked() {
                            table.push(0.);
                        }
                    }
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

                ScanSensorFaultModelConfig::show_all_mut(
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

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new("Scan sensor")
            .id_salt(format!("scan-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Detection distance:");
                    ui.label(format!("{}", self.detection_distance));
                });

                if let Some(p) = &self.activation_time {
                    ui.horizontal(|ui| {
                        p.show(ui, _ctx, unique_id);
                    });
                }

                ui.horizontal(|ui| {
                    ui.label("Height:");
                    ui.label(format!("{}", self.height));
                });

                ui.horizontal(|ui| {
                    ui.label("Rays:");
                    ui.label(self.rays.to_string());
                });

                match &self.rays {
                    RayConfig::Regular(n) => {
                        ui.label(format!("Number of rays: {}", n));
                    }
                    RayConfig::DegreeTable(table) | RayConfig::RadianTable(table) => {
                        for angle in table.iter() {
                            ui.label(format!("- {}", angle));
                        }
                    }
                }

                SensorFilterConfig::show_filters(&self.filters, ui, _ctx, unique_id);

                ScanSensorFaultModelConfig::show_all(&self.faults, ui, _ctx, unique_id);
            });
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ScanSensorRecord {
    last_time: Option<f32>,
}

#[cfg(feature = "gui")]
impl UIComponent for ScanSensorRecord {
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

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ScanObservation {
    pub distances: Vec<f32>,
    pub angles: Vec<f32>,
    pub radial_velocities: Vec<f32>,
    pub applied_faults: Vec<ScanSensorFaultModelConfig>,
}

impl Iterator for ScanObservation {
    type Item = (f32, f32, f32); // distance, angle, radial velocity

    fn next(&mut self) -> Option<Self::Item> {
        if self.distances.is_empty() || self.angles.is_empty() || self.radial_velocities.is_empty()
        {
            None
        } else {
            Some((
                self.distances.remove(0),
                self.angles.remove(0),
                self.radial_velocities.remove(0),
            ))
        }
    }
}

impl Recordable<ScanObservationRecord> for ScanObservation {
    fn record(&self) -> ScanObservationRecord {
        ScanObservationRecord {
            distances: self.distances.clone(),
            angles: self.angles.clone(),
            radial_velocities: self.radial_velocities.clone(),
            applied_faults: self.applied_faults.clone(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScanObservationRecord {
    pub distances: Vec<f32>,
    pub angles: Vec<f32>,
    pub radial_velocities: Vec<f32>,
    pub applied_faults: Vec<ScanSensorFaultModelConfig>,
}

#[cfg(feature = "gui")]
impl UIComponent for ScanObservationRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!("Distances: {:?}", self.distances));
            ui.label(format!("Angles: {:?}", self.angles));
            ui.label(format!("Radial velocities: {:?}", self.radial_velocities));
            if self.applied_faults.is_empty() {
                ui.label("No faults applied.");
            } else {
                ui.label("Applied faults:");
                for fault in &self.applied_faults {
                    ui.label(format!("{:?}", fault));
                }
            }
        });
    }
}

#[derive(Debug)]
pub struct ScanSensor {
    detection_distance: f32,
    /// Orientation of rays in radians
    rays: Vec<f32>,
    height: f32,
    activation_time: Option<Periodicity>,
    faults: Vec<FaultModelTypeScanSensor>,
    filters: Vec<SensorFilterType<ScanSensorVariablesFilter>>,
    last_time: Option<f32>,
}

impl ScanSensor {
    pub fn from_config(
        config: &ScanSensorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        initial_time: f32,
    ) -> SimbaResult<Self> {
        let rays = match &config.rays {
            RayConfig::Regular(n) => (0..*n)
                .map(|i| i as f32 * 2.0 * std::f32::consts::PI / (*n as f32))
                .collect(),
            RayConfig::DegreeTable(table) => table.iter().map(|d| d.to_radians()).collect(),
            RayConfig::RadianTable(table) => table.clone(),
        };

        let mut fault_models = Vec::new();
        for fault_config in &config.faults {
            fault_models.push(match &fault_config {
                ScanSensorFaultModelConfig::AdditiveRobotCentered(cfg) => {
                    FaultModelTypeScanSensor::AdditiveRobotCentered(AdditiveFault::from_config(
                        cfg,
                        va_factory,
                        initial_time,
                    ))
                }
                ScanSensorFaultModelConfig::PointAdditiveRobotCentered(cfg) => {
                    FaultModelTypeScanSensor::PointAdditiveRobotCentered(
                        AdditiveFault::from_config(cfg, va_factory, initial_time),
                    )
                }
                ScanSensorFaultModelConfig::PointAdditiveObservationCentered(cfg) => {
                    FaultModelTypeScanSensor::PointAdditiveObservationCentered(
                        AdditiveFault::from_config(cfg, va_factory, initial_time),
                    )
                }
                ScanSensorFaultModelConfig::Clutter(cfg) => FaultModelTypeScanSensor::Clutter(
                    ClutterFault::from_config(cfg, va_factory, initial_time),
                ),
                ScanSensorFaultModelConfig::Misdetection(cfg) => {
                    FaultModelTypeScanSensor::Misdetection(MisdetectionFault::from_config(
                        cfg,
                        va_factory,
                        initial_time,
                    ))
                }
                ScanSensorFaultModelConfig::PointMisdetection(cfg) => {
                    FaultModelTypeScanSensor::PointMisdetection(MisdetectionFault::from_config(
                        cfg,
                        va_factory,
                        initial_time,
                    ))
                }
                ScanSensorFaultModelConfig::Python(cfg) => FaultModelTypeScanSensor::Python(
                    PythonFaultModel::from_config(cfg, global_config, initial_time)
                        .expect("Failed to create Python Fault Model"),
                ),
                ScanSensorFaultModelConfig::External(cfg) => FaultModelTypeScanSensor::External(
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

        Ok(Self {
            detection_distance: config.detection_distance,
            rays,
            height: config.height,
            activation_time: config
                .activation_time
                .as_ref()
                .map(|p| Periodicity::from_config(p, va_factory, initial_time)),
            faults: fault_models,
            filters: config.filters.iter().try_fold(Vec::new(), |mut acc, f| {
                sensor_filters::make_sensor_filter_from_config(
                    f,
                    plugin_api,
                    global_config,
                    va_factory,
                    initial_time,
                )
                .map(|filter| {
                    acc.push(filter);
                    acc
                })
            })?,
            last_time: None,
        })
    }
}

impl Sensor for ScanSensor {
    fn post_init(&mut self, _node: &mut Node, _initial_time: f32) -> SimbaResult<()> {
        for filter in &mut self.filters {
            match filter {
                SensorFilterType::PythonFilter(f) => f.post_init(_node, _initial_time)?,
                SensorFilterType::External(f) => f.post_init(_node, _initial_time)?,
                _ => (), // No post_init needed for other filters for now
            }
        }
        Ok(())
    }

    fn next_time_step(&self) -> f32 {
        self.activation_time
            .as_ref()
            .map(|p| p.next_time())
            .unwrap_or(f32::INFINITY)
    }

    fn get_observations(&mut self, node: &mut Node, time: f32) -> Vec<SensorObservation> {
        if let Some(last_time) = self.last_time
            && (time - last_time).abs() < TIME_ROUND
        {
            return Vec::new();
        }
        let environment = node.environment();

        let state = if let Some(arc_physics) = node.physics() {
            let physics = arc_physics.read().unwrap();
            physics.state(time).clone()
        } else {
            State::new() // 0
        };
        let position = state.pose.fixed_rows::<2>(0).into_owned();

        // List of observable landmarks with their angle ranges (in sensor frame) and extremities (in world frame)
        let observable_landmarks = environment
            .get_observable_landmarks(
                &position,
                Some(self.height),
                self.detection_distance,
                Some(node.name()),
            )
            .into_iter()
            .filter_map(|l| {
                if l.height >= self.height {
                    let (pt1, pt2) = l.extremities();
                    let angle1 = (pt1.y - position.y).atan2(pt1.x - position.x) - state.pose.z;
                    let angle2 = (pt2.y - position.y).atan2(pt2.x - position.x) - state.pose.z;
                    // Difference between angles should be in [-pi, pi] and we want angle1 to be the smallest
                    let (angle1, angle2) = if angle1 - angle2 > std::f32::consts::PI {
                        (angle1, angle2 + 2. * std::f32::consts::PI)
                    } else if angle2 - angle1 > std::f32::consts::PI {
                        (angle2, angle1 + 2. * std::f32::consts::PI)
                    } else if angle1 > angle2 {
                        (angle2, angle1)
                    } else {
                        (angle1, angle2)
                    };
                    Some((l, angle1, angle2, pt1, pt2))
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();

        if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
            debug!("Scan Sensor - Observable landmarks:");
            for (l, angle1, angle2, _, _) in &observable_landmarks {
                debug!(
                    "- Landmark {}: angle range [{:.2}, {:.2}]",
                    l.id, angle1, angle2
                );
            }
        }

        let mut observation = ScanObservation::default();
        // Ray casting
        for ray in &self.rays {
            let world_ray_angle = state.pose.z + ray;
            let ray_direction =
                nalgebra::Vector2::new(world_ray_angle.cos(), world_ray_angle.sin());

            let candidates = observable_landmarks
                .iter()
                .filter_map(|l| {
                    if is_angle_inside(*ray, l.1, l.2) {
                        Some((&l.0, l.3, l.4))
                    } else {
                        None
                    }
                })
                .collect::<Vec<_>>();

            let ray_end = position + ray_direction * self.detection_distance;
            let closest_intersection = candidates.iter().fold(None, |acc, (l, pt1, pt2)| {
                let intersection = segments_intersection(
                    &position,
                    &ray_end,
                    &pt1.fixed_rows::<2>(0).clone_owned(),
                    &pt2.fixed_rows::<2>(0).clone_owned(),
                )?;
                let distance = (intersection - position).norm();
                if distance <= self.detection_distance {
                    match acc {
                        None => Some((distance, l, intersection)),
                        Some((prev_distance, _, _)) if distance < prev_distance => {
                            Some((distance, l, intersection))
                        }
                        _ => acc,
                    }
                } else {
                    acc
                }
            });
            if let Some((distance, _l, intersection)) = closest_intersection {
                observation.distances.push(distance);
                let angle =
                    (intersection.y - position.y).atan2(intersection.x - position.x) - state.pose.z;
                observation.angles.push(angle);

                let velocity_vector = Vector2::new(
                    state.velocity[0] * state.pose.z.cos(),
                    state.velocity[0] * state.pose.z.sin(),
                );
                let radial_velocity = velocity_vector.dot(&ray_direction);

                observation.radial_velocities.push(radial_velocity);
            }
        }

        let initial_observation = SensorObservation::Scan(observation);
        if let Some(activation_time) = self.activation_time.as_mut() {
            activation_time.update(time);
        }
        self.last_time = Some(time);

        let mut keep_observation = Some(initial_observation);
        for filter in &self.filters {
            if let Some(obs) = keep_observation {
                keep_observation = match filter {
                    SensorFilterType::PythonFilter(f) => f.filter(time, obs, &state, None),
                    SensorFilterType::External(f) => f.filter(time, obs, &state, None),
                    SensorFilterType::RangeFilter(f) => {
                        if let SensorObservation::Scan(obs) = obs {
                            let mut obs = obs;
                            let mut to_remove = Vec::new();
                            for (i, (r, theta, v)) in obs
                                .distances
                                .iter()
                                .zip(obs.angles.iter().zip(obs.radial_velocities.iter()))
                                .map(|(r, (theta, v))| (r, theta, v))
                                .enumerate()
                            {
                                if f.match_exclusion(&ScanSensorVariablesFilter::mapped_values(
                                    |variant| match variant {
                                        ScanSensorVariablesFilter::R => *r,
                                        ScanSensorVariablesFilter::Theta => *theta,
                                        ScanSensorVariablesFilter::RadialVelocity => *v,
                                        ScanSensorVariablesFilter::SelfVelocity => {
                                            state.velocity.fixed_rows::<2>(0).norm()
                                        }
                                        ScanSensorVariablesFilter::X => *r * theta.cos(),
                                        ScanSensorVariablesFilter::Y => *r * theta.sin(),
                                    },
                                )) {
                                    to_remove.push(i);
                                }
                            }
                            for &i in to_remove.iter().rev() {
                                obs.distances.remove(i);
                                obs.angles.remove(i);
                                obs.radial_velocities.remove(i);
                            }
                            Some(SensorObservation::Scan(obs))
                        } else {
                            unreachable!()
                        }
                    }
                    _ => unimplemented!(
                        "{} filter not implemented for ScanSensor",
                        filter.to_string()
                    ),
                }
            } else {
                break;
            }
        }

        if let Some(filtered_observation) = keep_observation {
            let mut observations = vec![filtered_observation];
            for fault_model in self.faults.iter_mut() {
                match fault_model {
                    FaultModelTypeScanSensor::Python(f) => f.add_faults(
                        time,
                        time,
                        &mut observations,
                        SensorObservation::Scan(ScanObservation::default()),
                        node.environment(),
                    ),
                    FaultModelTypeScanSensor::External(f) => f.add_faults(
                        time,
                        time,
                        &mut observations,
                        SensorObservation::Scan(ScanObservation::default()),
                        node.environment(),
                    ),
                    FaultModelTypeScanSensor::AdditiveRobotCentered(f) => {
                        let obs_list_len = observations.len();
                        for (i, obs) in observations
                            .iter_mut()
                            .map(|o| {
                                if let SensorObservation::Scan(observation) = o {
                                    observation
                                } else {
                                    unreachable!()
                                }
                            })
                            .enumerate()
                        {
                            let seed = time + i as f32 / (100. * obs_list_len as f32);
                            let adding_values = f.add_faults(
                                seed,
                                ScanSensorVariablesGlobalFaults::mapped_values(|variant| {
                                    match variant {
                                        ScanSensorVariablesGlobalFaults::X => 0.,
                                        ScanSensorVariablesGlobalFaults::Y => 0.,
                                        ScanSensorVariablesGlobalFaults::RadialVelocity => 0.,
                                        ScanSensorVariablesGlobalFaults::Orientation => 0.,
                                    }
                                }),
                                &ScanSensorVariablesGlobalProp::mapped_values(|variant| {
                                    match variant {
                                        ScanSensorVariablesGlobalProp::Orientation => state.pose.z,
                                        ScanSensorVariablesGlobalProp::X => state.pose.x,
                                        ScanSensorVariablesGlobalProp::Y => state.pose.y,
                                        ScanSensorVariablesGlobalProp::SelfVelocity => {
                                            state.velocity.fixed_rows::<2>(0).norm()
                                        }
                                    }
                                }),
                            );
                            let mut addition_vector = Vector3::zeros();
                            if let Some(x) = adding_values.get(&ScanSensorVariablesGlobalFaults::X)
                            {
                                addition_vector.x = *x;
                            }
                            if let Some(y) = adding_values.get(&ScanSensorVariablesGlobalFaults::Y)
                            {
                                addition_vector.y = *y;
                            }
                            if let Some(z) =
                                adding_values.get(&ScanSensorVariablesGlobalFaults::Orientation)
                            {
                                addition_vector.z = *z;
                            }
                            let rotation = Rotation2::new(addition_vector.z);
                            let obs_clone = obs.clone();
                            obs.distances.clear();
                            obs.angles.clear();
                            obs.radial_velocities.clear();
                            for (d, angle, v) in obs_clone {
                                let point = Vector2::new(d * angle.cos(), d * angle.sin());
                                let new_point =
                                    rotation * point + addition_vector.fixed_rows::<2>(0);
                                let d = new_point.norm();
                                let angle = new_point.y.atan2(new_point.x);
                                let v = if let Some(radial_velocity) = adding_values
                                    .get(&ScanSensorVariablesGlobalFaults::RadialVelocity)
                                {
                                    v + radial_velocity
                                } else {
                                    v
                                };
                                obs.distances.push(d);
                                obs.angles.push(angle);
                                obs.radial_velocities.push(v);
                            }
                        }
                    }
                    FaultModelTypeScanSensor::PointAdditiveRobotCentered(f) => {
                        let obs_list_len = observations.len();
                        for (i, obs) in observations
                            .iter_mut()
                            .map(|o| {
                                if let SensorObservation::Scan(observation) = o {
                                    observation
                                } else {
                                    unreachable!()
                                }
                            })
                            .enumerate()
                        {
                            let seed = time + i as f32 / (100. * obs_list_len as f32);
                            let obs_clone = obs.clone();
                            obs.distances.clear();
                            obs.angles.clear();
                            obs.radial_velocities.clear();
                            for (j, (d, angle, v)) in obs_clone.into_iter().enumerate() {
                                let point_seed = f32::powi(seed, j as i32 + 1);
                                let new_values = f.add_faults(
                                    point_seed,
                                    ScanSensorVariablesPointFaults::mapped_values(|variant| {
                                        match variant {
                                            ScanSensorVariablesPointFaults::X => d * angle.cos(),
                                            ScanSensorVariablesPointFaults::Y => d * angle.sin(),
                                            ScanSensorVariablesPointFaults::R => d,
                                            ScanSensorVariablesPointFaults::Theta => angle,
                                            ScanSensorVariablesPointFaults::RadialVelocity => v,
                                        }
                                    }),
                                    &ScanSensorVariablesPointProp::mapped_values(|variant| {
                                        match variant {
                                            ScanSensorVariablesPointProp::R => d,
                                            ScanSensorVariablesPointProp::Theta => angle,
                                            ScanSensorVariablesPointProp::X => d * angle.cos(),
                                            ScanSensorVariablesPointProp::Y => d * angle.sin(),
                                            ScanSensorVariablesPointProp::RadialVelocity => v,
                                            ScanSensorVariablesPointProp::SelfVelocity => {
                                                state.velocity.fixed_rows::<2>(0).norm()
                                            }
                                        }
                                    }),
                                );
                                let mut d = if let Some(new_r) =
                                    new_values.get(&ScanSensorVariablesPointFaults::R)
                                {
                                    *new_r
                                } else {
                                    d
                                };
                                let mut angle = if let Some(new_angle) =
                                    new_values.get(&ScanSensorVariablesPointFaults::Theta)
                                {
                                    *new_angle
                                } else {
                                    angle
                                };
                                let new_x = if let Some(new_x) =
                                    new_values.get(&ScanSensorVariablesPointFaults::X)
                                {
                                    *new_x
                                } else {
                                    d * angle.cos()
                                };
                                let new_y = if let Some(new_y) =
                                    new_values.get(&ScanSensorVariablesPointFaults::Y)
                                {
                                    *new_y
                                } else {
                                    d * angle.sin()
                                };
                                d = (new_x.powi(2) + new_y.powi(2)).sqrt();
                                angle = new_y.atan2(new_x);
                                let v = if let Some(radial_velocity) =
                                    new_values.get(&ScanSensorVariablesPointFaults::RadialVelocity)
                                {
                                    *radial_velocity
                                } else {
                                    v
                                };
                                obs.distances.push(d);
                                obs.angles.push(angle);
                                obs.radial_velocities.push(v);
                            }
                            obs.applied_faults.push(
                                ScanSensorFaultModelConfig::PointAdditiveRobotCentered(
                                    f.config().clone(),
                                ),
                            );
                        }
                    }
                    FaultModelTypeScanSensor::PointAdditiveObservationCentered(f) => {
                        let obs_list_len = observations.len();
                        for (i, obs) in observations
                            .iter_mut()
                            .map(|o| {
                                if let SensorObservation::Scan(observation) = o {
                                    observation
                                } else {
                                    unreachable!()
                                }
                            })
                            .enumerate()
                        {
                            let seed = time + i as f32 / (100. * obs_list_len as f32);
                            let obs_clone = obs.clone();
                            obs.distances.clear();
                            obs.angles.clear();
                            obs.radial_velocities.clear();
                            for (j, (d, angle, v)) in obs_clone.into_iter().enumerate() {
                                let point_seed = f32::powi(seed, j as i32 + 1);
                                let adding_values = f.add_faults(
                                    point_seed,
                                    ScanSensorVariablesPointFaults::mapped_values(|variant| {
                                        match variant {
                                            ScanSensorVariablesPointFaults::X => 0.,
                                            ScanSensorVariablesPointFaults::Y => 0.,
                                            ScanSensorVariablesPointFaults::R => 0.,
                                            ScanSensorVariablesPointFaults::Theta => 0.,
                                            ScanSensorVariablesPointFaults::RadialVelocity => 0.,
                                        }
                                    }),
                                    &ScanSensorVariablesPointProp::mapped_values(|variant| {
                                        match variant {
                                            ScanSensorVariablesPointProp::R => d,
                                            ScanSensorVariablesPointProp::Theta => angle,
                                            ScanSensorVariablesPointProp::X => d * angle.cos(),
                                            ScanSensorVariablesPointProp::Y => d * angle.sin(),
                                            ScanSensorVariablesPointProp::RadialVelocity => v,
                                            ScanSensorVariablesPointProp::SelfVelocity => {
                                                state.velocity.fixed_rows::<2>(0).norm()
                                            }
                                        }
                                    }),
                                );
                                let add_x = if let Some(x) =
                                    adding_values.get(&ScanSensorVariablesPointFaults::X)
                                {
                                    *x
                                } else {
                                    0.
                                };
                                let add_y = if let Some(y) =
                                    adding_values.get(&ScanSensorVariablesPointFaults::Y)
                                {
                                    *y
                                } else {
                                    0.
                                };
                                let add_r = if let Some(r) =
                                    adding_values.get(&ScanSensorVariablesPointFaults::R)
                                {
                                    *r
                                } else {
                                    0.
                                };
                                let add_angle = if let Some(angle) =
                                    adding_values.get(&ScanSensorVariablesPointFaults::Theta)
                                {
                                    *angle
                                } else {
                                    0.
                                };

                                let add_x = add_x + add_r * (angle + add_angle).cos();
                                let add_y = add_y + add_r * (angle + add_angle).sin();
                                let new_x = d * angle.cos() + add_x;
                                let new_y = d * angle.sin() + add_y;
                                let d = (new_x.powi(2) + new_y.powi(2)).sqrt();
                                let angle = new_y.atan2(new_x);
                                let v = if let Some(radial_velocity) = adding_values
                                    .get(&ScanSensorVariablesPointFaults::RadialVelocity)
                                {
                                    v + *radial_velocity
                                } else {
                                    v
                                };
                                obs.distances.push(d);
                                obs.angles.push(angle);
                                obs.radial_velocities.push(v);
                            }
                            obs.applied_faults.push(
                                ScanSensorFaultModelConfig::PointAdditiveObservationCentered(
                                    f.config().clone(),
                                ),
                            );
                        }
                    }
                    FaultModelTypeScanSensor::Clutter(f) => {
                        let obs_list_len = observations.len();
                        for (i, obs) in observations
                            .iter_mut()
                            .map(|o| {
                                if let SensorObservation::Scan(observation) = o {
                                    observation
                                } else {
                                    unreachable!()
                                }
                            })
                            .enumerate()
                        {
                            let new_obs_from_clutter =
                                f.add_faults(time, i as f32 / (1000. * obs_list_len as f32));
                            for (_, obs_params) in new_obs_from_clutter {
                                let mut x = obs_params
                                    .get(&ScanSensorVariablesPointFaults::X)
                                    .cloned()
                                    .unwrap_or(0.);
                                let mut y = obs_params
                                    .get(&ScanSensorVariablesPointFaults::Y)
                                    .cloned()
                                    .unwrap_or(0.);

                                let r = obs_params
                                    .get(&ScanSensorVariablesPointFaults::R)
                                    .cloned()
                                    .unwrap_or(0.);
                                let theta = obs_params
                                    .get(&ScanSensorVariablesPointFaults::Theta)
                                    .cloned()
                                    .unwrap_or(0.);
                                let v = obs_params
                                    .get(&ScanSensorVariablesPointFaults::RadialVelocity)
                                    .cloned()
                                    .unwrap_or(0.);
                                x += r * theta.cos();
                                y += r * theta.sin();
                                let r = (x.powi(2) + y.powi(2)).sqrt();
                                let theta = y.atan2(x);
                                obs.distances.push(r);
                                obs.angles.push(theta);
                                obs.radial_velocities.push(v);
                            }
                            obs.applied_faults
                                .push(ScanSensorFaultModelConfig::Clutter(f.config().clone()));
                        }
                    }
                    FaultModelTypeScanSensor::Misdetection(f) => {
                        observations = observations
                            .into_iter()
                            .enumerate()
                            .filter_map(|(i, obs)| {
                                if let SensorObservation::Scan(observation) = obs {
                                    if f.detected(time + (i as f32) / 1000.) {
                                        Some(SensorObservation::Scan(observation))
                                    } else {
                                        None
                                    }
                                } else {
                                    unreachable!()
                                }
                            })
                            .collect();
                    }
                    FaultModelTypeScanSensor::PointMisdetection(f) => {
                        let obs_list_len = observations.len();
                        for (i, obs) in observations
                            .iter_mut()
                            .map(|o| {
                                if let SensorObservation::Scan(observation) = o {
                                    observation
                                } else {
                                    unreachable!()
                                }
                            })
                            .enumerate()
                        {
                            let seed = time + i as f32 / (100. * obs_list_len as f32);
                            let obs_clone = obs.clone();
                            obs.distances.clear();
                            obs.angles.clear();
                            obs.radial_velocities.clear();
                            for (j, (d, angle, v)) in obs_clone.into_iter().enumerate() {
                                if f.detected(seed + (j as f32) / 1000.) {
                                    obs.distances.push(d);
                                    obs.angles.push(angle);
                                    obs.radial_velocities.push(v);
                                }
                            }
                        }
                    }
                }
            }
            observations
        } else {
            if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
                debug!("Scan Observation filtered out");
            }
            Vec::new()
        }
    }
}

impl Recordable<SensorRecord> for ScanSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::ScanSensor(ScanSensorRecord {
            last_time: self.last_time,
        })
    }
}
