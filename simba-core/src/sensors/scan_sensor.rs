use std::{iter::Scan, sync::Arc};

use log::debug;
use nalgebra::{Rotation2, Rotation3, Vector2};
use serde::{Deserialize, Serialize};
use simba_macros::config_derives;

use crate::{
    config::NumberConfig,
    constants::TIME_ROUND,
    enum_variables,
    errors::SimbaResult,
    logger::is_enabled,
    node::Node,
    plugin_api::PluginAPI,
    recordable::Recordable,
    sensors::{
        Sensor, SensorObservation, SensorRecord,
        fault_models::fault_model::{self, FaultModel, FaultModelConfig},
        sensor_filters::{self, SensorFilter, SensorFilterConfig, SensorFilterType},
    },
    simulator::SimulatorConfig,
    state_estimators::State,
    utils::{
        determinist_random_variable::DeterministRandomVariableFactory, enum_tools::EnumVariables, geometry::{is_angle_inside, segments_intersection}, periodicity::{Periodicity, PeriodicityConfig}
    },
};
#[cfg(feature = "gui")]
use crate::{
    gui::{UIComponent, utils::string_combobox},
    utils::enum_tools::ToVec,
};

enum_variables!(
    ScanSensorVariables;
    R, "r";
    Theta, "theta";
    RadialVelocity, "radial_velocity", "v";
    SelfVelocity, "self_velocity";
    X, "x";
    Y, "y";
);

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
    pub faults: Vec<FaultModelConfig>,
    #[check]
    pub filters: Vec<SensorFilterConfig<ScanSensorVariables>>,
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

                FaultModelConfig::show_faults_mut(
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
                        for (i, angle) in table.iter().enumerate() {
                            ui.label(format!("- {}", angle));
                        }
                    }
                }

                SensorFilterConfig::show_filters(&self.filters, ui, _ctx, unique_id);

                FaultModelConfig::show_faults(&self.faults, ui, _ctx, unique_id);
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScanObservation {
    pub distances: Vec<f32>,
    pub angles: Vec<f32>,
    pub radial_velocities: Vec<f32>,
    pub applied_faults: Vec<FaultModelConfig>,
}

impl Default for ScanObservation {
    fn default() -> Self {
        Self {
            distances: Vec::new(),
            angles: Vec::new(),
            radial_velocities: Vec::new(),
            applied_faults: Vec::new(),
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
    pub applied_faults: Vec<FaultModelConfig>,
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
    faults: Vec<Box<dyn FaultModel>>,
    filters: Vec<SensorFilterType<ScanSensorVariables>>,
    last_time: Option<f32>,
}

impl ScanSensor {
    pub fn from_config(
        config: &ScanSensorConfig,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        node_name: &str,
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
        Ok(Self {
            detection_distance: config.detection_distance,
            rays,
            height: config.height,
            activation_time: config
                .activation_time
                .as_ref()
                .map(|p| Periodicity::from_config(p, va_factory, initial_time)),
            faults: config
                .faults
                .iter()
                .map(|f| {
                    fault_model::make_fault_model_from_config(
                        f,
                        global_config,
                        node_name,
                        va_factory,
                        initial_time,
                    )
                })
                .collect(),
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

        let rotation_matrix = Rotation2::new(state.pose.z);

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
        self.activation_time.as_mut().map(|p| p.update(time));
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
                                if f.match_exclusion(&ScanSensorVariables::mapped_values(
                                    |variant| match variant {
                                        ScanSensorVariables::R => *r,
                                        ScanSensorVariables::Theta => *theta,
                                        ScanSensorVariables::RadialVelocity => *v,
                                        ScanSensorVariables::SelfVelocity => {
                                            state.velocity.fixed_rows::<2>(0).norm()
                                        }
                                        ScanSensorVariables::X => *r * theta.cos(),
                                        ScanSensorVariables::Y => *r * theta.sin(),
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
                fault_model.add_faults(
                    time,
                    time,
                    &mut observations,
                    SensorObservation::Scan(ScanObservation::default()),
                    node.environment(),
                );
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
