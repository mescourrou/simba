/*!
Provides a [`Sensor`] which can provide linear velocity and angular velocity.
*/

use std::sync::{Arc, Mutex};

use super::fault_models::fault_model::{
    make_fault_model_from_config, FaultModel, FaultModelConfig,
};
use super::sensor::{Sensor, SensorObservation, SensorRecord};

use crate::constants::TIME_ROUND;

#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::logger::is_enabled;
use crate::plugin_api::PluginAPI;
use crate::recordable::Recordable;
use crate::sensors::sensor_filters::{
    make_sensor_filter_from_config, SensorFilter, SensorFilterConfig,
};
use crate::simulator::SimulatorConfig;
use crate::state_estimators::state_estimator::{State, StateRecord};
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::geometry::smallest_theta_diff;
use crate::utils::maths::round_precision;
use config_checker::macros::Check;
use log::debug;
use serde_derive::{Deserialize, Serialize};

extern crate nalgebra as na;

/// Configuration of the [`OdometrySensor`].
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct OdometrySensorConfig {
    /// Observation period of the sensor.
    #[check[ge(0.)]]
    pub period: f32,
    #[check]
    pub faults: Vec<FaultModelConfig>,
    #[check]
    pub filters: Vec<SensorFilterConfig>,
}

impl Default for OdometrySensorConfig {
    fn default() -> Self {
        Self {
            period: 0.1,
            faults: Vec::new(),
            filters: Vec::new(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for OdometrySensorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Odometry sensor")
            .id_salt(format!("odometry-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Period:");
                    if self.period < TIME_ROUND {
                        self.period = TIME_ROUND;
                    }
                    ui.add(egui::DragValue::new(&mut self.period));
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

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new("Odometry sensor")
            .id_salt(format!("odometry-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(format!("Period: {}", self.period));
                });

                SensorFilterConfig::show_filters(&self.filters, ui, ctx, unique_id);

                FaultModelConfig::show_faults(&self.faults, ui, ctx, unique_id);
            });
    }
}

/// Record of the [`OdometrySensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct OdometrySensorRecord {
    last_time: f32,
    last_state: StateRecord,
}

impl Default for OdometrySensorRecord {
    fn default() -> Self {
        Self {
            last_time: 0.,
            last_state: StateRecord::default(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for OdometrySensorRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.label(format!("Last time: {}", self.last_time));
        ui.label("Last state: ");
        self.last_state.show(ui, ctx, unique_id);
    }
}

/// Observation of the odometry.
#[derive(Serialize, Deserialize, Debug, Default, Clone)]
pub struct OdometryObservation {
    pub linear_velocity: f32,
    pub angular_velocity: f32,
    pub applied_faults: Vec<FaultModelConfig>,
}

impl Recordable<OdometryObservationRecord> for OdometryObservation {
    fn record(&self) -> OdometryObservationRecord {
        OdometryObservationRecord {
            linear_velocity: self.linear_velocity,
            angular_velocity: self.angular_velocity,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct OdometryObservationRecord {
    pub linear_velocity: f32,
    pub angular_velocity: f32,
}

#[cfg(feature = "gui")]
impl UIComponent for OdometryObservationRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!("Linear velocity: {}", self.linear_velocity));
            ui.label(format!("Angular velocity: {}", self.angular_velocity));
        });
    }
}

/// Sensor which observes the robot's odometry
#[derive(Debug)]
pub struct OdometrySensor {
    /// Last state to compute the velocity.
    last_state: State,
    /// Observation period
    period: f32,
    /// Last observation time.
    last_time: f32,
    faults: Arc<Mutex<Vec<Box<dyn FaultModel>>>>,
    filters: Arc<Mutex<Vec<Box<dyn SensorFilter>>>>,
}

impl OdometrySensor {
    /// Makes a new [`OdometrySensor`].
    pub fn new() -> Self {
        OdometrySensor::from_config(
            &OdometrySensorConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &"NoName".to_string(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Makes a new [`OdometrySensor`] from the given config.
    pub fn from_config(
        config: &OdometrySensorConfig,
        _plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        robot_name: &String,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        let fault_models = Arc::new(Mutex::new(Vec::new()));
        let mut unlock_fault_model = fault_models.lock().unwrap();
        for fault_config in &config.faults {
            unlock_fault_model.push(make_fault_model_from_config(
                fault_config,
                global_config,
                robot_name,
                va_factory,
            ));
        }
        drop(unlock_fault_model);

        let filters = Arc::new(Mutex::new(Vec::new()));
        let mut unlock_filters = filters.lock().unwrap();
        for filter_config in &config.filters {
            unlock_filters.push(make_sensor_filter_from_config(filter_config));
        }
        drop(unlock_filters);

        Self {
            last_state: State::new(),
            period: config.period,
            last_time: 0.,
            faults: fault_models,
            filters,
        }
    }
}

impl Default for OdometrySensor {
    fn default() -> Self {
        Self::new()
    }
}

use crate::node::Node;

impl Sensor for OdometrySensor {
    fn init(&mut self, robot: &mut Node) {
        self.last_state = robot
            .physics()
            .expect("Node with GNSS sensor should have Physics")
            .read()
            .unwrap()
            .state(0.)
            .clone();
    }

    fn get_observations(&mut self, robot: &mut Node, time: f32) -> Vec<SensorObservation> {
        let mut observation_list = Vec::<SensorObservation>::new();
        if (time - self.next_time_step()).abs() >= TIME_ROUND {
            return observation_list;
        }
        let arc_physic = robot
            .physics()
            .expect("Node with Odometry sensor should have Physics");
        let physic = arc_physic.read().unwrap();
        let state = physic.state(time);

        let dt = time - self.last_time;

        let obs = SensorObservation::Odometry(OdometryObservation {
            linear_velocity: state.velocity,
            angular_velocity: smallest_theta_diff(state.pose.z, self.last_state.pose.z) / dt,
            applied_faults: Vec::new(),
        });

        if let Some(obs) = self
            .filters
            .lock()
            .unwrap()
            .iter()
            .try_fold(obs, |obs, filter| filter.filter(obs, &state, None))
        {
            observation_list.push(obs);
            for fault_model in self.faults.lock().unwrap().iter() {
                fault_model.add_faults(
                    time,
                    self.period,
                    &mut observation_list,
                    SensorObservation::Odometry(OdometryObservation::default()),
                );
            }
        } else if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
            debug!("Odometry observation was filtered out");
        }

        self.last_time = time;
        self.last_state = state.clone();
        observation_list
    }

    fn next_time_step(&self) -> f32 {
        round_precision(self.last_time + self.period, TIME_ROUND).unwrap()
    }

    fn period(&self) -> f32 {
        self.period
    }
}

impl Recordable<SensorRecord> for OdometrySensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::OdometrySensor(OdometrySensorRecord {
            last_time: self.last_time,
            last_state: self.last_state.record(),
        })
    }
}
