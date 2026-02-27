/*!
Provides a [`Sensor`] which can provide linear velocity and angular velocity.
*/

use std::sync::{Arc, Mutex};

use super::fault_models::fault_model::{
    FaultModel, FaultModelConfig, make_fault_model_from_config,
};
use super::{Sensor, SensorObservation, SensorRecord};

use crate::constants::TIME_ROUND;

use crate::environment::Environment;
use crate::errors::SimbaResult;
#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::logger::is_enabled;
use crate::plugin_api::PluginAPI;
use crate::recordable::Recordable;
use crate::sensors::sensor_filters::{
    SensorFilter, SensorFilterConfig, make_sensor_filter_from_config,
};
use crate::simulator::SimulatorConfig;
use crate::state_estimators::{State, StateRecord};
use crate::utils::SharedMutex;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::maths::round_precision;
use crate::utils::periodicity::{Periodicity, PeriodicityConfig};
use log::debug;
use serde_derive::{Deserialize, Serialize};
use simba_macros::config_derives;

extern crate nalgebra as na;

/// Configuration of the [`SpeedSensor`].
#[config_derives]
pub struct SpeedSensorConfig {
    /// Observation period of the sensor.
    pub activation_time: Option<PeriodicityConfig>,
    #[check]
    pub faults: Vec<FaultModelConfig>,
    #[check]
    pub filters: Vec<SensorFilterConfig>,
}

impl Default for SpeedSensorConfig {
    fn default() -> Self {
        Self {
            activation_time: Some(PeriodicityConfig {
                period: crate::config::NumberConfig::Num(0.1),
                offset: None,
                table: None,
            }),
            faults: Vec::new(),
            filters: Vec::new(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for SpeedSensorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Speed sensor")
            .id_salt(format!("speed-sensor-{}", unique_id))
            .show(ui, |ui| {
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
        egui::CollapsingHeader::new("Speed sensor")
            .id_salt(format!("speed-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    if let Some(p) = &self.activation_time {
                        p.show(ui, ctx, unique_id);
                    } else {
                        ui.label("No activation");
                    }
                });
                SensorFilterConfig::show_filters(&self.filters, ui, ctx, unique_id);

                FaultModelConfig::show_faults(&self.faults, ui, ctx, unique_id);
            });
    }
}

/// Record of the [`SpeedSensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SpeedSensorRecord {
    last_time: Option<f32>,
    last_state: StateRecord,
}

impl Default for SpeedSensorRecord {
    fn default() -> Self {
        Self {
            last_time: None,
            last_state: StateRecord::default(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for SpeedSensorRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.label(format!(
            "Last time: {}",
            match self.last_time {
                Some(t) => t.to_string(),
                None => "None".to_string(),
            }
        ));
        ui.label("Last state: ");
        self.last_state.show(ui, ctx, unique_id);
    }
}

/// Observation of the speed.
#[derive(Serialize, Deserialize, Debug, Default, Clone)]
pub struct SpeedObservation {
    pub linear_velocity: f32,
    pub lateral_velocity: f32,
    pub angular_velocity: f32,
    pub applied_faults: Vec<FaultModelConfig>,
}

impl Recordable<SpeedObservationRecord> for SpeedObservation {
    fn record(&self) -> SpeedObservationRecord {
        SpeedObservationRecord {
            linear_velocity: self.linear_velocity,
            lateral_velocity: self.lateral_velocity,
            angular_velocity: self.angular_velocity,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct SpeedObservationRecord {
    pub linear_velocity: f32,
    pub lateral_velocity: f32,
    pub angular_velocity: f32,
}

#[cfg(feature = "gui")]
impl UIComponent for SpeedObservationRecord {
    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label(format!("Linear velocity: {}", self.linear_velocity));
            ui.label(format!("Lateral velocity: {}", self.lateral_velocity));
            ui.label(format!("Angular velocity: {}", self.angular_velocity));
        });
    }
}

/// Sensor which observes the robot's speed
#[derive(Debug)]
pub struct SpeedSensor {
    /// Last state to compute the velocity.
    last_state: State,
    /// Observation period
    activation_time: Option<Periodicity>,
    /// Last observation time.
    last_time: Option<f32>,
    faults: SharedMutex<Vec<Box<dyn FaultModel>>>,
    filters: SharedMutex<Vec<Box<dyn SensorFilter>>>,
}

impl SpeedSensor {
    /// Makes a new [`SpeedSensor`].
    pub fn new() -> Self {
        SpeedSensor::from_config(
            &SpeedSensorConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &"NoName".to_string(),
            &DeterministRandomVariableFactory::default(),
            0.0,
        )
    }

    /// Makes a new [`SpeedSensor`] from the given config.
    pub fn from_config(
        config: &SpeedSensorConfig,
        _plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        robot_name: &String,
        va_factory: &DeterministRandomVariableFactory,
        initial_time: f32,
    ) -> Self {
        let fault_models = Arc::new(Mutex::new(Vec::new()));
        let mut unlock_fault_model = fault_models.lock().unwrap();
        for fault_config in &config.faults {
            unlock_fault_model.push(make_fault_model_from_config(
                fault_config,
                global_config,
                robot_name,
                va_factory,
                initial_time,
            ));
        }
        drop(unlock_fault_model);

        let filters = Arc::new(Mutex::new(Vec::new()));
        let mut unlock_filters = filters.lock().unwrap();
        for filter_config in &config.filters {
            unlock_filters.push(make_sensor_filter_from_config(
                filter_config,
                global_config,
                initial_time,
            ));
        }
        drop(unlock_filters);

        let period = config
            .activation_time
            .as_ref()
            .map(|p| Periodicity::from_config(p, va_factory, initial_time));
        Self {
            last_state: State::new(),
            activation_time: period,
            last_time: None,
            faults: fault_models,
            filters,
        }
    }
}

impl Default for SpeedSensor {
    fn default() -> Self {
        Self::new()
    }
}

use crate::node::Node;

impl Sensor for SpeedSensor {
    fn post_init(&mut self, node: &mut Node, initial_time: f32) -> SimbaResult<()> {
        self.last_state = node
            .physics()
            .expect("Node with Speed sensor should have Physics")
            .read()
            .unwrap()
            .state(initial_time)
            .clone();
        for filter in self.filters.lock().unwrap().iter_mut() {
            filter.post_init(node, initial_time)?;
        }
        for fault_model in self.faults.lock().unwrap().iter_mut() {
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
        let arc_physic = node
            .physics()
            .expect("Node with Speed sensor should have Physics");
        let physic = arc_physic.read().unwrap();
        let state = physic.state(time);

        let obs = SensorObservation::Speed(SpeedObservation {
            linear_velocity: state.velocity.x,
            lateral_velocity: state.velocity.y,
            angular_velocity: state.velocity.z,
            applied_faults: Vec::new(),
        });

        if let Some(obs) = self
            .filters
            .lock()
            .unwrap()
            .iter()
            .try_fold(obs, |obs, filter| filter.filter(time, obs, &state, None))
        {
            observation_list.push(obs);
            for fault_model in self.faults.lock().unwrap().iter_mut() {
                fault_model.add_faults(
                    time,
                    time,
                    &mut observation_list,
                    SensorObservation::Speed(SpeedObservation::default()),
                    node.environment(),
                );
            }
        } else if is_enabled(crate::logger::InternalLog::SensorManagerDetailed) {
            debug!("Speed observation was filtered out");
        }

        self.activation_time.as_mut().map(|p| p.update(time));
        self.last_time = Some(time);
        self.last_state = state.clone();
        observation_list
    }

    fn next_time_step(&self) -> f32 {
        if let Some(activation) = &self.activation_time {
            activation.next_time()
        } else {
            f32::INFINITY
        }
    }
}

impl Recordable<SensorRecord> for SpeedSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::SpeedSensor(SpeedSensorRecord {
            last_time: self.last_time,
            last_state: self.last_state.record(),
        })
    }
}
