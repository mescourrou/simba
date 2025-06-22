/*!
Provide the [`SensorManager`], which owns the different [`Sensor`]s and get the
available observations.
*/

extern crate confy;
use config_checker::macros::Check;
use core::f32;
use log::debug;
use serde_derive::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::{Arc, RwLock};

#[cfg(feature = "gui")]
use crate::gui::{
    utils::{string_checkbox, text_singleline_with_apply},
    UIComponent,
};
use crate::logger::is_enabled;
use crate::networking::message_handler::MessageHandler;
use crate::node::Node;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::{simulator::SimulatorConfig, stateful::Stateful};

use super::gnss_sensor::GNSSSensor;
use super::odometry_sensor::{OdometrySensor, OdometrySensorConfig};
use super::robot_sensor::RobotSensor;
use super::sensor::{Observation, ObservationRecord};
use super::{
    oriented_landmark_sensor::OrientedLandmarkSensor,
    sensor::{Sensor, SensorConfig, SensorRecord},
};
use crate::plugin_api::PluginAPI;

#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct ManagedSensorConfig {
    pub name: String,
    pub send_to: Vec<String>,
    pub config: SensorConfig,
}

impl Default for ManagedSensorConfig {
    fn default() -> Self {
        Self {
            name: "some_sensor".to_string(),
            send_to: Vec::new(),
            config: SensorConfig::OdometrySensor(OdometrySensorConfig::default()),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for ManagedSensorConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new(&self.name)
            .id_source(format!("managed-sensor-{}", unique_id).as_str())
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Name: ");
                    text_singleline_with_apply(
                        ui,
                        format!("managed-sensor-name-key-{}", unique_id).as_str(),
                        buffer_stack,
                        &mut self.name,
                    );
                });

                let mut node_list = Vec::from_iter(
                    global_config.robots.iter().map(|x| x.name.clone()).chain(
                        global_config
                            .computation_units
                            .iter()
                            .map(|x| x.name.clone()),
                    ),
                );
                if let Some(idx) = node_list
                    .iter()
                    .position(|x| x == current_node_name.unwrap())
                {
                    node_list.remove(idx);
                }
                ui.horizontal_wrapped(|ui| {
                    ui.label("Send to:");
                    string_checkbox(ui, &node_list, &mut self.send_to);
                });

                self.config.show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );
            });
    }

    fn show(
        &self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new(&self.name)
            .id_source(format!("managed-sensor-{}", unique_id).as_str())
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(format!("Name: {}", self.name));
                });

                ui.horizontal_wrapped(|ui| {
                    ui.label("Send to: ");
                    for to in &self.send_to {
                        ui.label(format!("{}, ", to));
                    }
                });

                self.config.show(
                    ui,
                    ctx,
                    unique_id,
                );
            });
    }
}

/// Configuration listing all the [`SensorConfig`]s.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct SensorManagerConfig {
    #[check]
    pub sensors: Vec<ManagedSensorConfig>,
}

impl Default for SensorManagerConfig {
    fn default() -> Self {
        Self {
            sensors: Vec::new(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for SensorManagerConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new("Sensor Manager")
            .id_source(format!("sensor-manager-{}", unique_id))
            .show(ui, |ui| {
                let mut sensor_to_remove = None;
                for (i, sensor) in self.sensors.iter_mut().enumerate() {
                    let sensor_unique_id = format!("{}-{}", unique_id, &sensor.name);
                    ui.horizontal_top(|ui| {
                        sensor.show_mut(
                            ui,
                            ctx,
                            buffer_stack,
                            global_config,
                            current_node_name,
                            &sensor_unique_id,
                        );
                        if ui.button("X").clicked() {
                            sensor_to_remove = Some(i);
                        }
                    });
                }
                if let Some(i) = sensor_to_remove {
                    self.sensors.remove(i);
                }
                if ui.button("Add").clicked() {
                    self.sensors.push(ManagedSensorConfig::default());
                }
            });
    }

    fn show(
        &self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new("Sensor Manager")
            .id_source(format!("sensor-manager-{}", unique_id))
            .show(ui, |ui| {
                for sensor in &self.sensors {
                    let sensor_unique_id = format!("{}-{}", unique_id, &sensor.name);
                    ui.horizontal_top(|ui| {
                        sensor.show(
                            ui,
                            ctx,
                            &sensor_unique_id,
                        );
                    });
                }
            });
    }
}

/// Record listing all the [`SensorRecord`]s and the next observation time.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SensorManagerRecord {
    pub sensors: Vec<ManagedSensorRecord>,
    pub next_time: Option<f32>,
    pub last_observations: Vec<ObservationRecord>,
    pub received_observations: Vec<ObservationRecord>,
}

#[cfg(feature = "gui")]
impl UIComponent for SensorManagerRecord {
    fn show(
            &self,
            ui: &mut egui::Ui,
            ctx: &egui::Context,
            unique_id: &String,
        ) {
        egui::CollapsingHeader::new("Sensors").show(ui, |ui| {
            for s in &self.sensors {
                egui::CollapsingHeader::new(&s.name).show(ui, |ui| {
                    s.record.show(ui, ctx, unique_id);
                });
            }
        });

        egui::CollapsingHeader::new("Last observations").show(ui, |ui| {
            for (i, o) in self.last_observations.iter().enumerate() {
                egui::CollapsingHeader::new(format!("#{}", i)).show(ui, |ui| {
                    o.show(ui, ctx, unique_id);
                });
            }
        });

        egui::CollapsingHeader::new("Received observations").show(ui, |ui| {
            for (i, o) in self.received_observations.iter().enumerate() {
                egui::CollapsingHeader::new(format!("#{}", i)).show(ui, |ui| {
                    o.show(ui, ctx, unique_id);
                });
            }
        });
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ManagedSensorRecord {
    pub name: String,
    pub record: SensorRecord,
}

#[derive(Debug)]
struct ManagedSensor {
    name: String,
    send_to: Vec<String>,
    sensor: Arc<RwLock<Box<dyn Sensor>>>,
}

/// Sensor manager which manages all the node's [`Sensor`]s.
#[derive(Debug)]
pub struct SensorManager {
    sensors: Vec<ManagedSensor>,
    next_time: Option<f32>,
    last_observations: Vec<ObservationRecord>,
    received_observations: Vec<Observation>,
}

impl SensorManager {
    /// Makes a new [`SensorManager`] without any [`Sensor`].
    pub fn new() -> Self {
        Self {
            sensors: Vec::new(),
            next_time: None,
            last_observations: Vec::new(),
            received_observations: Vec::new(),
        }
    }

    /// Makes a new [`SensorManager`] from the given config.
    ///
    /// ## Arguments
    /// * `config` - Config of the [`SensorManager`].
    /// * `plugin_api` - Not used yet, but will be used for external [`Sensor`]s.
    /// * `meta_config` - Simulator meta config.
    pub fn from_config(
        config: &SensorManagerConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        node_name: &String,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        let mut manager = Self::new();
        for sensor_config in &config.sensors {
            manager.sensors.push(ManagedSensor {
                name: sensor_config.name.clone(),
                send_to: sensor_config.send_to.clone(),
                sensor: Arc::new(RwLock::new(match &sensor_config.config {
                    SensorConfig::OrientedLandmarkSensor(c) => {
                        Box::new(OrientedLandmarkSensor::from_config(
                            c,
                            plugin_api,
                            global_config,
                            node_name,
                            va_factory,
                        )) as Box<dyn Sensor>
                    }
                    SensorConfig::OdometrySensor(c) => Box::new(OdometrySensor::from_config(
                        c,
                        plugin_api,
                        global_config,
                        node_name,
                        va_factory,
                    )) as Box<dyn Sensor>,
                    SensorConfig::GNSSSensor(c) => Box::new(GNSSSensor::from_config(
                        c,
                        plugin_api,
                        global_config,
                        node_name,
                        va_factory,
                    )) as Box<dyn Sensor>,
                    SensorConfig::RobotSensor(c) => Box::new(RobotSensor::from_config(
                        c,
                        plugin_api,
                        global_config,
                        node_name,
                        va_factory,
                    )) as Box<dyn Sensor>,
                })),
            });
        }
        manager.next_time = None;
        for sensor in &manager.sensors {
            manager.next_time = Some(
                manager
                    .next_time
                    .unwrap_or(f32::INFINITY)
                    .min(sensor.sensor.read().unwrap().next_time_step()),
            );
        }
        manager
    }

    /// Initialize the [`Sensor`]s. Should be called at the beginning of the run, after
    /// the initialization of the modules.
    pub fn init(&mut self, node: &mut Node) {
        for sensor in &mut self.sensors {
            sensor.sensor.write().unwrap().init(node);
        }
    }

    /// Get the observations at the given `time`.
    pub fn get_observations(&mut self, node: &mut Node, time: f32) -> Vec<Observation> {
        let mut observations = Vec::<Observation>::new();
        let mut min_next_time = None;
        let mut obs_to_send = BTreeMap::new();
        for sensor in &mut self.sensors {
            let sensor_observations: Vec<Observation> = sensor
                .sensor
                .write()
                .unwrap()
                .get_observations(node, time)
                .into_iter()
                .map(|obs| Observation {
                    sensor_name: sensor.name.clone(),
                    observer: node.name(),
                    time,
                    sensor_observation: obs,
                })
                .collect();
            if sensor_observations.len() > 0 {
                for to in &sensor.send_to {
                    if !obs_to_send.contains_key(to) {
                        obs_to_send.insert(to, Vec::new());
                    }
                    obs_to_send
                        .get_mut(to)
                        .unwrap()
                        .extend(sensor_observations.clone());
                }
            }
            observations.extend(sensor_observations);
            min_next_time = Some(
                min_next_time
                    .unwrap_or(f32::INFINITY)
                    .min(sensor.sensor.read().unwrap().next_time_step()),
            );
        }
        if obs_to_send.len() > 0 {
            for (to, observations) in obs_to_send {
                if observations.len() > 0 {
                    let obs_serialized = serde_json::to_value(observations).unwrap();
                    node.network()
                        .expect(
                            "This Node has no network, it cannot send observation to other nodes",
                        )
                        .write()
                        .unwrap()
                        .send_to(to.clone(), obs_serialized, time, Vec::new())
                        .unwrap();
                }
            }
        }
        observations.extend(self.received_observations.drain(0..));
        self.next_time = min_next_time;
        self.last_observations = observations.iter().map(|obs| obs.record()).collect();
        observations
    }

    /// Get the time of the next observation.
    pub fn next_time_step(&self) -> Option<f32> {
        self.next_time
    }
}

impl Stateful<SensorManagerRecord> for SensorManager {
    fn record(&self) -> SensorManagerRecord {
        let mut record = SensorManagerRecord {
            next_time: self.next_time,
            sensors: Vec::new(),
            received_observations: self
                .received_observations
                .iter()
                .map(|o| o.record())
                .collect(),
            last_observations: self.last_observations.clone(),
        };
        for sensor in &self.sensors {
            record.sensors.push(ManagedSensorRecord {
                name: sensor.name.clone(),
                record: sensor.sensor.read().unwrap().record(),
            });
        }
        record
    }

    fn from_record(&mut self, record: SensorManagerRecord) {
        self.next_time = record.next_time;
        self.received_observations = record
            .received_observations
            .into_iter()
            .map(|o| {
                let mut obs = Observation::new();
                obs.from_record(o);
                obs
            })
            .collect();
        for (i, sensor) in self.sensors.iter_mut().enumerate() {
            sensor.name = record.sensors[i].name.clone();
            sensor
                .sensor
                .write()
                .unwrap()
                .from_record(record.sensors[i].record.clone())
        }
    }
}

impl MessageHandler for SensorManager {
    fn handle_message(
        &mut self,
        _robot: &mut Node,
        from: &String,
        message: &serde_json::Value,
        time: f32,
    ) -> Result<(), ()> {
        if let Ok(obs_list) = serde_json::from_value::<Vec<Observation>>(message.clone()) {
            self.received_observations.extend(obs_list);
            // Assure that the observations are always in the same order, for determinism:
            self.received_observations
                .sort_by(|a, b| a.observer.cmp(&b.observer));
            if self.received_observations.len() > 0 {
                self.next_time = Some(time);
            }
            if is_enabled(crate::logger::InternalLog::SensorManager) {
                debug!("Receive observations from {from} at time {time}");
            }
            Ok(())
        } else {
            Err(())
        }
    }
}
