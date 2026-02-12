/*!
Provide the [`SensorManager`], which owns the different [`Sensor`]s and get the
available observations.
*/

extern crate confy;
use core::f32;
use log::{debug, warn};
use pyo3::prelude::*;
use serde_derive::{Deserialize, Serialize};
use simba_com::pub_sub::{MultiClientTrait, PathKey};
use simba_macros::config_derives;
use std::collections::BTreeMap;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex, RwLock};

use crate::constants::TIME_ROUND;
use crate::errors::SimbaResult;
#[cfg(feature = "gui")]
use crate::gui::{
    UIComponent,
    utils::{string_checkbox, text_singleline_with_apply},
};
use crate::logger::{InternalLog, is_enabled};
use crate::networking;
use crate::networking::network::{self, Envelope, Network};
use crate::node::Node;
use crate::sensors::displacement_sensor::DisplacementSensor;
use crate::sensors::external_sensor::ExternalSensor;
use crate::simulator::SimbaBrokerMultiClient;
use crate::state_estimators::State;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::{SharedMutex, SharedRwLock};
use crate::{recordable::Recordable, simulator::SimulatorConfig};

use super::gnss_sensor::GNSSSensor;
use super::oriented_landmark_sensor::OrientedLandmarkSensor;
use super::robot_sensor::RobotSensor;
use super::speed_sensor::{SpeedSensor, SpeedSensorConfig};
use super::{Observation, ObservationRecord, Sensor, SensorConfig, SensorRecord};
use crate::plugin_api::PluginAPI;

#[config_derives]
pub struct ManagedSensorConfig {
    pub name: String,
    pub send_to: Vec<String>,
    pub triggered: bool,
    #[check]
    pub config: SensorConfig,
}

impl Default for ManagedSensorConfig {
    fn default() -> Self {
        Self {
            name: "some_sensor".to_string(),
            send_to: Vec::new(),
            triggered: false,
            config: SensorConfig::SpeedSensor(SpeedSensorConfig::default()),
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
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new(&self.name)
            .id_salt(format!("managed-sensor-{}", unique_id).as_str())
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

                ui.horizontal(|ui| {
                    ui.label("Triggered: ");
                    ui.checkbox(&mut self.triggered, "");
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

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new(&self.name)
            .id_salt(format!("managed-sensor-{}", unique_id).as_str())
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(format!("Name: {}", self.name));
                });

                ui.horizontal(|ui| {
                    ui.label(format!("Triggered: {}", self.triggered));
                });

                ui.horizontal_wrapped(|ui| {
                    ui.label("Send to: ");
                    for to in &self.send_to {
                        ui.label(format!("{}, ", to));
                    }
                });

                self.config.show(ui, ctx, unique_id);
            });
    }
}

/// Configuration listing all the [`SensorConfig`]s.
#[config_derives]
#[derive(Default)]
pub struct SensorManagerConfig {
    #[check]
    pub sensors: Vec<ManagedSensorConfig>,
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
        unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Sensor Manager")
            .id_salt(format!("sensor-manager-{}", unique_id))
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

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new("Sensor Manager")
            .id_salt(format!("sensor-manager-{}", unique_id))
            .show(ui, |ui| {
                for sensor in &self.sensors {
                    let sensor_unique_id = format!("{}-{}", unique_id, &sensor.name);
                    ui.horizontal_top(|ui| {
                        sensor.show(ui, ctx, &sensor_unique_id);
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
}

#[cfg(feature = "gui")]
impl UIComponent for SensorManagerRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
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
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ManagedSensorRecord {
    pub name: String,
    pub last_triggered: Option<f32>,
    pub record: SensorRecord,
}

#[derive(Debug)]
struct ManagedSensor {
    name: String,
    send_to: Vec<String>,
    triggered: bool,
    last_triggered: Option<f32>,
    sensor: SharedRwLock<Box<dyn Sensor>>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[pyclass(get_all, set_all)]
pub struct SensorTriggerMessage {}

#[pymethods]
impl SensorTriggerMessage {
    #[new]
    pub fn new() -> Self {
        Self {}
    }
}

/// Sensor manager which manages all the node's [`Sensor`]s.
#[derive(Debug)]
pub struct SensorManager {
    sensors: Vec<ManagedSensor>,
    next_time: Option<f32>,
    last_observations: Vec<ObservationRecord>,
    local_observations: Vec<Observation>,
    distant_observations: Vec<Observation>,
    message_client: Option<SimbaBrokerMultiClient>,
    channel_root: Option<PathKey>,
}

impl SensorManager {
    pub const CHANNEL_NAME: &'static str = "sensors";
    pub const OBSERVATION_CHANNEL: &'static str = "observations";

    /// Makes a new [`SensorManager`] without any [`Sensor`].
    pub fn new() -> Self {
        Self {
            sensors: Vec::new(),
            next_time: None,
            last_observations: Vec::new(),
            local_observations: Vec::new(),
            distant_observations: Vec::new(),
            message_client: None,
            channel_root: None,
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
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        node_name: &String,
        va_factory: &DeterministRandomVariableFactory,
        network: &SharedRwLock<Network>,
        initial_time: f32,
        initial_state: &State,
    ) -> SimbaResult<Self> {
        let mut manager = Self::new();
        let sensor_manager_key = PathKey::from_str(networking::channels::internal::NODE)
            .join_str(node_name.as_str())
            .join_str(Self::CHANNEL_NAME);
        manager.channel_root = Some(sensor_manager_key.clone());
        network.write().unwrap().make_channel(
            sensor_manager_key
                .clone()
                .join_str(Self::OBSERVATION_CHANNEL),
        );
        for sensor_config in &config.sensors {
            if sensor_config.triggered {
                network
                    .write()
                    .unwrap()
                    .make_channel(sensor_manager_key.clone().join_str(&sensor_config.name));
            }

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
                            initial_time,
                        )) as Box<dyn Sensor>
                    }
                    #[allow(deprecated)]
                    SensorConfig::OdometrySensor(c) => {
                        warn!("OdometrySensor is deprecated and renamed to SpeedSensor");
                        Box::new(SpeedSensor::from_config(
                            c,
                            plugin_api,
                            global_config,
                            node_name,
                            va_factory,
                            initial_time,
                        )) as Box<dyn Sensor>
                    }
                    SensorConfig::SpeedSensor(c) => Box::new(SpeedSensor::from_config(
                        c,
                        plugin_api,
                        global_config,
                        node_name,
                        va_factory,
                        initial_time,
                    )) as Box<dyn Sensor>,
                    SensorConfig::DisplacementSensor(c) => {
                        Box::new(DisplacementSensor::from_config(
                            c,
                            plugin_api,
                            global_config,
                            node_name,
                            va_factory,
                            initial_time,
                            initial_state,
                        )) as Box<dyn Sensor>
                    }
                    SensorConfig::GNSSSensor(c) => Box::new(GNSSSensor::from_config(
                        c,
                        plugin_api,
                        global_config,
                        node_name,
                        va_factory,
                        initial_time,
                    )) as Box<dyn Sensor>,
                    SensorConfig::RobotSensor(c) => Box::new(RobotSensor::from_config(
                        c,
                        plugin_api,
                        global_config,
                        node_name,
                        va_factory,
                        initial_time,
                    )) as Box<dyn Sensor>,
                    SensorConfig::External(c) => Box::new(ExternalSensor::from_config(
                        c,
                        plugin_api,
                        global_config,
                        va_factory,
                        network,
                        initial_time,
                    )?) as Box<dyn Sensor>,
                })),
                triggered: sensor_config.triggered,
                last_triggered: None,
            });
        }

        // Subscribe to all channels of the sensor manager, to receive both observations and trigger messages:
        manager.message_client = Some(
            network
                .write()
                .unwrap()
                .subscribe_to(&[sensor_manager_key], None),
        );
        debug!("Sensor Manager subscribed to channel {:?}", manager.message_client.as_ref().unwrap().subscribed_keys());
        manager.next_time = None;
        for sensor in &manager.sensors {
            manager.next_time = Some(
                manager
                    .next_time
                    .unwrap_or(f32::INFINITY)
                    .min(sensor.sensor.read().unwrap().next_time_step()),
            );
        }
        Ok(manager)
    }

    /// Initialize the [`Sensor`]s. Should be called at the beginning of the run, after
    /// the initialization of the modules.
    pub fn init(&mut self, node: &mut Node, initial_time: f32) {
        for sensor in &mut self.sensors {
            sensor.sensor.write().unwrap().init(node, initial_time);
        }
    }

    pub fn handle_messages(&mut self, time: f32) {
        while let Some((path, envelope)) = self.message_client.as_ref().unwrap().try_receive(time) {
            debug!("Sensor Manager received message on path {:?} at time {}: {:?}", path, envelope.timestamp, envelope.message);
            if path
                == self
                    .channel_root
                    .as_ref()
                    .unwrap()
                    .join_str(Self::OBSERVATION_CHANNEL)
            {
                let obs_list =
                    serde_json::from_value::<Vec<Observation>>(envelope.message.clone()).unwrap();
                self.last_observations
                    .extend(obs_list.iter().map(|o| o.record()));
                self.distant_observations.extend(obs_list);
                // Assure that the observations are always in the same order, for determinism:
                self.distant_observations
                    .sort_by(|a, b| a.observer.cmp(&b.observer));
                // if self.received_observations.len() > 0 {
                //     self.next_time = Some(time);
                // }
                if is_enabled(crate::logger::InternalLog::SensorManager) {
                    debug!(
                        "Receive observations from {} at time {}",
                        envelope.from, envelope.timestamp
                    );
                }
            } else if serde_json::from_value::<SensorTriggerMessage>(envelope.message.clone())
                .is_ok()
            {
                let sensor_name = path.to_vec().last().unwrap().clone();
                for sensor in &mut self.sensors {
                    if sensor.name == sensor_name {
                        sensor.last_triggered = Some(time);
                        if is_enabled(crate::logger::InternalLog::SensorManager) {
                            debug!("Sensor {} triggered at time {}", sensor.name, time);
                        }
                    }
                }
            } else {
                warn!(
                    "[Sensor Manager] Received message on unknown type or path {:?}: {:?}",
                    path, envelope.message
                );
            }
        }
    }

    /// Get the observations at the given `time`.
    pub fn get_observations(&mut self) -> Vec<Observation> {
        let mut observations = Vec::new();
        observations.extend(self.distant_observations.drain(0..));
        observations.extend(self.local_observations.drain(0..));
        let mut min_next_time = None;
        for sensor in &mut self.sensors {
            min_next_time = Some(
                min_next_time
                    .unwrap_or(f32::INFINITY)
                    .min(sensor.sensor.read().unwrap().next_time_step()),
            );
        }
        self.next_time = min_next_time;
        observations
    }

    pub fn make_observations(&mut self, node: &mut Node, time: f32) {
        self.local_observations.clear();
        self.last_observations.clear();
        let mut min_next_time = None;
        let mut obs_to_send = BTreeMap::new();
        for sensor in &mut self.sensors {
            if is_enabled(InternalLog::SensorManager) {
                log::debug!(
                    "Sensor {} last triggered at {:?} ({})",
                    sensor.name,
                    sensor.last_triggered,
                    sensor.triggered
                );
            }
            let sensor_observations: Vec<Observation> = if (sensor.triggered
                && match sensor.last_triggered {
                    Some(t) => (time - t).abs() < TIME_ROUND,
                    None => false,
                })
                || (sensor.sensor.read().unwrap().next_time_step() - time).abs() < TIME_ROUND
            {
                if is_enabled(InternalLog::SensorManager) {
                    log::debug!("Sensor {} is triggered, getting observations", sensor.name);
                }
                sensor
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
                    .collect()
            } else {
                Vec::new()
            };

            if !sensor_observations.is_empty() {
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
            self.local_observations.extend(sensor_observations);
            min_next_time = Some(
                min_next_time
                    .unwrap_or(f32::INFINITY)
                    .min(sensor.sensor.read().unwrap().next_time_step()),
            );
        }
        if !obs_to_send.is_empty() {
            let key_base = PathKey::from_str(networking::channels::internal::NODE);
            for (to, observations) in obs_to_send {
                if !observations.is_empty() {
                    let obs_serialized = serde_json::to_value(observations).unwrap();
                    node.network()
                        .expect(
                            "This Node has no network, it cannot send observation to other nodes",
                        )
                        .write()
                        .unwrap()
                        .send_to(
                            key_base.join_str(&to).join_str(Self::OBSERVATION_CHANNEL),
                            Envelope {
                                from: node.name(),
                                message: obs_serialized,
                                timestamp: time,
                                message_flags: Vec::new(),
                            },
                            time,
                        );
                }
            }
        }
        self.last_observations
            .extend(self.local_observations.iter().map(|o| o.record()));
        self.next_time = min_next_time;
    }

    /// Get the time of the next observation.
    pub fn next_time_step(&self) -> Option<f32> {
        self.next_time
    }
}

impl Default for SensorManager {
    fn default() -> Self {
        Self::new()
    }
}

impl Recordable<SensorManagerRecord> for SensorManager {
    fn record(&self) -> SensorManagerRecord {
        let mut record = SensorManagerRecord {
            next_time: self.next_time,
            sensors: Vec::new(),
            last_observations: self.last_observations.clone(),
        };
        record.last_observations.sort();
        for sensor in &self.sensors {
            record.sensors.push(ManagedSensorRecord {
                name: sensor.name.clone(),
                record: sensor.sensor.read().unwrap().record(),
                last_triggered: sensor.last_triggered,
            });
        }
        record
    }
}
