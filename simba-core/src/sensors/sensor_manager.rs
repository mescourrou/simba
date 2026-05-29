//! Sensor manager implementation.
//!
//! This module defines [`SensorManager`], the component responsible for owning node sensors,
//! triggering observation production, handling inter-node observation messages, and exposing
//! consolidated observations to the rest of the simulation pipeline.
//! Sensor creation is driven by [`SensorManagerConfig`], which contains [`ManagedSensorConfig`]
//! entries describing each managed sensor and its routing behavior.
//!
//!
//! All sensors generate different types of observations but every observation has the maximum of available variables.
//! If the user wants to use only a subset of the variables, they can ignore the others.
//!

extern crate confy;
use core::f32;
use pyo3::prelude::*;
use serde_derive::{Deserialize, Serialize};
use simba_com::pub_sub::{MultiClientTrait, PathKey};
use simba_macros::config_derives;
use std::collections::BTreeMap;
use std::str::FromStr;
use std::sync::{Arc, RwLock};

use crate::constants::TIME_ROUND;
use crate::context::Context;
use crate::errors::SimbaResult;
#[cfg(feature = "gui")]
use crate::gui::{
    UIComponent,
    utils::{string_checkbox, text_singleline_with_apply},
};
use crate::logger::InternalLog;
use crate::networking::message_types::MessageTypes;
use crate::networking::network::Envelope;
use crate::node::Node;
use crate::node::node_factory::FromConfigArguments;
use crate::sensors::displacement_sensor::DisplacementSensor;
use crate::sensors::external_sensor::ExternalSensor;
use crate::sensors::scan_sensor::ScanSensor;
use crate::simulator::SimbaBrokerMultiClient;
use crate::state_estimators::State;
use crate::utils::SharedRwLock;
use crate::{internal, networking, warning};
use crate::{recordable::Recordable, simulator::SimulatorConfig};

use super::gnss_sensor::GNSSSensor;
use super::oriented_landmark_sensor::OrientedLandmarkSensor;
use super::robot_sensor::RobotSensor;
use super::speed_sensor::{SpeedSensor, SpeedSensorConfig};
use super::{Observation, ObservationRecord, Sensor, SensorConfig, SensorRecord};

/// Configuration of one managed sensor entry.
///
/// It defines the sensor instance, optional destination nodes for forwarded observations,
/// and whether the sensor is event-triggered. Observation that are sent to other nodes are also
/// available locally in the node's sensor manager, so they can be used by the node's state estimator.
///
/// Default values:
/// - `name`: `"some_sensor"`
/// - `send_to`: empty vector
/// - `triggered`: `false`, setting it to `true` ignore the activation times of the sensor and wait for [`SensorTriggerMessage`] to produce observations.
/// - `config`: [`SensorConfig::Speed`] with [`SpeedSensorConfig::default`]
#[config_derives]
pub struct ManagedSensorConfig {
    /// Name used to identify this sensor in records and trigger channels.
    pub name: String,
    /// Destination node names receiving forwarded observations from this sensor.
    pub send_to: Vec<String>,
    /// Whether this sensor produces observations only when explicitly triggered.
    pub triggered: bool,
    #[check]
    /// Concrete sensor configuration.
    pub config: SensorConfig,
}

impl Default for ManagedSensorConfig {
    fn default() -> Self {
        Self {
            name: "some_sensor".to_string(),
            send_to: Vec::new(),
            triggered: false,
            config: SensorConfig::Speed(SpeedSensorConfig::default()),
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
///
/// Default values:
/// - `sensors`: empty vector
#[config_derives]
#[derive(Default)]
pub struct SensorManagerConfig {
    #[check]
    /// List of managed sensor configurations.
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

/// Record listing all the [`SensorRecord`]s, the next observation time and the last emitted observations of the node's sensors.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SensorManagerRecord {
    /// Per-sensor records currently managed by the node.
    pub sensors: Vec<ManagedSensorRecord>,
    /// Next time at which at least one sensor is expected to produce observations.
    pub next_time: Option<f32>,
    /// Last emitted observations in record form.
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

/// Record for one managed sensor.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ManagedSensorRecord {
    /// Sensor name.
    pub name: String,
    /// Last trigger time if this sensor was externally triggered.
    pub last_triggered: Option<f32>,
    /// Sensor-specific record payload.
    pub record: SensorRecord,
}

/// Runtime manager of one sensor allowing its triggering, observation retrieval, and observation forwarding.
#[derive(Debug)]
struct ManagedSensor {
    name: String,
    send_to: Vec<String>,
    triggered: bool,
    last_triggered: Option<f32>,
    sensor: SharedRwLock<Box<dyn Sensor>>,
}

/// Message used to trigger a sensor through the internal network.
///
/// The message is empty for now, but it could be extended in the future to include additional information about the trigger (e.g. time to trigger, dynamic sensor parameters, etc.).
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
#[pyclass(get_all, set_all)]
pub struct SensorTriggerMessage {}

#[pymethods]
impl SensorTriggerMessage {
    /// Creates an empty trigger message.
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
    /// Channel segment used as the root path for sensor-manager messages.
    pub const CHANNEL_NAME: &'static str = "sensors";
    /// Channel segment used for observation payload messages.
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
    /// * `from_config_args` - Additional arguments required for sensor instantiation from config, including plugin API reference, global config, random variable factory, and initial time.
    /// * `initial_state` - Initial state of the node, required for some sensors (e.g. [`DisplacementSensor`]) to compute their first observations.
    pub fn from_config(
        config: &SensorManagerConfig,
        from_config_args: &FromConfigArguments,
        initial_state: &State,
        context: &Context,
    ) -> SimbaResult<Self> {
        let mut manager = Self::new();
        let sensor_manager_key = PathKey::from_str(networking::channels::internal::NODE)
            .unwrap()
            .join_str(from_config_args.node_name.as_str())
            .join_str(Self::CHANNEL_NAME);
        manager.channel_root = Some(sensor_manager_key.clone());
        from_config_args.network.write().unwrap().make_channel(
            sensor_manager_key
                .clone()
                .join_str(Self::OBSERVATION_CHANNEL),
            context,
        );
        for sensor_config in &config.sensors {
            if sensor_config.triggered {
                from_config_args.network.write().unwrap().make_channel(
                    sensor_manager_key.clone().join_str(&sensor_config.name),
                    context,
                );
            }

            manager.sensors.push(ManagedSensor {
                name: sensor_config.name.clone(),
                send_to: sensor_config.send_to.clone(),
                sensor: Arc::new(RwLock::new(match &sensor_config.config {
                    SensorConfig::OrientedLandmark(c) => {
                        Box::new(OrientedLandmarkSensor::from_config(
                            c,
                            from_config_args.plugin_api,
                            from_config_args.global_config,
                            from_config_args.va_factory,
                            from_config_args.initial_time,
                            context,
                        )?) as Box<dyn Sensor>
                    }
                    SensorConfig::Speed(c) => Box::new(SpeedSensor::from_config(
                        c,
                        from_config_args.plugin_api,
                        from_config_args.global_config,
                        from_config_args.va_factory,
                        from_config_args.initial_time,
                        context,
                    )?) as Box<dyn Sensor>,
                    SensorConfig::Displacement(c) => Box::new(DisplacementSensor::from_config(
                        c,
                        from_config_args.plugin_api,
                        from_config_args.global_config,
                        from_config_args.va_factory,
                        from_config_args.initial_time,
                        initial_state,
                        context,
                    )?) as Box<dyn Sensor>,
                    SensorConfig::GNSS(c) => Box::new(GNSSSensor::from_config(
                        c,
                        from_config_args.plugin_api,
                        from_config_args.global_config,
                        from_config_args.va_factory,
                        from_config_args.initial_time,
                        context,
                    )?) as Box<dyn Sensor>,
                    SensorConfig::Robot(c) => Box::new(RobotSensor::from_config(
                        c,
                        from_config_args.plugin_api,
                        from_config_args.global_config,
                        from_config_args.va_factory,
                        from_config_args.initial_time,
                        context,
                    )?) as Box<dyn Sensor>,
                    SensorConfig::Scan(c) => Box::new(ScanSensor::from_config(
                        c,
                        from_config_args.plugin_api,
                        from_config_args.global_config,
                        from_config_args.va_factory,
                        from_config_args.initial_time,
                        context,
                    )?) as Box<dyn Sensor>,
                    SensorConfig::External(c) => Box::new(ExternalSensor::from_config(
                        c,
                        from_config_args.plugin_api,
                        from_config_args.global_config,
                        from_config_args.va_factory,
                        from_config_args.network,
                        from_config_args.initial_time,
                        context,
                    )?) as Box<dyn Sensor>,
                })),
                triggered: sensor_config.triggered,
                last_triggered: None,
            });
        }

        // Subscribe to all channels of the sensor manager, to receive both observations and trigger messages:
        manager.message_client = Some(from_config_args.network.write().unwrap().subscribe_to(
            &[sensor_manager_key],
            None,
            context,
        ));
        internal!(
            context,
            crate::logger::InternalLog::SensorManager,
            "Sensor Manager subscribed to channel {:?}",
            manager.message_client.as_ref().unwrap().subscribed_keys()
        );
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
    ///
    /// ## Arguments
    /// * `node` - Node owning the sensors.
    /// * `initial_time` - Initial simulation time used by sensor initialization hooks.
    /// * `context` - Shared simulation context used for logging.
    pub fn post_init(
        &mut self,
        node: &mut Node,
        initial_time: f32,
        context: &Context,
    ) -> SimbaResult<()> {
        for sensor in &mut self.sensors {
            sensor
                .sensor
                .write()
                .unwrap()
                .post_init(node, initial_time, context)?;
        }
        Ok(())
    }

    /// Handles incoming sensor-manager messages at the given simulation time.
    ///
    /// This consumes queued messages from the internal subscriber, updates remote
    /// observations, and applies trigger messages to targeted sensors.
    ///
    /// This is where distant observations are collected.
    ///
    /// ## Arguments
    /// * `time` - Current simulation time upper-bound for message retrieval.
    /// * `context` - Shared simulation context used for sensor-manager logs.
    pub fn handle_messages(&mut self, time: f32, context: &Context) {
        while let Some((path, envelope)) = self.message_client.as_ref().unwrap().try_receive(time) {
            internal!(
                context,
                crate::logger::InternalLog::SensorManager,
                "Sensor Manager received message on path {:?} at time {}",
                path,
                envelope.timestamp
            );
            if path
                == self
                    .channel_root
                    .as_ref()
                    .unwrap()
                    .join_str(Self::OBSERVATION_CHANNEL)
            {
                let obs_list = match envelope.message {
                    MessageTypes::Observations(obs) => obs,
                    _ => panic!("Received message on observation channel with invalid type"),
                };
                self.last_observations
                    .extend(obs_list.iter().map(|o| o.record(context)));
                self.distant_observations.extend(obs_list);
                // Assure that the observations are always in the same order, for determinism:
                self.distant_observations
                    .sort_by(|a, b| a.observer.cmp(&b.observer));
                // if self.received_observations.len() > 0 {
                //     self.next_time = Some(time);
                // }
                internal!(
                    context,
                    crate::logger::InternalLog::SensorManager,
                    "Receive observations from {} at time {}",
                    envelope.from,
                    envelope.timestamp
                );
            } else if let MessageTypes::SensorTrigger(_) = &envelope.message {
                let sensor_name = path.to_vec().last().unwrap().clone();
                for sensor in &mut self.sensors {
                    if sensor.name == sensor_name {
                        sensor.last_triggered = Some(time);
                        internal!(
                            context,
                            crate::logger::InternalLog::SensorManager,
                            "Sensor {} triggered at time {}",
                            sensor.name,
                            time
                        );
                    }
                }
            } else {
                warning!(
                    context,
                    "[Sensor Manager] Received message on unknown type or path {:?}: {:?}",
                    path,
                    envelope.message
                );
            }
        }
    }

    /// Consume the last observations. This includes both local observations produced by the node's sensors
    /// and distant observations received from other nodes.
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

    /// Produces local observations for sensors that are due or externally triggered.
    ///
    /// Generated observations are stored locally and forwarded to destination nodes
    /// according to each sensor's `send_to` configuration.
    ///
    /// ## Arguments
    /// * `node` - Node owning the sensors and network handle.
    /// * `time` - Current simulation time used to decide which sensors should emit.
    /// * `context` - Shared simulation context used for sensor-manager logs.
    pub fn make_observations(&mut self, node: &mut Node, time: f32, context: &Context) {
        self.local_observations.clear();
        self.last_observations.clear();
        let mut min_next_time = None;
        let mut obs_to_send = BTreeMap::new();
        for sensor in &mut self.sensors {
            internal!(
                context,
                InternalLog::SensorManager,
                "Sensor {} last triggered at {:?} ({})",
                sensor.name,
                sensor.last_triggered,
                sensor.triggered
            );
            let sensor_observations: Vec<Observation> = if (sensor.triggered
                && match sensor.last_triggered {
                    Some(t) => (time - t).abs() < TIME_ROUND,
                    None => false,
                })
                || (sensor.sensor.read().unwrap().next_time_step() - time).abs() < TIME_ROUND
            {
                internal!(
                    context,
                    InternalLog::SensorManager,
                    "Sensor {} is triggered, getting observations",
                    sensor.name
                );
                sensor
                    .sensor
                    .write()
                    .unwrap()
                    .get_observations(node, time, context)
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
            let key_base = PathKey::from_str(networking::channels::internal::NODE).unwrap();
            for (to, observations) in obs_to_send {
                if !observations.is_empty() {
                    let obs_msg = observations.into();
                    node.network()
                        .expect(
                            "This Node has no network, it cannot send observation to other nodes",
                        )
                        .write()
                        .unwrap()
                        .send_to(
                            key_base
                                .join_str(to)
                                .join_str(Self::CHANNEL_NAME)
                                .join_str(Self::OBSERVATION_CHANNEL),
                            Envelope {
                                from: node.name(),
                                message: obs_msg,
                                timestamp: time,
                                message_flags: Vec::new(),
                            },
                            time,
                            context,
                        );
                }
            }
        }
        self.last_observations
            .extend(self.local_observations.iter().map(|o| o.record(context)));
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
    fn record(&self, context: &Context) -> SensorManagerRecord {
        let mut record = SensorManagerRecord {
            next_time: self.next_time,
            sensors: Vec::new(),
            last_observations: self.last_observations.clone(),
        };
        record.last_observations.sort();
        for sensor in &self.sensors {
            record.sensors.push(ManagedSensorRecord {
                name: sensor.name.clone(),
                record: sensor.sensor.read().unwrap().record(context),
                last_triggered: sensor.last_triggered,
            });
        }
        record
    }
}
