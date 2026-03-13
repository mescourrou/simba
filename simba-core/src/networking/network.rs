//! Network transport primitives used by nodes to exchange messages.
//!
//! This module defines [`Network`], which wraps broker-based publish/subscribe communication,
//! plus [`NetworkConfig`] for node-level communication settings.
//!
//! A [`Network`] can:
//! - create channels,
//! - subscribe multi-clients to channels,
//! - send targeted or node-local messages.
//!
//! [`NetworkConfig`] defaults are:
//! - `range = 0.0`: no distance filtering;
//! - `reception_delay = 0.0`: no additional reception delay.

extern crate confy;
use core::f32;
use std::fmt;
use std::str::FromStr;

use config_checker::*;
use log::debug;
use pyo3::pyclass;
use serde_derive::{Deserialize, Serialize};
use serde_json::Value;
use simba_com::pub_sub::{BrokerTrait, BrokerTraitExtended, PathKey};
use simba_macros::config_derives;

use crate::logger::is_enabled;
use crate::networking::channels;
use crate::simulator::{SimbaBroker, SimbaBrokerMultiClient, SimulatorConfig};
use crate::utils::SharedRwLock;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
#[cfg(feature = "gui")]
use crate::{constants::TIME_ROUND, gui::UIComponent};

/// Configuration for the [`Network`].
#[config_derives]
pub struct NetworkConfig {
    /// Maximum communication range in meters.
    ///
    /// Use `0.0` to disable range filtering (default: `0.0`).
    pub range: f32,
    /// Fixed additional delay applied when receiving messages.
    ///
    /// Use `0.0` for no additional delay (default: `0.0`).
    pub reception_delay: f32,
}

impl Check for NetworkConfig {
    fn do_check(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.range < 0. {
            errors.push(format!("Range should be positive, got {}", self.range));
        }
        if self.reception_delay < 0. {
            errors.push(format!(
                "Reception_delay should be positive, got {}",
                self.reception_delay
            ));
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            range: 0.,
            reception_delay: 0.,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for NetworkConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        _unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Network").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Range (0 for no limit): ");
                if self.range < 0. {
                    self.range = 0.;
                }
                ui.add(egui::DragValue::new(&mut self.range));
            });

            ui.horizontal(|ui| {
                ui.label("Reception delay: ");
                if self.reception_delay < 0. {
                    self.reception_delay = 0.;
                }
                ui.add(
                    egui::DragValue::new(&mut self.reception_delay)
                        .max_decimals((1. / TIME_ROUND) as usize),
                );
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        egui::CollapsingHeader::new("Network").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!("Range (0 for no limit): {}", self.range));
            });

            ui.horizontal(|ui| {
                ui.label(format!("Reception delay: {}", self.reception_delay));
            });
        });
    }
}

/// Transmission mode for messages.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[pyclass(get_all, set_all, eq, eq_int)]
pub enum MessageFlag {
    /// Bypass regular timing and treat the message as instantaneous.
    God,
    /// Ask the receiver to unsubscribe from the associated channel.
    Unsubscribe,
    /// Ask to terminate the receiving node.
    Kill,
}

/// Transport envelope sent through broker channels.
///
/// This structure carries payload, metadata, and control flags used by the networking layer.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Envelope {
    /// Sender node name.
    pub from: String,
    /// Serialized message payload.
    pub message: Value,
    /// Simulation timestamp associated with this message.
    pub timestamp: f32,
    /// Optional transport flags that alter handling behavior.
    pub message_flags: Vec<MessageFlag>,
}

/// Network interface for [`Node`](crate::node::Node).
///
/// Each [`Node`](crate::node::Node) should have a [`Network`] instance. Through this interface,
/// the nodes can send messages to other nodes using pair-to-pair communication,
/// or broadcast diffusion.
pub struct Network {
    /// Name of the robot (will be in the 'from' field of the sent messages).
    from: String,
    /// Range limitation.
    range: f32,
    /// Added delay to the messages at reception.
    reception_delay: f32,
    /// Shared broker reference for channel management and message routing.
    broker: SharedRwLock<SimbaBroker>,
}

impl fmt::Debug for Network {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("Network")
            .field(&self.from)
            .field(&self.range)
            .field(&self.reception_delay)
            .finish()
    }
}

impl Network {
    /// Create a new default Network.
    pub fn new(from: String, broker: &SharedRwLock<SimbaBroker>) -> Network {
        Network::from_config(
            from,
            &NetworkConfig::default(),
            &SimulatorConfig::default(),
            &DeterministRandomVariableFactory::default(),
            broker,
            0.0,
        )
    }

    /// Makes a new Network from the given config.
    pub fn from_config(
        from: String,
        config: &NetworkConfig,
        _global_config: &SimulatorConfig,
        _va_factory: &DeterministRandomVariableFactory,
        broker: &SharedRwLock<SimbaBroker>,
        _initial_time: f32,
    ) -> Network {
        Network {
            from,
            range: config.range,
            reception_delay: config.reception_delay,
            broker: broker.clone(),
        }
    }

    /// Creates an internal channel and returns its absolute key.
    ///
    /// Relative paths are namespaced under the current node internal prefix
    /// [`channels::internal::NODE`].
    pub fn make_channel_internal(&self, key: PathKey) -> PathKey {
        let key = if key.absolute() {
            key
        } else {
            key.prepend_str(&self.from)
                .prepend_str(channels::internal::NODE)
        };
        if is_enabled(crate::logger::InternalLog::NetworkMessages) {
            debug!("Creating channel '{}'", key);
        }
        self.broker.write().unwrap().add_channel(key.clone());
        key
    }

    /// Creates a channel and returns its absolute key.
    ///
    /// Relative paths are namespaced under the current node internal prefix
    /// [`channels::internal::NODE`]. When `self.range > 0.0`, message delivery is filtered by
    /// Euclidean distance.
    pub fn make_channel(&self, key: PathKey) -> PathKey {
        let key = if key.absolute() {
            key
        } else {
            key.prepend_str(&self.from)
                .prepend_str(channels::internal::NODE)
        };
        let range = self.range;
        if is_enabled(crate::logger::InternalLog::NetworkMessages) {
            debug!("Creating channel '{}' with range {}", key, range);
        }
        self.broker
            .write()
            .unwrap()
            .add_channel_conditionnal(key.clone(), move |x1, x2| {
                if range == 0. {
                    true
                } else if let Some(x1) = x1
                    && let Some(x2) = x2
                {
                    ((x1[0] - x2[0]).powi(2) + (x1[1] - x2[1]).powi(2)).sqrt() <= range
                } else {
                    true
                }
            });
        key
    }

    /// Subscribes a multi-client to the provided channels using the configured reception delay.
    ///
    /// If `multi_client` is `None`, a new [`SimbaBrokerMultiClient`] is created and returned.
    pub fn subscribe_to(
        &self,
        keys: &[PathKey],
        multi_client: Option<SimbaBrokerMultiClient>,
    ) -> SimbaBrokerMultiClient {
        let mut multi_client = multi_client.unwrap_or_else(|| {
            SimbaBrokerMultiClient::new(
                self.broker.clone(),
                self.from.clone(),
                self.reception_delay,
                PathKey::from_str(channels::internal::NODE)
                    .unwrap()
                    .join_str(&self.from),
            )
        });
        if is_enabled(crate::logger::InternalLog::NetworkMessages) {
            debug!("Subscribe to '{:?}'", keys);
            debug!(
                "Channels: {}",
                self.broker
                    .read()
                    .unwrap()
                    .channel_list()
                    .iter()
                    .map(|k| k.to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }
        self.broker
            .write()
            .unwrap()
            .subscribe_to_list(keys, self.reception_delay, &mut multi_client)
            .unwrap();
        multi_client
    }

    /// Subscribes a multi-client to channels with zero reception delay.
    ///
    /// This is useful for control paths that must be handled immediately. If `multi_client` is
    /// `None`, a new [`SimbaBrokerMultiClient`] is created first.
    pub fn subscribe_to_instantaneous(
        &self,
        keys: &[PathKey],
        multi_client: Option<SimbaBrokerMultiClient>,
    ) -> SimbaBrokerMultiClient {
        // Create a multiclient with the usual reception delay to be able to add other channels with the usual reception delay later if needed
        let mut multi_client = multi_client.unwrap_or_else(|| {
            SimbaBrokerMultiClient::new(
                self.broker.clone(),
                self.from.clone(),
                self.reception_delay,
                PathKey::from_str(channels::internal::NODE)
                    .unwrap()
                    .join_str(&self.from),
            )
        });
        // but subscribe to the channel with 0 reception delay.
        if is_enabled(crate::logger::InternalLog::NetworkMessages) {
            debug!("Subscribe instantaneous to '{:?}'", keys);
        }
        self.broker
            .write()
            .unwrap()
            .subscribe_to_list(
                keys,
                0.,
                &mut multi_client
                    as &mut dyn simba_com::pub_sub::MultiClientTrait<PathKey, Envelope, String>,
            )
            .unwrap();
        multi_client
    }

    /// Sends `message` to a specific recipient node on `channel` at simulation `time`.
    ///
    /// If `channel` is relative, it is prefixed with the recipient node internal namespace.
    pub fn send_to_node(&self, recipient: String, channel: PathKey, message: Envelope, time: f32) {
        let key = if channel.absolute() {
            channel
        } else {
            channel
                .prepend_str(&recipient)
                .prepend_str(channels::internal::NODE)
        };
        if let Some(tmp_client) =
            self.broker
                .write()
                .unwrap()
                .subscribe_to(&key, self.from.clone(), self.reception_delay)
        {
            if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                debug!("Sending message to '{}': {:?}", key, message);
            }
            tmp_client.send(message, time);
        }
    }

    /// Sends `message` to this node-scoped `channel` at simulation `time`.
    ///
    /// If `channel` is relative, it is prefixed with this node internal namespace.
    pub fn send_to(&self, channel: PathKey, message: Envelope, time: f32) {
        let key = if channel.absolute() {
            channel
        } else {
            channel
                .prepend_str(&self.from)
                .prepend_str(channels::internal::NODE)
        };
        if let Some(tmp_client) =
            self.broker
                .write()
                .unwrap()
                .subscribe_to(&key, self.from.clone(), self.reception_delay)
        {
            if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                debug!("Sending message to '{}': {:?}", key, message);
            }
            tmp_client.send(message, time);
        }
    }
}
