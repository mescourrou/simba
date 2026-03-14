//! Central manager for inter-node network message routing.
//!
//! A single [`NetworkManager`] instance is owned by each
//! [`Simulator`](crate::simulator::Simulator). It holds the shared message broker used by all
//! node [`Network`](crate::networking::network::Network) instances and advances queued messages in
//! simulation time order.

use log::debug;
use serde_json::Value;
use simba_com::pub_sub::{BrokerTrait, BrokerTraitProcessing, PathBroker};

use crate::constants::TIME_ROUND;
use crate::errors::SimbaResult;
use crate::logger::is_enabled;
use crate::simulator::SimbaBroker;
use crate::utils::SharedRwLock;

use super::network::MessageFlag;
use std::collections::HashMap;

use std::sync::{Arc, RwLock};

/// Destination strategy used when sending a network message.
#[derive(Debug, Clone)]
pub enum MessageSendMethod {
    /// Sends the message to every compatible subscriber.
    Broadcast,
    /// Sends the message to a specific recipient node.
    Recipient(String),
    /// Sends a control message to the network manager itself.
    Manager,
}

/// Internal transport payload forwarded by the network manager broker.
#[derive(Debug, Clone)]
pub struct NetworkMessage {
    /// Serialized message payload.
    pub value: Value,
    /// Sender node name.
    pub from: String,
    /// Routing strategy for the destination.
    pub to: MessageSendMethod,
    /// Simulation timestamp associated with this message.
    pub time: f32,
    /// Communication range used for spatial filtering.
    pub range: f32,
    /// Transport-level flags that alter delivery behavior.
    pub message_flags: Vec<MessageFlag>,
}

/// Manages all [`Network`](crate::networking::network::Network) instances through a shared broker.
#[derive(Debug)]
pub struct NetworkManager {
    broker: SharedRwLock<SimbaBroker>,
}

impl NetworkManager {
    /// Creates a new [`NetworkManager`] with an empty broker.
    pub fn new() -> Self {
        Self {
            broker: Arc::new(RwLock::new(PathBroker::new(TIME_ROUND))),
        }
    }

    /// Clears all broker channels and pending routing state.
    pub fn reset(&mut self) {
        self.broker.write().unwrap().clear_channels();
    }

    /// Processes queued messages using the optional position map for range filtering.
    ///
    /// The `position_map` maps node names to their current 2D position.
    /// Nodes with `None` position are not range-filtered and receive all messages.
    pub fn process_messages(
        &mut self,
        position_map: &HashMap<String, Option<[f32; 2]>>,
    ) -> SimbaResult<()> {
        if is_enabled(crate::logger::InternalLog::NetworkMessages) {
            debug!("Processing messages");
        }
        self.broker
            .write()
            .unwrap()
            .process_messages(Some(position_map));
        Ok(())
    }

    /// Returns a shared handle to the underlying broker.
    pub fn broker(&self) -> SharedRwLock<SimbaBroker> {
        self.broker.clone()
    }
}

impl Default for NetworkManager {
    fn default() -> Self {
        Self::new()
    }
}
