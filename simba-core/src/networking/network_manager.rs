/*!
Provide the Manager of the nodes [`Network`](crate::networking::network::Network)s. Only one should exist for one
[`Simulator`](crate::simulator::Simulator).
*/

use log::debug;
use serde_json::Value;
use simba_com::pub_sub::{BrokerTraitProcessing, PathBroker};

use crate::constants::TIME_ROUND;
use crate::errors::SimbaResult;
use crate::logger::is_enabled;
use crate::simulator::SimbaBroker;
use crate::utils::SharedRwLock;

use super::network::MessageFlag;
use std::collections::HashMap;

use std::sync::{Arc, RwLock};

#[derive(Debug, Clone)]
pub enum MessageSendMethod {
    Broadcast,
    Recipient(String),
    /// Special messages to the simulator network manager
    Manager,
}

#[derive(Debug, Clone)]
pub struct NetworkMessage {
    pub value: Value,
    pub from: String,
    pub to: MessageSendMethod,
    pub time: f32,
    pub range: f32,
    pub message_flags: Vec<MessageFlag>,
}

/// Manages the [`Network`](crate::networking::network::Network)s, making the link between them, and keep a list.
#[derive(Debug)]
pub struct NetworkManager {
    broker: SharedRwLock<SimbaBroker>,
}

impl NetworkManager {
    pub fn new() -> Self {
        Self {
            broker: Arc::new(RwLock::new(PathBroker::new(TIME_ROUND))),
        }
    }

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

    pub fn broker(&self) -> SharedRwLock<SimbaBroker> {
        self.broker.clone()
    }
}

impl Default for NetworkManager {
    fn default() -> Self {
        Self::new()
    }
}
