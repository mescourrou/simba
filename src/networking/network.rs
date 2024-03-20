extern crate confy;
use std::sync::{Arc, RwLock};
use std::fmt;

use serde_derive::{Serialize, Deserialize};
use serde_json::Value;

use super::message_handler::MessageHandler;
use super::network_manager::NetworkManager;

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct NetworkConfig {
    range: f32,
    delay: f32
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            range: f32::INFINITY,
            delay: 0.
        }
    }
}

pub struct Network {
    from: String,
    range: f32,
    delay: f32,
    network_manager: Option<Arc<RwLock<NetworkManager>>>,
    message_handlers: Vec<Arc<RwLock<dyn MessageHandler>>>
}

impl fmt::Debug for Network {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("")
         .field(&self.from)
         .field(&self.range)
         .field(&self.delay)
         .finish()
    }
}

impl Network {
    pub fn new(from: String) -> Network {
        Network::from_config(from,&NetworkConfig::default())
    }

    pub fn from_config(from: String, config: &NetworkConfig) -> Network {
        Network {
            from,
            range: config.range,
            delay: config.delay,
            network_manager: None,
            message_handlers: Vec::new()
        }
    }

    pub fn set_network_manager(&mut self, network_manager: Arc<RwLock<NetworkManager>>) {
        self.network_manager = Some(network_manager);
    }

    pub fn send_to(&mut self, recipient: String, message: Value) {
        assert!(self.network_manager.is_some(), "Manager should be set.");
        println!("Sent to");
        self.network_manager.as_mut().unwrap().write().unwrap().send_from_to(self.from.clone(), recipient, message);
        println!("Out of Sent to");
    }

    pub fn broadcast(&mut self, message: Value) {
        assert!(self.network_manager.is_some(), "Manager should be set.");
        println!("Broadcast");
        self.network_manager.as_mut().unwrap().write().unwrap().broadcast_from(self.from.clone(), message);
        println!("Out of Broadcast");
    }

    pub fn receive(&mut self, from: String, message: Value) {
        println!("Receive message from {from}:\n{:?}", message);
        // TODO: add delay and range
        for handler in &self.message_handlers {
            if handler.write().unwrap().handle_message(&from, &message).is_ok() {
                break;
            }
        }
    }

    pub fn subscribe(&mut self, handler: Arc<RwLock<dyn MessageHandler>>) {
        self.message_handlers.push(handler);
    }
}