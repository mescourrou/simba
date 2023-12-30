extern crate confy;
use std::sync::{Arc, RwLock};

use serde_derive::{Serialize, Deserialize};

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

#[derive(Debug)]
pub struct Network {
    range: f32,
    delay: f32,
    network_manager: Option<Arc<RwLock<NetworkManager>>>
}

impl Network {
    pub fn new() -> Network {
        Network::from_config(&NetworkConfig::default())
    }

    pub fn from_config(config: &NetworkConfig) -> Network {
        Network {
            range: config.range,
            delay: config.delay,
            network_manager: None
        }
    }

    pub fn set_network_manager(&mut self, network_manager: Arc<RwLock<NetworkManager>>) {
        self.network_manager = Some(network_manager);
    }
}