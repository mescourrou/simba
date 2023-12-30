use serde_json::Value;

use super::network::Network;
use std::collections::BTreeMap;

use std::sync::{Arc, RwLock};

#[derive(Debug)]
pub struct NetworkManager {
    turtles_networks: BTreeMap<String, Arc<RwLock<Network>>>
}

impl NetworkManager {
    pub fn new() -> Self {
        Self {
            turtles_networks: BTreeMap::new()
        }
    }

    pub fn register_turtle_network(&mut self, turtle_name: String, network: Arc<RwLock<Network>>) {
        self.turtles_networks.insert(turtle_name, Arc::clone(&network));
    }

    pub fn send_from_to(&mut self, from: String, to: String, message: Value) {
        if from == to {
            println!("Impossible to send a message to ourselve !");
        } else {
            let network_option = self.turtles_networks.get(&to);
            if let Some(network) = network_option {
                network.write().unwrap().receive(from, message);
            }
        }
    }
}