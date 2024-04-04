use serde_json::Value;

use super::network::Network;
use std::collections::BTreeMap;

use std::sync::{Arc, RwLock};

#[derive(Debug)]
pub struct NetworkManager {
    turtles_networks: BTreeMap<String, Arc<RwLock<Network>>>,
}

impl NetworkManager {
    pub fn new() -> Self {
        Self {
            turtles_networks: BTreeMap::new(),
        }
    }

    pub fn register_turtle_network(&mut self, turtle_name: String, network: Arc<RwLock<Network>>) {
        self.turtles_networks
            .insert(turtle_name.clone(), Arc::clone(&network));
        for (other_turtle, other_network) in &self.turtles_networks {
            if other_turtle.clone() != turtle_name.clone() {
                let other_emitter = other_network.write().unwrap().get_emitter();
                network
                    .write()
                    .unwrap()
                    .add_emitter(other_turtle.clone(), other_emitter);
                let emitter = network.write().unwrap().get_emitter();
                other_network
                    .write()
                    .unwrap()
                    .add_emitter(turtle_name.clone(), emitter)
            }
        }
    }

    // pub fn send_from_to(&mut self, from: String, to: String, message: Value) {
    //     println!("Send from {} to {}", from, to);
    //     if from == to {
    //         println!("Impossible to send a message to ourselve !");
    //     } else {
    //         let network_option = self.turtles_networks.get(&to);
    //         if let Some(network) = network_option {
    //             println!("Calling receive from {} to {} !", from, to);
    //             network.write().unwrap().receive(from, message);
    //             println!("Call success");
    //         }
    //     }
    // }

    // pub fn broadcast_from(&mut self, from: String, message: Value) {
    //     println!("Send broadcas from {}", from);
    //     for (to, network) in &self.turtles_networks {
    //         if from != to.clone() {
    //             println!("Calling receive broadcast {} !", from);
    //             network.write().unwrap().receive(from.clone(), message.clone());
    //             println!("Call success");
    //         }
    //     }
    // }
}
