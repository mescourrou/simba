/*!
Provide the Manager of the robots [`Network`]s. Only one should exist for one
[`Simulator`](crate::simulator::Simulator).
*/

use super::network::Network;
use std::collections::BTreeMap;

use std::sync::{Arc, RwLock};

/// Manages the [`Network`]s, making the link between them, and keep a list.
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

    /// Add a new [`Network`] node to the network. It creates the links to each existing network.
    ///
    /// ## Argument
    /// * `turtle_name` - Name of the [`Turtlebot`](crate::turtlebot::Turtlebot).
    /// * `network` - [`Network`] to add.
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
}
