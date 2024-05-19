/*!
Provide the Manager of the robots [`Network`]s. Only one should exist for one
[`Simulator`](crate::simulator::Simulator).
*/

use crate::turtlebot::Turtlebot;

use super::network::Network;
use std::collections::BTreeMap;

use std::sync::{Arc, RwLock};

/// Manages the [`Network`]s, making the link between them, and keep a list.
#[derive(Debug)]
pub struct NetworkManager {
    turtles_networks: BTreeMap<String, Arc<RwLock<Network>>>,
    turtles: BTreeMap<String, Arc<RwLock<Turtlebot>>>,
}

impl NetworkManager {
    pub fn new() -> Self {
        Self {
            turtles_networks: BTreeMap::new(),
            turtles: BTreeMap::new(),
        }
    }

    /// Add a new [`Network`] node to the network. It creates the links to each existing network.
    ///
    /// ## Argument
    /// * `turtle` - Reference to the [`Turtlebot`](crate::turtlebot::Turtlebot).
    /// * `network` - [`Network`] to add.
    pub fn register_turtle_network(
        &mut self,
        turtle: Arc<RwLock<Turtlebot>>,
        network: Arc<RwLock<Network>>,
    ) {
        let turtle_name = turtle.read().unwrap().name();
        self.turtles_networks
            .insert(turtle_name.clone(), Arc::clone(&network));
        self.turtles
            .insert(turtle_name.clone(), Arc::clone(&turtle));
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

    /// Compute the distance between two turtles at the given time, using their real pose.
    pub fn distance_between(&self, turtle1: &String, turtle2: &String, time: f32) -> f32 {
        let turtle1_pos = self
            .turtles
            .get(turtle1)
            .expect(format!("Turtle '{}' not found", turtle1).as_str())
            .read()
            .unwrap()
            .physics()
            .read()
            .unwrap()
            .state(time)
            .pose;
        let turtle2_pos = self
            .turtles
            .get(turtle2)
            .expect(format!("Turtle '{}' not found", turtle2).as_str())
            .read()
            .unwrap()
            .physics()
            .read()
            .unwrap()
            .state(time)
            .pose;

        let distance = (turtle1_pos.rows(0, 2) - turtle2_pos.rows(0, 2)).norm();

        return distance;
    }
}
