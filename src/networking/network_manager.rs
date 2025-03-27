/*!
Provide the Manager of the robots [`Network`]s. Only one should exist for one
[`Simulator`](crate::simulator::Simulator).
*/

use crate::robot::Robot;

use super::network::Network;
use std::collections::BTreeMap;

use std::sync::{Arc, RwLock};

/// Manages the [`Network`]s, making the link between them, and keep a list.
#[derive(Debug)]
pub struct NetworkManager {
    robots_networks: BTreeMap<String, Arc<RwLock<Network>>>,
    robots: BTreeMap<String, Arc<RwLock<Robot>>>,
}

impl NetworkManager {
    pub fn new() -> Self {
        Self {
            robots_networks: BTreeMap::new(),
            robots: BTreeMap::new(),
        }
    }

    /// Add a new [`Network`] node to the network. It creates the links to each existing network.
    ///
    /// ## Argument
    /// * `robot` - Reference to the [`Robot`](crate::robot::Robot).
    /// * `network` - [`Network`] to add.
    pub fn register_robot_network(&mut self, robot: &Robot) {
        let robot_name = robot.name();
        self.robots_networks.insert(
            robot_name.clone(),
            Arc::clone(&robot.network()),
        );
        // self.robots.insert(robot_name.clone(), Arc::clone(&robot));
        // for (other_robot, other_network) in &self.robots_networks {
        //     if other_robot.clone() != robot_name.clone() {
        //         let other_emitter = other_network.write().unwrap().get_emitter();
        //         robot
        //             .write()
        //             .unwrap()
        //             .network()
        //             .write()
        //             .unwrap()
        //             .add_emitter(other_robot.clone(), other_emitter);
        //         let emitter = robot
        //             .write()
        //             .unwrap()
        //             .network()
        //             .write()
        //             .unwrap()
        //             .get_emitter();
        //         other_network
        //             .write()
        //             .unwrap()
        //             .add_emitter(robot_name.clone(), emitter)
        //     }
        // }
    }

    /// Compute the distance between two robots at the given time, using their real pose.
    pub fn distance_between(&self, robot1: &String, robot2: &String, time: f32) -> f32 {
        // let robot1_pos = self
        //     .robots
        //     .get(robot1)
        //     .expect(format!("Robot '{}' not found", robot1).as_str())
        //     .read()
        //     .unwrap()
        //     .physics()
        //     .read()
        //     .unwrap()
        //     .state(time)
        //     .pose;
        // let robot2_pos = self
        //     .robots
        //     .get(robot2)
        //     .expect(format!("Robot '{}' not found", robot2).as_str())
        //     .read()
        //     .unwrap()
        //     .physics()
        //     .read()
        //     .unwrap()
        //     .state(time)
        //     .pose;

        // let distance = (robot1_pos.rows(0, 2) - robot2_pos.rows(0, 2)).norm();
        let distance= 0.;
        distance
    }
}
