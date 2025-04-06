/*!
Provide the Manager of the robots [`Network`]s. Only one should exist for one
[`Simulator`](crate::simulator::Simulator).
*/

use log::debug;
use serde_json::Value;

use crate::robot::Robot;
use crate::simulator::TimeCvData;
use crate::state_estimators::state_estimator::State;
use crate::utils::time_ordered_data::TimeOrderedData;

use super::network::{MessageFlag, Network};
use std::collections::{BTreeMap, HashMap};

use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Condvar, Mutex, RwLock};

#[derive(Debug, Clone)]
pub enum MessageSendMethod {
    Broadcast,
    Recipient(String),
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

/// Manages the [`Network`]s, making the link between them, and keep a list.
#[derive(Debug)]
pub struct NetworkManager {
    robots_senders: BTreeMap<String, Sender<NetworkMessage>>,
    robots_receivers: BTreeMap<String, Receiver<NetworkMessage>>,
    time_cv: Arc<(Mutex<TimeCvData>, Condvar)>,
}

impl NetworkManager {
    pub fn new(time_cv: Arc<(Mutex<TimeCvData>, Condvar)>) -> Self {
        Self {
            robots_senders: BTreeMap::new(),
            robots_receivers: BTreeMap::new(),
            time_cv,
        }
    }

    /// Add a new [`Network`] node to the network. It creates the links to each existing network.
    ///
    /// ## Argument
    /// * `robot` - Reference to the [`Robot`](crate::robot::Robot).
    /// * `network` - [`Network`] to add.
    pub fn register_robot_network(&mut self, robot: &mut Robot) {
        let robot_name = robot.name();

        let to_robot = mpsc::channel();
        let from_robot = mpsc::channel();
        self.robots_senders.insert(robot_name.clone(), to_robot.0);
        self.robots_receivers
            .insert(robot_name.clone(), from_robot.1);

        robot
            .network()
            .write()
            .unwrap()
            .set_network_manager_link(from_robot.0, to_robot.1);
    }

    /// Compute the distance between two robots at the given time, using their real pose.
    fn distance_between(
        position_history: &HashMap<String, TimeOrderedData<State>>,
        robot1: &String,
        robot2: &String,
        time: f32,
    ) -> f32 {
        let robot1_pos = position_history
            .get(robot1)
            .expect(format!("Unknown robot {robot1}").as_str())
            .get_data_at_time(time)
            .expect(format!("No state data for robot {robot1} at time {time}").as_str())
            .1
            .pose;
        let robot2_pos = position_history
            .get(robot2)
            .expect(format!("Unknown robot {robot2}").as_str())
            .get_data_at_time(time)
            .expect(format!("No state data for robot {robot2} at time {time}").as_str())
            .1
            .pose;

        let distance = (robot1_pos.rows(0, 2) - robot2_pos.rows(0, 2)).norm();
        distance
    }

    pub fn process_messages(&self, position_history: &HashMap<String, TimeOrderedData<State>>) {
        let mut message_sent = false;
        for (robot_name, receiver) in self.robots_receivers.iter() {
            if let Ok(msg) = receiver.try_recv() {
                match &msg.to {
                    MessageSendMethod::Recipient(r) => {
                        if msg.range == 0.
                            || msg.range
                                >= NetworkManager::distance_between(
                                    position_history,
                                    robot_name,
                                    r,
                                    msg.time,
                                )
                        {
                            debug!("Receiving message from `{robot_name}` for `{r}`... Sending");
                            self.robots_senders
                                .get(r)
                                .expect(format!("Unknown robot {r}").as_str())
                                .send(msg)
                                .unwrap();
                            message_sent = true;
                        } else {
                            debug!(
                                "Receiving message from `{robot_name}` for `{r}`... Out of range"
                            );
                        }
                    }
                    MessageSendMethod::Broadcast => {
                        for (recipient_name, sender) in self.robots_senders.iter() {
                            if msg.range == 0.
                                || msg.range
                                    >= NetworkManager::distance_between(
                                        position_history,
                                        robot_name,
                                        recipient_name,
                                        msg.time,
                                    )
                            {
                                debug!("Receiving message from `{robot_name}` for broadcast... Sending to `{recipient_name}`");
                                sender.send(msg.clone()).unwrap();
                                message_sent = true;
                            }
                        }
                    }
                }
            }
        }
        if message_sent {
            let _lk = self.time_cv.0.lock();
            self.time_cv.1.notify_all();
        }
    }
}
