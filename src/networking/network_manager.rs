/*!
Provide the Manager of the nodes [`Network`]s. Only one should exist for one
[`Simulator`](crate::simulator::Simulator).
*/

use log::debug;
use serde_json::Value;

use crate::logger::is_enabled;
use crate::node::Node;
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
    nodes_senders: BTreeMap<String, Sender<NetworkMessage>>,
    nodes_receivers: BTreeMap<String, Receiver<NetworkMessage>>,
    time_cv: Arc<(Mutex<TimeCvData>, Condvar)>,
}

impl NetworkManager {
    pub fn new(time_cv: Arc<(Mutex<TimeCvData>, Condvar)>) -> Self {
        Self {
            nodes_senders: BTreeMap::new(),
            nodes_receivers: BTreeMap::new(),
            time_cv,
        }
    }

    /// Add a new [`Network`] node to the network. It creates the links to each existing network.
    ///
    /// ## Argument
    /// * `node` - Reference to the [`Node`].
    /// * `network` - [`Network`] to add.
    pub fn register_node_network(&mut self, node: &mut Node) {
        if let Some(network) = node.network() {
            let node_name = node.name();

            let to_node = mpsc::channel();
            let from_node = mpsc::channel();
            self.nodes_senders.insert(node_name.clone(), to_node.0);
            self.nodes_receivers.insert(node_name.clone(), from_node.1);

            network
                .write()
                .unwrap()
                .set_network_manager_link(from_node.0, to_node.1);
        }
    }

    /// Compute the distance between two nodes at the given time, using their real pose.
    fn distance_between(
        position_history: &HashMap<String, TimeOrderedData<State>>,
        node1: &String,
        node2: &String,
        time: f32,
    ) -> f32 {
        let node1_pos = position_history
            .get(node1)
            .expect(format!("Unknown node {node1}").as_str())
            .get_data_at_time(time)
            .expect(format!("No state data for node {node1} at time {time}").as_str())
            .1
            .pose;
        let node2_pos = position_history
            .get(node2)
            .expect(format!("Unknown node {node2}").as_str())
            .get_data_at_time(time)
            .expect(format!("No state data for node {node2} at time {time}").as_str())
            .1
            .pose;

        let distance = (node1_pos.rows(0, 2) - node2_pos.rows(0, 2)).norm();
        distance
    }

    pub fn process_messages(&self, position_history: &HashMap<String, TimeOrderedData<State>>) {
        let mut message_sent = false;
        for (node_name, receiver) in self.nodes_receivers.iter() {
            if let Ok(msg) = receiver.try_recv() {
                match &msg.to {
                    MessageSendMethod::Recipient(r) => {
                        if msg.range == 0.
                            || msg.range
                                >= NetworkManager::distance_between(
                                    position_history,
                                    node_name,
                                    r,
                                    msg.time,
                                )
                        {
                            if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                                debug!("Receiving message from `{node_name}` for `{r}`... Sending");
                            }
                            self.nodes_senders
                                .get(r)
                                .expect(format!("Unknown node {r}").as_str())
                                .send(msg)
                                .unwrap();
                            message_sent = true;
                        } else {
                            if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                                debug!(
                                    "Receiving message from `{node_name}` for `{r}`... Out of range"
                                );
                            }
                        }
                    }
                    MessageSendMethod::Broadcast => {
                        for (recipient_name, sender) in self.nodes_senders.iter() {
                            if msg.range == 0.
                                || msg.range
                                    >= NetworkManager::distance_between(
                                        position_history,
                                        node_name,
                                        recipient_name,
                                        msg.time,
                                    )
                            {
                                if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                                    debug!("Receiving message from `{node_name}` for broadcast... Sending to `{recipient_name}`");
                                }
                                sender.send(msg.clone()).unwrap();
                                message_sent = true;
                            }
                        }
                    }
                }
            }
        }
        if message_sent {
            if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                debug!("Wait for CV lock");
            }
            let lk = self.time_cv.0.lock().unwrap();
            if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                debug!("Got CV lock");
            }
            self.time_cv.1.notify_all();
            if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                debug!("Release CV lock");
            }
            std::mem::drop(lk);
        }
    }
}
