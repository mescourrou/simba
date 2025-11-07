/*!
Provide the Manager of the nodes [`Network`](crate::networking::network::Network)s. Only one should exist for one
[`Simulator`](crate::simulator::Simulator).
*/

use log::debug;
use serde_json::Value;

use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
use crate::logger::is_enabled;
use crate::networking::NetworkError;
use crate::node::Node;
use crate::simulator::TimeCv;
use crate::state_estimators::state_estimator::State;
use crate::utils::time_ordered_data::TimeOrderedData;

use super::network::MessageFlag;
use std::collections::BTreeMap;

use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;

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
    nodes_senders: BTreeMap<String, Sender<NetworkMessage>>,
    nodes_receivers: BTreeMap<String, Receiver<NetworkMessage>>,
    time_cv: Arc<TimeCv>,
}

impl NetworkManager {
    pub fn new(time_cv: Arc<TimeCv>) -> Self {
        Self {
            nodes_senders: BTreeMap::new(),
            nodes_receivers: BTreeMap::new(),
            time_cv,
        }
    }

    /// Add a new [`Network`](crate::networking::network::Network) node to the network. It creates the links to each existing network.
    ///
    /// ## Argument
    /// * `node` - Reference to the [`Node`].
    pub fn register_node_network(&mut self, node: &mut Node) {
        if let Some(network) = node.network() {
            let node_name = node.name();

            let to_node = mpsc::channel();
            let from_node = mpsc::channel();
            self.nodes_senders.insert(node_name.clone(), to_node.0);
            self.nodes_receivers.insert(node_name.clone(), from_node.1);
            if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                debug!("Add node `{node_name}` to senders and receivers");
            }
            network
                .write()
                .unwrap()
                .set_network_manager_link(from_node.0, to_node.1);
        }
    }

    /// Compute the distance between two nodes at the given time, using their real pose.
    fn distance_between(
        position_history: &BTreeMap<String, TimeOrderedData<State>>,
        node1: &String,
        node2: &String,
        time: f32,
    ) -> SimbaResult<f32> {
        let node1_pos = position_history
            .get(node1)
            .ok_or(SimbaError::new(
                SimbaErrorTypes::NetworkError(NetworkError::NodeUnknown),
                format!("Unknown node {node1} to compute distance"),
            ))?
            .get_data_at_time(time)
            .ok_or(SimbaError::new(
                SimbaErrorTypes::NetworkError(NetworkError::Other),
                format!("No state data for node {node1} at time {time}"),
            ))?
            .1
            .pose;
        let node2_pos = position_history
            .get(node2)
            .ok_or(SimbaError::new(
                SimbaErrorTypes::NetworkError(NetworkError::NodeUnknown),
                format!("Unknown node {node2} to compute distance"),
            ))?
            .get_data_at_time(time)
            .ok_or(SimbaError::new(
                SimbaErrorTypes::NetworkError(NetworkError::Other),
                format!("No state data for node {node2} at time {time}"),
            ))?
            .1
            .pose;

        let distance = (node1_pos.rows(0, 2) - node2_pos.rows(0, 2)).norm();
        Ok(distance)
    }

    pub fn process_messages(
        &mut self,
        position_history: &BTreeMap<String, TimeOrderedData<State>>,
    ) -> SimbaResult<()> {
        let mut message_sent = false;

        let time_cv = self.time_cv.clone();
        let _lk = time_cv.waiting.lock().unwrap();

        // Keep the lock for all the processing, otherwise nodes can think that all messages are treated between the decrease and the increase of the counter
        let mut circulating_messages = time_cv.circulating_messages.lock().unwrap();
        let mut nodes_to_remove = Vec::new();
        for (node_name, receiver) in self.nodes_receivers.iter() {
            if let Ok(msg) = receiver.try_recv() {
                *circulating_messages -= 1;
                match &msg.to {
                    MessageSendMethod::Recipient(r) => {
                        if msg.range == 0.
                            || msg.range
                                >= NetworkManager::distance_between(
                                    position_history,
                                    node_name,
                                    r,
                                    msg.time,
                                )?
                        {
                            if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                                debug!("Receiving message from `{node_name}` for `{r}`... Sending");
                            }
                            match self.nodes_senders.get(r) {
                                Some(sender) => {
                                    *circulating_messages += 1;
                                    sender.send(msg).unwrap();
                                }
                                None => {
                                    if message_sent {
                                        self.time_cv.condvar.notify_all();
                                        if is_enabled(crate::logger::InternalLog::NodeSyncDetailed)
                                        {
                                            debug!("Notify CV");
                                        }
                                    }
                                    return Err(SimbaError::new(
                                        SimbaErrorTypes::NetworkError(NetworkError::NodeUnknown),
                                        format!("Unknown recipient node `{r}`"),
                                    ));
                                }
                            }
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
                                    )?
                            {
                                if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                                    debug!("Receiving message from `{node_name}` for broadcast... Sending to `{recipient_name}`");
                                }
                                *circulating_messages += 1;
                                sender.send(msg.clone()).unwrap();
                                message_sent = true;
                            }
                        }
                    }
                    MessageSendMethod::Manager => {
                        if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                            debug!("Receiving message from `{node_name}` for Manager");
                        }
                        for flag in msg.message_flags {
                            match flag {
                                MessageFlag::Unsubscribe => {
                                    nodes_to_remove.push(node_name.clone());
                                }
                                _ => (),
                            }
                        }
                    }
                }
            }
        }
        self.time_cv.condvar.notify_all();
        if message_sent {
            if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
                debug!("Notify CV");
            }
        }
        for node_name in nodes_to_remove {
            self.unsubscribe_node(&node_name);
        }
        Ok(())
    }

    fn unsubscribe_node(&mut self, node_name: &String) {
        if is_enabled(crate::logger::InternalLog::NetworkMessages) {
            debug!("Unsubscribe node `{node_name}` from manager");
        }
        self.nodes_receivers.remove(node_name).unwrap();
        self.nodes_senders.remove(node_name).unwrap();
    }
}
