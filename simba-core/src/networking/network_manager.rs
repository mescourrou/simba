/*!
Provide the Manager of the nodes [`Network`](crate::networking::network::Network)s. Only one should exist for one
[`Simulator`](crate::simulator::Simulator).
*/

use log::debug;
use serde_json::Value;
use simba_com::pub_sub::{BrokerTrait, BrokerTraitProcessing, PathBroker};
use simba_com::time_ordered_data::TimeOrderedData;

use crate::constants::TIME_ROUND;
use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
use crate::logger::is_enabled;
use crate::networking::NetworkError;
use crate::networking::network::Envelope;
use crate::node::{Node, NodeState};
use crate::simulator::{SimbaBroker, TimeCv};
use crate::state_estimators::State;
use crate::utils::SharedRwLock;

use super::network::MessageFlag;
use std::collections::{BTreeMap, HashMap};

use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, RwLock};

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
    simulator_messages: Vec<NetworkMessage>,
    broker: SharedRwLock<SimbaBroker>,
}

impl NetworkManager {
    pub fn new(time_cv: Arc<TimeCv>) -> Self {
        Self {
            nodes_senders: BTreeMap::new(),
            nodes_receivers: BTreeMap::new(),
            time_cv,
            simulator_messages: Vec::new(),
            broker: Arc::new(RwLock::new(PathBroker::new(TIME_ROUND))),
        }
    }

    // /// Add a new [`Network`](crate::networking::network::Network) node to the network. It creates the links to each existing network.
    // ///
    // /// ## Argument
    // /// * `node` - Reference to the [`Node`].
    // pub fn register_node_network(&mut self, node: &mut Node) {
    //     if let Some(network) = node.network() {
    //         let node_name = node.name();

    //         let to_node = mpsc::channel();
    //         let from_node = mpsc::channel();
    //         self.nodes_senders.insert(node_name.clone(), to_node.0);
    //         self.nodes_receivers.insert(node_name.clone(), from_node.1);
    //         if is_enabled(crate::logger::InternalLog::NetworkMessages) {
    //             debug!("Add node `{node_name}` to senders and receivers");
    //         }
    //         network
    //             .write()
    //             .unwrap()
    //             .set_network_manager_link(from_node.0, to_node.1);
    //     }
    // }

    // /// Compute the distance between two nodes at the given time, using their real pose.
    // fn distance_between(
    //     position_history: &BTreeMap<String, TimeOrderedData<(State, NodeState)>>,
    //     node1: &String,
    //     node2: &String,
    //     time: f32,
    // ) -> SimbaResult<f32> {
    //     let node1_pos = position_history
    //         .get(node1)
    //         .ok_or(SimbaError::new(
    //             SimbaErrorTypes::NetworkError(NetworkError::NodeUnknown),
    //             format!("Unknown node {node1} to compute distance"),
    //         ))?
    //         .get_data_at_time(time)
    //         .ok_or(SimbaError::new(
    //             SimbaErrorTypes::NetworkError(NetworkError::Other),
    //             format!("No state data for node {node1} at time {time}"),
    //         ))?
    //         .1
    //         .0
    //         .pose;
    //     let node2_pos = position_history
    //         .get(node2)
    //         .ok_or(SimbaError::new(
    //             SimbaErrorTypes::NetworkError(NetworkError::NodeUnknown),
    //             format!("Unknown node {node2} to compute distance"),
    //         ))?
    //         .get_data_at_time(time)
    //         .ok_or(SimbaError::new(
    //             SimbaErrorTypes::NetworkError(NetworkError::Other),
    //             format!("No state data for node {node2} at time {time}"),
    //         ))?
    //         .1
    //         .0
    //         .pose;

    //     let distance = (node1_pos.rows(0, 2) - node2_pos.rows(0, 2)).norm();
    //     Ok(distance)
    // }

    pub fn process_messages(
        &mut self,
        position_map: &HashMap<String, Option<[f32; 2]>>,
    ) -> SimbaResult<()> {
        // let time_cv = self.time_cv.clone();
        // let _lk = time_cv.waiting.lock().unwrap();

        // // Keep the lock for all the processing, otherwise nodes can think that all messages are treated between the decrease and the increase of the counter
        // let mut circulating_messages = time_cv.circulating_messages.lock().unwrap();
        // let mut nodes_to_remove = Vec::new();
        // let mut message_sent = false;
        // let mut messages = Vec::new();
        // for (node_name, receiver) in self.nodes_receivers.iter() {
        //     if let Ok(msg) = receiver.try_recv() {
        //         *circulating_messages -= 1;
        //         messages.push((node_name.clone(), msg));
        //     }
        // }
        // for message in self.simulator_messages.drain(0..) {
        //     messages.push(("simulator".to_string(), message));
        // }

        // for (node_name, msg) in messages {
        //     match &msg.to {
        //         MessageSendMethod::Recipient(r) => {
        //             if msg.range == 0.
        //                 || msg.range
        //                     >= NetworkManager::distance_between(
        //                         position_history,
        //                         &node_name,
        //                         r,
        //                         msg.time,
        //                     )?
        //             {
        //                 if is_enabled(crate::logger::InternalLog::NetworkMessages) {
        //                     debug!("Receiving message from `{node_name}` for `{r}`... Sending");
        //                 }
        //                 let r = r.clone();
        //                 match self.nodes_senders.get(&r) {
        //                     Some(sender) => match sender.send(msg) {
        //                         Ok(_) => {
        //                             *circulating_messages += 1;
        //                             message_sent = true;
        //                         }
        //                         Err(e) => {
        //                             if message_sent {
        //                                 self.time_cv.condvar.notify_all();
        //                                 if is_enabled(crate::logger::InternalLog::NodeSyncDetailed)
        //                                 {
        //                                     debug!("Notify CV (network_manager line {})", line!());
        //                                 }
        //                             }
        //                             return Err(SimbaError::new(
        //                                 SimbaErrorTypes::NetworkError(NetworkError::Unknown(
        //                                     "SendError".to_string(),
        //                                 )),
        //                                 format!("Error while sending message to `{r}`: {}", e),
        //                             ));
        //                         }
        //                     },
        //                     None => {
        //                         if message_sent {
        //                             self.time_cv.condvar.notify_all();
        //                             if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
        //                                 debug!("Notify CV (network_manager line {})", line!());
        //                             }
        //                         }
        //                         return Err(SimbaError::new(
        //                             SimbaErrorTypes::NetworkError(NetworkError::NodeUnknown),
        //                             format!("Unknown recipient node `{r}`"),
        //                         ));
        //                     }
        //                 }
        //             } else if is_enabled(crate::logger::InternalLog::NetworkMessages) {
        //                 debug!("Receiving message from `{node_name}` for `{r}`... Out of range");
        //             }
        //         }
        //         MessageSendMethod::Broadcast => {
        //             for (recipient_name, sender) in self.nodes_senders.iter() {
        //                 if msg.range == 0.
        //                     || msg.range
        //                         >= NetworkManager::distance_between(
        //                             position_history,
        //                             &node_name,
        //                             recipient_name,
        //                             msg.time,
        //                         )?
        //                 {
        //                     if is_enabled(crate::logger::InternalLog::NetworkMessages) {
        //                         debug!(
        //                             "Receiving message from `{node_name}` for broadcast... Sending to `{recipient_name}`"
        //                         );
        //                     }
        //                     match sender.send(msg.clone()) {
        //                         Ok(_) => {
        //                             *circulating_messages += 1;
        //                             message_sent = true;
        //                         }
        //                         Err(e) => {
        //                             if message_sent {
        //                                 self.time_cv.condvar.notify_all();
        //                                 if is_enabled(crate::logger::InternalLog::NodeSyncDetailed)
        //                                 {
        //                                     debug!("Notify CV (network_manager line {})", line!());
        //                                 }
        //                             }
        //                             return Err(SimbaError::new(
        //                                 SimbaErrorTypes::NetworkError(NetworkError::Unknown(
        //                                     "SendError".to_string(),
        //                                 )),
        //                                 format!(
        //                                     "Error while sending message to `{recipient_name}`: {}",
        //                                     e
        //                                 ),
        //                             ));
        //                         }
        //                     }
        //                 }
        //             }
        //         }
        //         MessageSendMethod::Manager => {
        //             if is_enabled(crate::logger::InternalLog::NetworkMessages) {
        //                 debug!("Receiving message from `{node_name}` for Manager");
        //             }
        //             for flag in msg.message_flags {
        //                 if let MessageFlag::Unsubscribe = flag {
        //                     nodes_to_remove.push(node_name.clone());
        //                 }
        //             }
        //         }
        //     }
        // }
        // if message_sent {
        //     self.time_cv.condvar.notify_all();
        //     if is_enabled(crate::logger::InternalLog::NodeSyncDetailed) {
        //         debug!("Notify CV (network_manager line {})", line!());
        //     }
        // }
        // for node_name in nodes_to_remove {
        //     self.unsubscribe_node(&node_name);
        // }

        if is_enabled(crate::logger::InternalLog::NetworkMessages) {
            debug!("Processing messages");
        }
        self.broker
            .write()
            .unwrap()
            .process_messages(Some(&position_map));
        Ok(())
    }

    pub fn broker(&self) -> SharedRwLock<SimbaBroker> {
        self.broker.clone()
    }

    // fn unsubscribe_node(&mut self, node_name: &String) {
    //     if is_enabled(crate::logger::InternalLog::NetworkMessages) {
    //         debug!("Unsubscribe node `{node_name}` from manager");
    //     }
    //     self.nodes_receivers.remove(node_name).unwrap();
    //     self.nodes_senders.remove(node_name).unwrap();
    // }

    // #[cfg(not(feature = "force_hard_determinism"))]
    // pub(crate) fn send_message(
    //     &mut self,
    //     message: Value,
    //     send_method: MessageSendMethod,
    //     time: f32,
    //     flags: Vec<MessageFlag>,
    // ) -> SimbaResult<()> {
    //     let message = NetworkMessage {
    //         value: message,
    //         from: "simulator".to_string(),
    //         to: send_method,
    //         time,
    //         range: 0.,
    //         message_flags: flags,
    //     };
    //     if let MessageSendMethod::Recipient(to) = &message.to
    //         && !self.nodes_senders.contains_key(to)
    //     {
    //         return Err(SimbaError::new(
    //             SimbaErrorTypes::NetworkError(NetworkError::NodeUnknown),
    //             format!("Unknown recipient node `{to}`"),
    //         ));
    //     }
    //     self.simulator_messages.push(message);
    //     Ok(())
    // }
}
