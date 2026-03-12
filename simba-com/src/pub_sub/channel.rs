use std::{
    collections::{HashMap, HashSet},
    fmt::Debug,
    hash::Hash,
    sync::{
        Arc, Mutex,
        mpsc::{self, Receiver, Sender},
    },
};

#[cfg(feature = "debug_mode")]
use log::debug;

use crate::pub_sub::{SharedMutex, client::Client};

pub trait ChannelProcessing<NodeIdType, ConditionArgType>: Send + Sync + Debug {
    fn process_messages(
        &self,
        client_condition_args: Option<&HashMap<NodeIdType, ConditionArgType>>,
    );
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;
}

type SenderType<MessageType> = Sender<(MessageType, f32)>;
type ReceiverType<MessageType> = Receiver<(MessageType, f32)>;

///
/// Lock order: receivers then senders
#[derive(Clone)]
pub struct Channel<
    MessageType: Clone + Send + 'static + Default,
    NodeIdType: Hash + Eq + Clone + Send + Sync + 'static,
    ConditionArgType: Clone + Send + 'static + Default = u8,
> {
    senders: SharedMutex<HashMap<(NodeIdType, usize), SenderType<MessageType>>>,
    receivers: SharedMutex<HashMap<(NodeIdType, usize), ReceiverType<MessageType>>>,
    condition: SharedMutex<dyn Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static>,
    time_round: f32,
    client_count: SharedMutex<usize>,
    name: String,
}

impl<
    MessageType: Clone + Send + 'static + Default,
    NodeIdType: Hash + Eq + Clone + Send + Sync + 'static,
    ConditionArgType: Clone + Send + 'static + Default,
> Debug for Channel<MessageType, NodeIdType, ConditionArgType>
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Channel")
            .field("name", &self.name)
            .field("senders_count", &self.senders.lock().unwrap().len())
            .field("receivers_count", &self.receivers.lock().unwrap().len())
            .finish()
    }
}

impl<
    MessageType: Clone + Send + 'static + Default,
    NodeIdType: Hash + Eq + Clone + Send + Sync + 'static + Debug,
    ConditionArgType: Clone + Send + 'static + Default,
> Channel<MessageType, NodeIdType, ConditionArgType>
{
    pub fn new(time_round: f32, name: &str) -> Self {
        Self {
            senders: Arc::new(Mutex::new(HashMap::new())),
            receivers: Arc::new(Mutex::new(HashMap::new())),
            condition: Arc::new(Mutex::new(|_, _| true)),
            time_round,
            client_count: Arc::new(Mutex::new(0)),
            name: name.into(),
        }
    }

    pub fn new_conditionnal(
        condition: impl Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
        time_round: f32,
        name: &str,
    ) -> Self {
        Self {
            senders: Arc::new(Mutex::new(HashMap::new())),
            receivers: Arc::new(Mutex::new(HashMap::new())),
            condition: Arc::new(Mutex::new(condition)),
            time_round,
            client_count: Arc::new(Mutex::new(0)),
            name: name.into(),
        }
    }

    pub fn client(&mut self, node_id: NodeIdType, reception_delay: f32) -> Client<MessageType> {
        let (to_client_tx, to_client_rx) = mpsc::channel();
        let (from_client_tx, from_client_rx) = mpsc::channel();
        let mut client_count = self.client_count.lock().unwrap();
        let id = *client_count;
        *client_count += 1;
        self.receivers
            .lock()
            .unwrap()
            .insert((node_id.clone(), id), from_client_rx);
        self.senders
            .lock()
            .unwrap()
            .insert((node_id.clone(), id), to_client_tx);
        #[cfg(feature = "debug_mode")]
        debug!(
            "[Channel {}] New client with id {} for node {:?}. Total clients: {}",
            self.name, id, node_id, self.receivers.lock().unwrap().len()
        );
        Client::new(
            from_client_tx,
            to_client_rx,
            reception_delay,
            self.time_round,
        )
    }
}

impl<
    MessageType: Clone + Send + 'static + Default,
    NodeIdType: Hash + Eq + Clone + Send + Sync + 'static + Debug,
    ConditionArgType: Clone + Send + 'static + Default,
> ChannelProcessing<NodeIdType, ConditionArgType>
    for Channel<MessageType, NodeIdType, ConditionArgType>
{
    fn process_messages(
        &self,
        client_condition_args: Option<&HashMap<NodeIdType, ConditionArgType>>,
    ) {
        let mut dead_clients = HashSet::new();
        // Lock sender and receiver to avoid list manipulation and keeping ids consistent between the receiving phase and the removing phase
        let mut receivers = self.receivers.lock().unwrap();
        let mut senders = self.senders.lock().unwrap();
        let mut messages_to_send = Vec::new();
        for ((from_id, receiver_id), receiver) in receivers.iter() {
            while let Ok(message) = receiver.try_recv() {
                if message.1 < 0. {
                    #[cfg(feature = "debug_mode")]
                    debug!("[Channel {}] Receive end-client message", self.name);
                    dead_clients.insert((from_id.clone(), receiver_id.clone()));
                    continue;
                }
                messages_to_send.push((from_id.clone(), receiver_id.clone(), message));
            }
        }
        for (from_id, from_sender_id, message) in messages_to_send {
            let from_arg = client_condition_args.and_then(|args| args.get(&from_id));
            for ((to_id, sender_id), sender) in senders.iter() {
                let to_arg = if from_arg.is_some() {
                    client_condition_args.and_then(|args| args.get(to_id))
                } else {
                    None
                };
                let send = if let Some(to_arg) = to_arg {
                    (self.condition.lock().unwrap())(from_arg.unwrap().clone(), to_arg.clone())
                } else {
                    true
                };
                // Avoid sending the message back to the sender
                if &from_id == to_id && *sender_id == from_sender_id {
                    #[cfg(feature = "debug_mode")]
                    debug!(
                        "[Channel {}] Not sending message back to the sender ({:?} to {:?})",
                        self.name, from_id, to_id
                    );
                    continue;
                }
                if send {
                    if sender.send(message.clone()).is_err() {
                        // panic!(
                        //     "Failed to send message from {:?} to ({:?}, {}): {:?}",
                        //     from_id,
                        //     to_id,
                        //     *sender_id,
                        //     e.to_string()
                        // );
                        // Assume dead client
                        dead_clients.insert((to_id.clone(), *sender_id));
                    } else {
                        #[cfg(feature = "debug_mode")]
                        debug!("[Channel {}] Message from {:?} to {:?} sent", self.name, from_id, to_id);
                    }
                } else {
                    #[cfg(feature = "debug_mode")]
                    debug!(
                        "[Channel {}] Message from {:?} to {:?} not sent due to condition",
                        self.name, from_id, to_id
                    );
                }
            }
        }
        if dead_clients.is_empty() {
            return;
        }
        #[cfg(feature = "debug_mode")]
        {
            debug!("[Channel {}] Removing {} dead clients from channel", self.name, dead_clients.len());
            debug!(
                "[Channel {}] Current receiver list: {:?}",
                self.name,
                receivers.keys()
            );
            debug!(
                "[Channel {}] Current sender list: {:?}",
                self.name,
                senders
                    .keys()
            );
        }

        // Remove the dead clients
        for (key, sender_id) in dead_clients.into_iter() {
            #[cfg(feature = "debug_mode")]
            debug!(
                "[Channel {}] Removing client {:?} with sender id {} from channel",
                self.name, key, sender_id
            );
            // Remove receivers
            let _ = receivers
                .remove(&(key.clone(), sender_id))
                .expect("Client name to remove does not exist in receivers");

            // Remove senders
            let _ = senders
                .remove(&(key, sender_id))
                .expect("Client name to remove does not exist in senders");
        }
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}
