use std::{
    collections::HashMap,
    fmt::Debug,
    hash::Hash,
    sync::{
        Arc, Mutex,
        mpsc::{self, Receiver, Sender},
    },
};

use log::debug;
use multimap::MultiMap;
use rand::random;

use crate::pub_sub::client::Client;

pub trait ChannelProcessing<NodeIdType, ConditionArgType>: Send + Sync + Debug {
    fn process_messages(
        &self,
        client_condition_args: Option<&HashMap<NodeIdType, ConditionArgType>>,
    );
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;
}

///
/// Lock order: receivers then senders
#[derive(Clone)]
pub struct Channel<
    MessageType: Clone + Send + 'static + Default,
    NodeIdType: Hash + Eq + Clone + Send + Sync + 'static,
    ConditionArgType: Clone + Send + 'static + Default = u8,
> {
    senders: Arc<Mutex<MultiMap<NodeIdType, (usize, Sender<(MessageType, f32)>)>>>,
    receivers: Arc<Mutex<MultiMap<NodeIdType, (usize, Receiver<(MessageType, f32)>)>>>,
    condition: Arc<Mutex<dyn Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static>>,
    time_round: f32,
    client_count: usize,
}

impl<
    MessageType: Clone + Send + 'static + Default,
    NodeIdType: Hash + Eq + Clone + Send + Sync + 'static,
    ConditionArgType: Clone + Send + 'static + Default,
> Debug for Channel<MessageType, NodeIdType, ConditionArgType>
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Channel")
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
    pub fn new(time_round: f32) -> Self {
        Self {
            senders: Arc::new(Mutex::new(MultiMap::new())),
            receivers: Arc::new(Mutex::new(MultiMap::new())),
            condition: Arc::new(Mutex::new(|_, _| true)),
            time_round,
            client_count: 0,
        }
    }

    pub fn new_conditionnal(
        condition: impl Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
        time_round: f32,
    ) -> Self {
        Self {
            senders: Arc::new(Mutex::new(MultiMap::new())),
            receivers: Arc::new(Mutex::new(MultiMap::new())),
            condition: Arc::new(Mutex::new(condition)),
            time_round,
            client_count: 0,
        }
    }

    pub fn client(&mut self, node_id: NodeIdType, reception_delay: f32) -> Client<MessageType> {
        let (to_client_tx, to_client_rx) = mpsc::channel();
        let (from_client_tx, from_client_rx) = mpsc::channel();
        let id = self.client_count;
        self.client_count += 1;
        self.receivers
            .lock()
            .unwrap()
            .insert(node_id.clone(), (id, from_client_rx));
        self.senders
            .lock()
            .unwrap()
            .insert(node_id.clone(), (id, to_client_tx));
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
        let mut dead_clients = Vec::new();
        // Lock sender and receiver to avoid list manipulation and keeping ids consistent between the receiving phase and the removing phase
        let mut receivers = self.receivers.lock().unwrap();
        let mut senders = self.senders.lock().unwrap();
        let mut messages_to_send = Vec::new();
        for (from_id, receivers) in receivers.iter_all() {
            for (i, receiver) in receivers.iter().enumerate() {
                while let Ok(message) = receiver.1.try_recv() {
                    if message.1 < 0. {
                        dead_clients.push((i, from_id.clone(), receiver.0));
                        break;
                    }
                    messages_to_send.push((from_id.clone(), receiver.0, message));
                }
            }
        }
        // Remove the dead clients
        for (i, key, sender_id) in dead_clients.into_iter().rev() {
            // Remove receivers
            let mut clients = receivers
                .remove(&key)
                .expect("Client to remove does not exist in receivers");
            let client_index = clients
                .iter()
                .position(|(id, _)| *id == sender_id)
                .expect("Client to remove does not exist in receivers");
            clients.remove(client_index);
            receivers.insert_many(key.clone(), clients);

            // Remove senders
            let mut clients = senders
                .remove(&key)
                .expect("Client to remove does not exist in senders");
            let client_index = clients
                .iter()
                .position(|(id, _)| *id == sender_id)
                .expect("Client to remove does not exist in senders");
            clients.remove(client_index);
            senders.insert_many(key.clone(), clients);
        }
        for (from_id, from_sender_id, message) in messages_to_send {
            let from_arg = client_condition_args.and_then(|args| args.get(&from_id));
            for (to_id, senders) in senders.iter_all() {
                let to_arg = if from_arg.is_some() {
                    client_condition_args.and_then(|args| args.get(to_id))
                } else {
                    None
                };
                let send = if to_arg.is_some() {
                    (self.condition.lock().unwrap())(
                        from_arg.unwrap().clone(),
                        to_arg.unwrap().clone(),
                    )
                } else {
                    true
                };
                for (sender_id, sender) in senders {
                    // Avoid sending the message back to the sender
                    if &from_id == to_id && *sender_id == from_sender_id {
                        continue;
                    }
                    if send {
                        if let Err(e) = sender.send(message.clone()) {
                            panic!(
                                "Failed to send message from {:?} to ({:?}, {}): {:?}",
                                from_id,
                                to_id,
                                *sender_id,
                                e.to_string()
                            );
                        } else {
                            debug!("Message from {:?} to {:?} sent", from_id, to_id);
                        }
                    } else {
                        debug!(
                            "Message from {:?} to {:?} not sent due to condition",
                            from_id, to_id
                        );
                    }
                }
            }
        }
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}
