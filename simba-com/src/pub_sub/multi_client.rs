use std::{
    collections::HashMap,
    fmt::{Debug, Display},
    sync::{Arc, RwLock},
};

use log::{debug, warn};

use crate::pub_sub::{BrokerTrait, Client, PathKey};

pub trait MultiClientTrait<KeyType, MessageType, NodeIdType>: Send + Sync + Debug
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync,
    MessageType: Clone + Send + 'static + Default,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static,
{
    fn add_client(&mut self, key: &KeyType, client: Client<MessageType>);
    fn subscribe(&mut self, key: &KeyType);
    fn subscribe_instantaneous(&mut self, key: &KeyType);

    fn send(&self, key: &KeyType, message: MessageType, time: f32);
    fn try_receive(&self, time: f32) -> Option<(KeyType, MessageType)>;
    fn next_message_time(&self) -> Option<f32>;

    fn subscribed_keys(&self) -> Vec<KeyType>;
    fn node_id(&self) -> &NodeIdType;

    fn transform_key(&self, key: &KeyType) -> KeyType { key.clone() }
}

#[derive(Debug)]
pub struct MultiClient<KeyType, MessageType, NodeIdType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync,
    MessageType: Clone + Send + 'static + Default,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static,
{
    clients: HashMap<KeyType, Client<MessageType>>,
    broker: Arc<RwLock<dyn BrokerTrait<KeyType, MessageType, NodeIdType>>>,
    reception_delay: f32,
    node_id: NodeIdType,
}

impl<KeyType, MessageType, NodeIdType> MultiClient<KeyType, MessageType, NodeIdType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync + Debug + Display,
    MessageType: Clone + Send + 'static + Default + Debug + Sync,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
{
    pub fn new(
        broker: Arc<RwLock<dyn BrokerTrait<KeyType, MessageType, NodeIdType>>>,
        node_id: NodeIdType,
        reception_delay: f32,
    ) -> Self {
        Self {
            clients: HashMap::new(),
            broker,
            reception_delay,
            node_id,
        }
    }
}

impl<KeyType, MessageType, NodeIdType> MultiClientTrait<KeyType, MessageType, NodeIdType>
    for MultiClient<KeyType, MessageType, NodeIdType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync + Debug + Display,
    MessageType: Clone + Send + 'static + Default + Debug + Sync,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
{
    fn add_client(&mut self, key: &KeyType, client: Client<MessageType>) {
        self.clients.insert(key.clone(), client);
    }

    fn subscribe(&mut self, key: &KeyType) {
        if self.clients.get(key).is_none() {
            let client = self
                .broker
                .write()
                .unwrap()
                .subscribe_to(key, self.node_id.clone(), self.reception_delay)
                .unwrap();
            self.add_client(key, client);
        }
    }

    fn subscribe_instantaneous(&mut self, key: &KeyType) {
        if self.clients.get(key).is_none() {
            let client = self
                .broker
                .write()
                .unwrap()
                .subscribe_to(key, self.node_id.clone(), 0.0)
                .unwrap();
            self.add_client(key, client);
        }
    }

    fn send(&self, key: &KeyType, message: MessageType, time: f32) {
        if let Some(client) = self.clients.get(key) {
            debug!("Sending message on key {:?} at time {}: {:?}", key, time, message);
            client.send(message.clone(), time);
        } else {
            if let Some(tmp_client) =
                self.broker
                    .write()
                    .unwrap()
                    .subscribe_to(key, self.node_id.clone(), 0.0)
            {
                debug!("Sending message on key {:?} at time {} with tmp client: {:?}", key, time, message);
                tmp_client.send(message.clone(), time);
            } else {
                warn!("Trying to send a message to '{}' that is not created", key);
            }
        }
    }

    fn try_receive(&self, time: f32) -> Option<(KeyType, MessageType)> {
        for (key, client) in &self.clients {
            if let Some(message) = client.try_receive(time) {
                return Some((key.clone(), message));
            }
        }
        None
    }

    fn next_message_time(&self) -> Option<f32> {
        let mut next_time = None;
        for client in self.clients.values() {
            if let Some(client_next_time) = client.next_message_time() {
                next_time = Some(client_next_time.min(next_time.unwrap_or(client_next_time)));
            }
        }
        next_time
    }

    fn subscribed_keys(&self) -> Vec<KeyType> {
        self.clients.keys().cloned().collect()
    }

    fn node_id(&self) -> &NodeIdType {
        &self.node_id
    }
}

#[derive(Debug)]
pub struct PathMultiClient<MessageType, NodeIdType>
where
    MessageType: Clone + Send + 'static + Default + Debug + Sync,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
{
    multi_client: MultiClient<PathKey, MessageType, NodeIdType>,
    base_path: PathKey,
}

impl<MessageType, NodeIdType> PathMultiClient<MessageType, NodeIdType>
where
    MessageType: Clone + Send + 'static + Default + Debug + Sync,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
{
    pub fn new(
        broker: Arc<RwLock<dyn BrokerTrait<PathKey, MessageType, NodeIdType>>>,
        node_id: NodeIdType,
        reception_delay: f32,
        client_path: PathKey,
    ) -> Self {
        assert!(client_path.absolute(), "The client path should be absolute");
        Self {
            multi_client: MultiClient::new(broker, node_id, reception_delay),
            base_path: client_path,
        }
    }
}

impl<MessageType, NodeIdType> MultiClientTrait<PathKey, MessageType, NodeIdType>
    for PathMultiClient<MessageType, NodeIdType>
where
    MessageType: Clone + Send + 'static + Default + Debug + Sync,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
{
    fn add_client(&mut self, key: &PathKey, client: Client<MessageType>) {
        self.multi_client.add_client(key, client);
    }

    fn subscribe(&mut self, key: &PathKey) {
        let key = self.transform_key(key);
        self.multi_client.subscribe(&key);
    }

    fn subscribe_instantaneous(&mut self, key: &PathKey) {
        let key = self.transform_key(key);
        self.multi_client.subscribe_instantaneous(&key);
    }

    fn send(&self, key: &PathKey, message: MessageType, time: f32) {
        let key = self.transform_key(key);
        self.multi_client.send(&key, message.clone(), time);
    }

    fn try_receive(&self, time: f32) -> Option<(PathKey, MessageType)> {
        self.multi_client.try_receive(time)
    }

    fn next_message_time(&self) -> Option<f32> {
        self.multi_client.next_message_time()
    }

    fn subscribed_keys(&self) -> Vec<PathKey> {
        self.multi_client.subscribed_keys()
    }

    fn node_id(&self) -> &NodeIdType {
        self.multi_client.node_id()
    }

    fn transform_key(&self, key: &PathKey) -> PathKey {
        if key.absolute() {
            key.clone()
        } else {
            self.base_path.join(key)
        }
    }
}
