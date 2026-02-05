use std::{collections::HashMap};

use crate::pub_sub::Client;


pub struct MultiClient<KeyType, MessageType, ConditionArgType = u8>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone,
    MessageType: Clone + Send + 'static,
    ConditionArgType: Clone + Send + 'static,
{
    clients: HashMap<KeyType, Client<MessageType, ConditionArgType>>,
}

impl<KeyType, MessageType, ConditionArgType> MultiClient<KeyType, MessageType, ConditionArgType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone,
    MessageType: Clone + Send + 'static,
    ConditionArgType: Clone + Send + 'static,
{
    pub fn new() -> Self {
        Self {
            clients: HashMap::new(),
        }
    }

    pub fn add_client(&mut self, key: &KeyType, client: Client<MessageType, ConditionArgType>) {
        self.clients.insert(key.clone(), client);
    }

    pub fn send(&self, key: &KeyType, message: MessageType, condition_arg: ConditionArgType, time: f32) {
        if let Some(client) = self.clients.get(key) {
            client.send(message.clone(), condition_arg.clone(), time);
        }
    }

    pub fn try_receive(&self, condition_arg: &ConditionArgType, time: f32) -> Option<(KeyType, MessageType)> {
        for (key, client) in &self.clients {
            if let Some(message) = client.try_receive(condition_arg, time) {
                return Some((key.clone(), message));
            }
        }
        None
    }
}

