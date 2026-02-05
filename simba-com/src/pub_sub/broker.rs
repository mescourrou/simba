use std::collections::HashMap;

use crate::pub_sub::{Client, MultiClient, channel::{Channel, ChannelProcessing}};

pub struct Broker<KeyType>
where KeyType: std::cmp::Eq + std::hash::Hash,
{
    channels: HashMap<KeyType, Box<dyn ChannelProcessing>>,
    time_round: f32,
}

impl<KeyType> Broker<KeyType> 
where KeyType: std::cmp::Eq + std::hash::Hash,
{
    pub fn new(time_round: f32) -> Self {
        Self {
            channels: HashMap::new(),
            time_round,
        }
    }

    pub fn process_messages(&self) {
        for channel in self.channels.values() {
            channel.process_messages();
        }
    }

    pub fn add_channel_conditionnal<MessageType: Clone + Send + 'static, ConditionArgType: Clone + Send + 'static>(
        &mut self,
        key: KeyType,
        condition: impl Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
    ) {
        self.channels.insert(key, Box::new(Channel::<MessageType, ConditionArgType>::new_conditionnal(condition, self.time_round)));
    }

    pub fn add_channel<MessageType: Clone + Send + 'static>(
        &mut self,
        key: KeyType,
    ) {
        self.channels.insert(key, Box::new(Channel::<MessageType, ()>::new(self.time_round)));
    }

    pub fn get_channel_conditionnal<MessageType: Clone + Send + 'static, ConditionArgType: Clone + Send + 'static>(
        &mut self,
        key: &KeyType,
    ) -> Option<Channel<MessageType, ConditionArgType>> {
        self.channels.get_mut(key)?.as_any_mut().downcast_mut::<Channel<MessageType, ConditionArgType>>().cloned()
    }

    pub fn get_channel<MessageType: Clone + Send + 'static>(
        &mut self,
        key: &KeyType,
    ) -> Option<Channel<MessageType, ()>> {
        self.channels.get_mut(key)?.as_any_mut().downcast_mut::<Channel<MessageType, ()>>().cloned()
    }

    pub fn subscribe_to<MessageType: Clone + Send + 'static>(
        &mut self,
        key: &KeyType,
        reception_delay: f32,
    ) -> Option<Client<MessageType, ()>> {
        self.get_channel(key).map(|mut channel| channel.client(reception_delay))
    }

    pub fn subscribe_to_conditionnal<MessageType: Clone + Send + 'static, ConditionArgType: Clone + Send + 'static>(
        &mut self,
        key: &KeyType,
        reception_delay: f32,
    ) -> Option<Client<MessageType, ConditionArgType>> {
        self.get_channel_conditionnal(key).map(|mut channel| channel.client(reception_delay))
    }

    pub fn remove_channel(&mut self, key: &KeyType) {
        self.channels.remove(key);
    }

    pub fn channel_exists(&self, key: &KeyType) -> bool {
        self.channels.contains_key(key)
    }

    pub fn channel_list(&self) -> Vec<&KeyType> {
        self.channels.keys().collect()
    }
}

impl<KeyType> Broker<KeyType> 
where KeyType: std::cmp::Eq + std::hash::Hash + Clone,
{
    /// Subscribe to multiple channels at once. If one of the channels does not exist, return None and do not subscribe to any channel.
    /// The channels must have the same MessageType.
    pub fn subscribe_to_list<MessageType: Clone + Send + 'static>(
        &mut self,
        keys: &[KeyType],
        reception_delay: f32,
    ) -> Option<MultiClient<KeyType, MessageType, ()>> {
        let mut multi_client = MultiClient::new();
        for key in keys {
            if let Some(client) = self.subscribe_to(key, reception_delay) {
                multi_client.add_client(key, client);
            } else {
                return None;
            }
        }
        Some(multi_client)
    }

    /// Subscribe to multiple channels at once. If one of the channels does not exist, return None and do not subscribe to any channel.
    /// The channels must be conditionnal and have the same ConditionArgType.
    /// The channels must have the same MessageType.
    pub fn subscribe_to_list_conditionnal<MessageType: Clone + Send + 'static, ConditionArgType: Clone + Send + 'static>(
        &mut self,
        keys: &[KeyType],
        reception_delay: f32,
    ) -> Option<MultiClient<KeyType, MessageType, ConditionArgType>> {
        let mut multi_client = MultiClient::new();
        for key in keys {
            if let Some(client) = self.subscribe_to_conditionnal(key, reception_delay) {
                multi_client.add_client(key, client);
            } else {
                return None;
            }
        }
        Some(multi_client)
    }

}

pub struct GenericBroker<KeyType, MessageType, ConditionArgType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone,
    MessageType: Clone + Send + 'static,
    ConditionArgType: Clone + Send + 'static,
{
    inner: Broker<KeyType>,
    _phantom: std::marker::PhantomData<(MessageType, ConditionArgType)>,
}

impl<KeyType, MessageType, ConditionArgType> GenericBroker<KeyType, MessageType, ConditionArgType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone,
    MessageType: Clone + Send + 'static,
    ConditionArgType: Clone + Send + 'static,
{
    pub fn new(time_round: f32) -> Self {
        Self {
            inner: Broker::new(time_round),
            _phantom: std::marker::PhantomData
        }
    }

    pub fn process_messages(&self) {
        self.inner.process_messages();
    }

    pub fn add_channel(&mut self, key: KeyType) {
        self.inner.add_channel_conditionnal::<MessageType, ConditionArgType>(key, |_, _| true);
    }

    pub fn get_channel(&mut self, key: &KeyType) -> Option<Channel<MessageType, ConditionArgType>> {
        self.inner.get_channel_conditionnal(key)
    }

    pub fn subscribe_to(&mut self, key: &KeyType, reception_delay: f32) -> Option<Client<MessageType, ConditionArgType>> {
        self.inner.subscribe_to_conditionnal(key, reception_delay)
    }

    pub fn remove_channel(&mut self, key: &KeyType) {
        self.inner.remove_channel(key);
    }

    pub fn channel_exists(&self, key: &KeyType) -> bool {
        self.inner.channel_exists(key)
    }

    pub fn channel_list(&self) -> Vec<&KeyType> {
        self.inner.channel_list()
    }

    pub fn subscribe_to_list(&mut self, keys: &[KeyType], reception_delay: f32) -> Option<MultiClient<KeyType, MessageType, ConditionArgType>> {
        self.inner.subscribe_to_list_conditionnal(keys, reception_delay)
    }    
}


