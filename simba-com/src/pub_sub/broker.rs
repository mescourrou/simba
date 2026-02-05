use std::collections::HashMap;

use crate::pub_sub::channel::{Channel, ChannelProcessing};

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


