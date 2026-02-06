use std::{collections::{BTreeMap, BTreeSet, HashMap}, path::Path};

use tree_ds::prelude::*;

use crate::pub_sub::{Client, MultiClient, channel::{Channel, ChannelProcessing}};

#[derive(Debug)]
pub struct Broker<KeyType>
where KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync,
{
    channels: HashMap<KeyType, Box<dyn ChannelProcessing>>,
    key_tree: Tree<AutomatedId, KeyType>,
    key_to_node_id: HashMap<KeyType, AutomatedId>,
    time_round: f32,
}

impl<KeyType> Broker<KeyType> 
where KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync,
{
    pub fn new(time_round: f32) -> Self {
        let mut key_tree = Tree::new(None);
        let root = key_tree.add_node(Node::new_with_auto_id(Some(KeyType::default())), None);
        let mut key_to_node_id = HashMap::new();
        key_to_node_id.insert(KeyType::default(), root.unwrap());
        Self {
            channels: HashMap::new(),
            key_tree,
            key_to_node_id,
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
        self.channels.insert(key.clone(), Box::new(Channel::<MessageType, ConditionArgType>::new_conditionnal(condition, self.time_round)));
        let new_id = self.key_tree.add_node(Node::new_with_auto_id(Some(key.clone())), Some(&self.key_to_node_id.get(&KeyType::default()).unwrap())).unwrap();
        self.key_to_node_id.insert(key, new_id);
    }

    pub fn add_channel<MessageType: Clone + Send + 'static>(
        &mut self,
        key: KeyType,
    ) {
        self.channels.insert(key.clone(), Box::new(Channel::<MessageType, ()>::new(self.time_round)));
        self.key_tree.add_node(Node::new_with_auto_id(Some(key.clone())), Some(&self.key_to_node_id.get(&KeyType::default()).unwrap())).unwrap();
    }


    pub fn add_metachannel(&mut self, key: KeyType, parent_key: &KeyType) {
        let parent_node_id = self.key_to_node_id.get(parent_key).expect("Parent key does not exist");
        self.key_tree.add_node(Node::new_with_auto_id(Some(key.clone())), Some(parent_node_id)).unwrap();
    }

    pub fn add_subchannel<MessageType: Clone + Send + 'static>(
        &mut self,
        key: KeyType,
        parent_key: &KeyType,
    ) {
        self.channels.insert(key.clone(), Box::new(Channel::<MessageType, ()>::new(self.time_round)));
        let parent_node_id = self.key_to_node_id.get(parent_key).expect("Parent key does not exist");
        self.key_tree.add_node(Node::new_with_auto_id(Some(key.clone())), Some(parent_node_id)).unwrap();
    }

    pub fn add_subchannel_conditionnal<MessageType: Clone + Send + 'static, ConditionArgType: Clone + Send + 'static>(
        &mut self,
        key: KeyType,
        parent_key: &KeyType,
        condition: impl Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
    ) {
        self.channels.insert(key.clone(), Box::new(Channel::<MessageType, ConditionArgType>::new_conditionnal(condition, self.time_round)));
        let parent_node_id = self.key_to_node_id.get(parent_key).expect("Parent key does not exist");
        self.key_tree.add_node(Node::new_with_auto_id(Some(key.clone())), Some(parent_node_id)).unwrap();
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

    pub fn subscribe_to_meta<MessageType: Clone + Send + 'static>(
        &mut self,
        key: &KeyType,
        reception_delay: f32,
    ) -> Option<MultiClient<KeyType, MessageType, ()>> {
        let parent_node_id = self.key_to_node_id.get(key).expect("Key does not exist");
        let mut keys = Vec::new();
        // Iterate on leaf nodes of the subtree of parent_node_id
        for node_id in self.key_tree.get_subtree(parent_node_id, None).unwrap().traverse(parent_node_id, TraversalStrategy::PreOrder).unwrap().iter().filter(|id| self.key_tree.get_node_degree(id).unwrap() == 0) {
            keys.push(self.key_tree.get_node_by_id(node_id).unwrap().get_value().unwrap().unwrap());
        }
        self.subscribe_to_list::<MessageType>(&keys, reception_delay)
    }

    pub fn subscribe_to_meta_conditionnal<MessageType: Clone + Send + 'static, ConditionArgType: Clone + Send + 'static>(
        &mut self,
        key: &KeyType,
        reception_delay: f32,
    ) -> Option<MultiClient<KeyType, MessageType, ConditionArgType>> {
        let parent_node_id = self.key_to_node_id.get(key).expect("Key does not exist");
        let mut keys = Vec::new();
        // Iterate on leaf nodes of the subtree of parent_node_id
        for node_id in self.key_tree.get_subtree(parent_node_id, None).unwrap().traverse(parent_node_id, TraversalStrategy::PreOrder).unwrap().iter().filter(|id| self.key_tree.get_node_degree(id).unwrap() == 0) {
            keys.push(self.key_tree.get_node_by_id(node_id).unwrap().get_value().unwrap().unwrap());
        }
        self.subscribe_to_list_conditionnal::<MessageType, ConditionArgType>(&keys, reception_delay)
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


#[derive(Debug)]
pub struct GenericBroker<KeyType, MessageType, ConditionArgType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync,
    MessageType: Clone + Send + 'static,
    ConditionArgType: Clone + Send + 'static,
{
    inner: Broker<KeyType>,
    _phantom: std::marker::PhantomData<(MessageType, ConditionArgType)>,
}

impl<KeyType, MessageType, ConditionArgType> GenericBroker<KeyType, MessageType, ConditionArgType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync,
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

    pub fn add_channel(&mut self, key: KeyType, condition: impl Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone) {
        self.inner.add_channel_conditionnal::<MessageType, ConditionArgType>(key, condition);
    }

    pub fn add_metachannel(&mut self, key: KeyType, parent_key: &KeyType) {
        self.inner.add_metachannel(key, parent_key);
    }

    pub fn add_subchannel(&mut self, key: KeyType, parent_key: &KeyType, condition: impl Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone) {
        self.inner.add_subchannel_conditionnal::<MessageType, ConditionArgType>(key, parent_key, condition);
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
