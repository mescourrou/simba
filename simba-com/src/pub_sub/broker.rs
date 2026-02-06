use std::{collections::{BTreeMap, BTreeSet, HashMap}, fmt::{Debug, Display}, path::Path};

use tree_ds::prelude::*;

use crate::pub_sub::{Client, MultiClient, channel::{Channel, ChannelProcessing}};



pub trait BrokerTrait<KeyType, MessageType, ConditionArgType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync,
    MessageType: Clone + Send + 'static,
    ConditionArgType: Clone + Send + 'static,
{
    fn process_messages(&self);

    fn add_channel_conditionnal(
        &mut self,
        key: KeyType,
        condition: impl Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
    );

    fn add_channel(
        &mut self,
        key: KeyType,
    ) {
        self.add_channel_conditionnal(key, |_, _| true);
    }

    fn add_metachannel(&mut self, key: KeyType, parent_key: Option<&KeyType>);

    fn add_subchannel(
        &mut self,
        key: KeyType,
        parent_key: &KeyType,
    ) {
        self.add_subchannel_conditionnal(key, parent_key, |_, _| true);
    }

    fn add_subchannel_conditionnal(
        &mut self,
        key: KeyType,
        parent_key: &KeyType,
        condition: impl Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
    );

    fn get_channel(
        &mut self,
        key: &KeyType,
    ) -> Option<Channel<MessageType, ConditionArgType>>;

    fn subscribe_to(
        &mut self,
        key: &KeyType,
        reception_delay: f32,
    ) -> Option<Client<MessageType, ConditionArgType>>;

    fn subscribe_to_meta(
        &mut self,
        key: &KeyType,
        reception_delay: f32,
    ) -> Option<MultiClient<KeyType, MessageType, ConditionArgType>>;

    fn remove_channel(&mut self, key: &KeyType);

    fn channel_exists(&self, key: &KeyType) -> bool;

    fn meta_exists(&self, key: &KeyType) -> bool;

    fn channel_list(&self) -> Vec<KeyType>;

    fn meta_tree(&self) -> Vec<(KeyType, KeyType)>;

    /// Subscribe to multiple channels at once. If one of the channels does not exist, return None and do not subscribe to any channel.
    fn subscribe_to_list(
        &mut self,
        keys: &[KeyType],
        reception_delay: f32,
    ) -> Option<MultiClient<KeyType, MessageType, ConditionArgType>>;
}


#[derive(Debug)]
pub struct Broker<KeyType, MessageType, ConditionArgType>
where KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync,
        MessageType: Clone + Send + 'static,
        ConditionArgType: Clone + Send + 'static,
{
    channels: HashMap<KeyType, Box<dyn ChannelProcessing>>,
    key_tree: Tree<AutomatedId, KeyType>,
    key_to_node_id: HashMap<KeyType, AutomatedId>,
    time_round: f32,
    _phantom: std::marker::PhantomData<(MessageType, ConditionArgType)>,
}

impl<KeyType, MessageType, ConditionArgType> Broker<KeyType, MessageType, ConditionArgType> 
where KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync,
        MessageType: Clone + Send + 'static,
        ConditionArgType: Clone + Send + 'static,
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
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<KeyType, MessageType, ConditionArgType> BrokerTrait<KeyType, MessageType, ConditionArgType> 
    for Broker<KeyType, MessageType, ConditionArgType> 
    where 
        KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync + Debug,
        MessageType: Clone + Send + 'static,
        ConditionArgType: Clone + Send + 'static,
{
    fn process_messages(&self) {
        for channel in self.channels.values() {
            channel.process_messages();
        }
    }

    fn add_channel_conditionnal(
        &mut self,
        key: KeyType,
        condition: impl Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
    ) {
        self.channels.insert(key.clone(), Box::new(Channel::<MessageType, ConditionArgType>::new_conditionnal(condition, self.time_round)));
        let new_id = self.key_tree.add_node(Node::new_with_auto_id(Some(key.clone())), Some(&self.key_to_node_id.get(&KeyType::default()).unwrap())).unwrap();
        self.key_to_node_id.insert(key, new_id);
    }

    fn add_channel(
        &mut self,
        key: KeyType,
    ) {
        self.channels.insert(key.clone(), Box::new(Channel::<MessageType, ConditionArgType>::new(self.time_round)));
        let new_id = self.key_tree.add_node(Node::new_with_auto_id(Some(key.clone())), Some(&self.key_to_node_id.get(&KeyType::default()).unwrap())).unwrap();
        self.key_to_node_id.insert(key, new_id);
    }


    fn add_metachannel(&mut self, key: KeyType, parent_key: Option<&KeyType>) {
        let parent_node_id = match parent_key {
            Some(parent_key) => self.key_to_node_id.get(parent_key).expect("Parent key does not exist"),
            None => self.key_to_node_id.get(&KeyType::default()).expect("Default key does not exist"),
        };
        let new_node_id = self.key_tree.add_node(Node::new_with_auto_id(Some(key.clone())), Some(parent_node_id)).unwrap();
        self.key_to_node_id.insert(key, new_node_id);
    }

    fn add_subchannel(
        &mut self,
        key: KeyType,
        parent_key: &KeyType,
    ) {
        self.channels.insert(key.clone(), Box::new(Channel::<MessageType, ConditionArgType>::new(self.time_round)));
        let parent_node_id = self.key_to_node_id.get(parent_key).expect("Parent key does not exist");
        let new_node_id = self.key_tree.add_node(Node::new_with_auto_id(Some(key.clone())), Some(parent_node_id)).unwrap();
        self.key_to_node_id.insert(key, new_node_id);
    }

    fn add_subchannel_conditionnal(
        &mut self,
        key: KeyType,
        parent_key: &KeyType,
        condition: impl Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
    ) {
        self.channels.insert(key.clone(), Box::new(Channel::<MessageType, ConditionArgType>::new_conditionnal(condition, self.time_round)));
        let parent_node_id = self.key_to_node_id.get(parent_key).expect("Parent key does not exist");
        let new_node_id = self.key_tree.add_node(Node::new_with_auto_id(Some(key.clone())), Some(parent_node_id)).unwrap();
        self.key_to_node_id.insert(key, new_node_id);
    }


    fn get_channel(
        &mut self,
        key: &KeyType,
    ) -> Option<Channel<MessageType, ConditionArgType>> {
        self.channels.get_mut(key)?.as_any_mut().downcast_mut::<Channel<MessageType, ConditionArgType>>().cloned()
    }

    fn subscribe_to(
        &mut self,
        key: &KeyType,
        reception_delay: f32,
    ) -> Option<Client<MessageType, ConditionArgType>> {
        self.get_channel(key).map(|mut channel| channel.client(reception_delay))
    }

    fn subscribe_to_meta(
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
        self.subscribe_to_list(&keys, reception_delay)
    }

    fn remove_channel(&mut self, key: &KeyType) {
        self.channels.remove(key);
    }

    fn channel_exists(&self, key: &KeyType) -> bool {
        self.channels.contains_key(key)
    }

    fn meta_exists(&self, key: &KeyType) -> bool {
        self.key_to_node_id.contains_key(key)
    }

    fn channel_list(&self) -> Vec<KeyType> {
        self.channels.keys().cloned().collect()
    }

    fn meta_tree(&self) -> Vec<(KeyType, KeyType)> {
        self.key_tree.traverse(&self.key_to_node_id[&KeyType::default()], TraversalStrategy::PreOrder).unwrap().iter().map(|id| {
            let node = self.key_tree.get_node_by_id(id).unwrap();
            let key = node.get_value().unwrap().unwrap();
            let parent_key = if let Ok(Some(parent_id)) = node.get_parent_id() {
                self.key_tree.get_node_by_id(&parent_id).unwrap().get_value().unwrap().unwrap()
            } else {
                KeyType::default()
            };
            (key, parent_key)
        }).collect()
    }

    /// Subscribe to multiple channels at once. If one of the channels does not exist, return None and do not subscribe to any channel.
    /// The channels must have the same MessageType.
    fn subscribe_to_list(
        &mut self,
        keys: &[KeyType],
        reception_delay: f32,
    ) -> Option<MultiClient<KeyType, MessageType, ConditionArgType>> {
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

}


#[derive(Debug, Clone, PartialEq, Eq, Hash, Default)]
pub struct PathKey {
    path: Vec<String>,
}

impl PathKey {
    pub fn new(path: Vec<String>) -> Self {
        Self { path }
    }

    pub fn from_str(path: &str) -> Self {
        Self { path: path.split('/').map(|s| s.to_string()).collect() }
    }

    pub fn to_string(&self) -> String {
        self.path.join("/")
    }

    pub fn parent(&self) -> Option<Self> {
        if self.path.len() <= 1 {
            None
        } else {
            Some(Self { path: self.path[..self.path.len() - 1].to_vec() })
        }
    }
}

impl Display for PathKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_string())
    }
}


#[derive(Debug)]
pub struct PathBroker<MessageType, ConditionArgType> 
where MessageType: Clone + Send + 'static,
      ConditionArgType: Clone + Send + 'static,
{
    broker: Broker<String, MessageType, ConditionArgType>,
}

impl<MessageType, ConditionArgType> PathBroker<MessageType, ConditionArgType> 
where MessageType: Clone + Send + 'static,
      ConditionArgType: Clone + Send + 'static,
{
    pub fn new(time_round: f32) -> Self {
        Self {
            broker: Broker::new(time_round),
        }
    }

    // fn update_tree(&mut self, key: &PathKey) {
    //     let mut current_path = Vec::new();
    //     for part in &key.path {
    //         current_path.push(part.clone());
    //         let current_key = PathKey::new(current_path.clone());
    //         if !self.broker.meta_exists(&current_key.to_string()) {
    //             let parent = current_key.parent().unwrap_or_default();
    //             println!("Adding metachannel {} with parent {}", current_key.to_string(), parent.to_string());
    //             self.broker.add_metachannel(current_key.to_string(), &parent.to_string());
    //         }
    //     }
    // }
}

impl<MessageType, ConditionArgType> BrokerTrait<PathKey, MessageType, ConditionArgType> for PathBroker<MessageType, ConditionArgType>
where MessageType: Clone + Send + 'static,
      ConditionArgType: Clone + Send + 'static,
{
    fn add_channel(
        &mut self,
        key: PathKey,
    ) {
        if let Some(parent) = key.parent() {
            self.add_metachannel(parent, None);
        }
        self.broker.add_channel(key.to_string());
    }

    fn add_channel_conditionnal(
            &mut self,
            key: PathKey,
            condition: impl Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
        ) {
        if let Some(parent) = key.parent() {
            self.add_metachannel(parent, None);
        }
        self.broker.add_channel_conditionnal(key.to_string(), condition);
    }

    fn add_metachannel(&mut self, key: PathKey, parent_key: Option<&PathKey>) {
        if key.parent() != parent_key.cloned() {
            if let Some(local_parent) = key.parent() {
                self.add_metachannel(local_parent, parent_key);
            } else {
                panic!("The new key should be a descendant of the parent key");
            }
        }
        self.broker.add_metachannel(key.to_string(), key.parent().map(|k| k.to_string()).as_ref());
    }

    fn add_subchannel(
            &mut self,
            key: PathKey,
            parent_key: &PathKey,
        ) {
        self.add_metachannel(parent_key.clone(), None);
        self.broker.add_subchannel(key.to_string(), &parent_key.to_string());
    }

    fn add_subchannel_conditionnal(
            &mut self,
            key: PathKey,
            parent_key: &PathKey,
            condition: impl Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
        ) {
        self.add_metachannel(parent_key.clone(), None);
        self.broker.add_subchannel_conditionnal(key.to_string(), &parent_key.to_string(), condition);
    }

    fn channel_exists(&self, key: &PathKey) -> bool {
        self.broker.channel_exists(&key.to_string())
    }

    fn meta_exists(&self, key: &PathKey) -> bool {
        self.broker.meta_exists(&key.to_string())
    }

    fn channel_list(&self) -> Vec<PathKey> {
        self.broker.channel_list().iter().map(|s| PathKey::from_str(s)).collect()
    }

    fn meta_tree(&self) -> Vec<(PathKey, PathKey)> {
        self.broker.key_tree.traverse(&self.broker.key_to_node_id[&String::default()], TraversalStrategy::PreOrder).unwrap().iter().map(|id| {
            let node = self.broker.key_tree.get_node_by_id(id).unwrap();
            let key = PathKey::from_str(&node.get_value().unwrap().unwrap());
            let parent_key = if let Ok(Some(parent_id)) = node.get_parent_id() {
                PathKey::from_str(&self.broker.key_tree.get_node_by_id(&parent_id).unwrap().get_value().unwrap().unwrap())
            } else {
                PathKey::default()
            };
            (key, parent_key)
        }).collect()
    }

    fn get_channel(
            &mut self,
            key: &PathKey,
        ) -> Option<Channel<MessageType, ConditionArgType>> {
        self.broker.get_channel(&key.to_string())
    }

    fn process_messages(&self) {
        self.broker.process_messages();
    }

    fn remove_channel(&mut self, key: &PathKey) {
        self.broker.remove_channel(&key.to_string());
    }

    fn subscribe_to(
            &mut self,
            key: &PathKey,
            reception_delay: f32,
        ) -> Option<Client<MessageType, ConditionArgType>> {
        self.broker.subscribe_to(&key.to_string(), reception_delay)
    }

    fn subscribe_to_list(
        &mut self,
        keys: &[PathKey],
        reception_delay: f32,
    ) -> Option<MultiClient<PathKey, MessageType, ConditionArgType>> {
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

    fn subscribe_to_meta(
            &mut self,
            key: &PathKey,
            reception_delay: f32,
        ) -> Option<MultiClient<PathKey, MessageType, ConditionArgType>> {
        let parent_node_id = self.broker.key_to_node_id.get(&key.to_string()).expect("Key does not exist");
        let mut keys = Vec::new();
        // Iterate on leaf nodes of the subtree of parent_node_id
        for node_id in self.broker.key_tree.get_subtree(parent_node_id, None).unwrap().traverse(parent_node_id, TraversalStrategy::PreOrder).unwrap().iter().filter(|id| self.broker.key_tree.get_node_degree(id).unwrap() == 0) {
            keys.push(PathKey::from_str(&self.broker.key_tree.get_node_by_id(node_id).unwrap().get_value().unwrap().unwrap()));
        }
        self.subscribe_to_list(&keys, reception_delay)
    }
}

#[cfg(test)]
mod tests {
    use crate::pub_sub::PathKey;

    #[test]
    fn path_key_from_str() {
        let key = PathKey::from_str("hello/world/test");
        assert_eq!(key.to_string(), "hello/world/test");
        assert_eq!(key.parent().unwrap().to_string(), "hello/world");
        assert_eq!(key.parent().unwrap().parent().unwrap().to_string(), "hello");
        assert_eq!(key.parent().unwrap().parent().unwrap().parent(), None);
    }

    #[test]
    fn path_key_from_vec() {
        let key = PathKey::new(vec!["hello".to_string(), "world".to_string(), "test".to_string()]);
        assert_eq!(key.to_string(), "hello/world/test");
        assert_eq!(key.parent().unwrap().to_string(), "hello/world");
        assert_eq!(key.parent().unwrap().parent().unwrap().to_string(), "hello");
        assert_eq!(key.parent().unwrap().parent().unwrap().parent(), None); 
    }
}