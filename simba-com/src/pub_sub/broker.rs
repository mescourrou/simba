pub use std::str::FromStr;
use std::{
    collections::HashMap,
    fmt::{Debug, Display},
};

use itertools::Itertools;
use log::warn;
use tree_ds::prelude::{AutomatedId, Node, TraversalStrategy, Tree};

use crate::pub_sub::{
    Client, MultiClientTrait,
    channel::{Channel, ChannelProcessing},
};

pub trait BrokerTrait<KeyType, MessageType, NodeIdType>: Debug + Send + Sync
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync,
    MessageType: Clone + Send + 'static + Default,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static,
{
    fn add_channel(&mut self, key: KeyType);

    fn add_metachannel(&mut self, key: KeyType, parent_key: Option<&KeyType>);

    fn add_subchannel(&mut self, key: KeyType, parent_key: &KeyType);

    fn subscribe_to(
        &mut self,
        key: &KeyType,
        node_id: NodeIdType,
        reception_delay: f32,
    ) -> Option<Client<MessageType>>;

    fn subscribe_to_meta(
        &mut self,
        key: &KeyType,
        reception_delay: f32,
        multi_client: &mut dyn MultiClientTrait<KeyType, MessageType, NodeIdType>,
    ) -> Result<(), String>;

    fn remove_channel(&mut self, key: &KeyType);

    fn clear_channels(&mut self);

    fn channel_exists(&self, key: &KeyType) -> bool;

    fn meta_exists(&self, key: &KeyType) -> bool;

    fn channel_list(&self) -> Vec<KeyType>;

    fn meta_tree(&self) -> Vec<(KeyType, KeyType)>;

    /// Subscribe to multiple channels at once. If one of the channels does not exist, return None and do not subscribe to any channel.
    fn subscribe_to_list(
        &mut self,
        keys: &[KeyType],
        reception_delay: f32,
        multi_client: &mut dyn MultiClientTrait<KeyType, MessageType, NodeIdType>,
    ) -> Result<(), String>;
}

pub trait BrokerTraitExtended<KeyType, MessageType, NodeIdType, ConditionArgType>:
    BrokerTrait<KeyType, MessageType, NodeIdType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync,
    MessageType: Clone + Send + 'static + Default,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static,
    ConditionArgType: Clone + Send + 'static + Default,
{
    fn add_channel_conditionnal<F>(&mut self, key: KeyType, condition: F)
    where
        F: Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone;

    fn add_subchannel_conditionnal<F>(&mut self, key: KeyType, parent_key: &KeyType, condition: F)
    where
        F: Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone;
}

pub trait BrokerTraitProcessing<KeyType, MessageType, NodeIdType, ConditionArgType>:
    BrokerTrait<KeyType, MessageType, NodeIdType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync,
    MessageType: Clone + Send + 'static + Default,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static,
    ConditionArgType: Clone + Send + 'static + Default,
{
    fn process_messages(
        &self,
        client_condition_args: Option<&HashMap<NodeIdType, ConditionArgType>>,
    );

    fn get_channel(
        &mut self,
        key: &KeyType,
    ) -> Option<Channel<MessageType, NodeIdType, ConditionArgType>>;
}

#[derive(Debug)]
pub struct Broker<KeyType, MessageType, NodeIdType, ConditionArgType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync,
    MessageType: Clone + Send + 'static + Default + Debug,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
    ConditionArgType: Clone + Send + 'static + Default + Debug,
{
    channels: HashMap<KeyType, Box<dyn ChannelProcessing<NodeIdType, ConditionArgType>>>,
    key_tree: Tree<AutomatedId, KeyType>,
    key_to_node_id: HashMap<KeyType, AutomatedId>,
    time_round: f32,
    _phantom: std::marker::PhantomData<MessageType>,
}

impl<KeyType, MessageType, NodeIdType, ConditionArgType>
    Broker<KeyType, MessageType, NodeIdType, ConditionArgType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync,
    MessageType: Clone + Send + 'static + Default + Debug,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
    ConditionArgType: Clone + Send + 'static + Default + Debug,
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

impl<KeyType, MessageType, NodeIdType, ConditionArgType>
    BrokerTrait<KeyType, MessageType, NodeIdType>
    for Broker<KeyType, MessageType, NodeIdType, ConditionArgType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync + Debug + Display,
    MessageType: Clone + Send + 'static + Default + Debug + Sync,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
    ConditionArgType: Clone + Send + 'static + Default + Debug,
{
    fn add_channel(&mut self, key: KeyType) {
        if self.key_to_node_id.contains_key(&key) {
            return;
        }
        self.channels.insert(
            key.clone(),
            Box::new(Channel::<MessageType, NodeIdType, ConditionArgType>::new(
                self.time_round,
            )),
        );
        #[cfg(feature = "debug_mode")]
        log::debug!("Adding channel for key: {}", key);
        let new_id = self
            .key_tree
            .add_node(
                Node::new_with_auto_id(Some(key.clone())),
                Some(self.key_to_node_id.get(&KeyType::default()).unwrap()),
            )
            .unwrap();
        self.key_to_node_id.insert(key, new_id);
    }

    fn add_metachannel(&mut self, key: KeyType, parent_key: Option<&KeyType>) {
        if self.key_to_node_id.contains_key(&key) {
            return;
        }
        #[cfg(feature = "debug_mode")]
        log::debug!(
            "Adding meta channel for key: {}, parent: {:?}",
            key,
            parent_key
        );
        let parent_node_id = match parent_key {
            Some(parent_key) => self
                .key_to_node_id
                .get(parent_key)
                .expect("Parent key does not exist"),
            None => self
                .key_to_node_id
                .get(&KeyType::default())
                .expect("Default key does not exist"),
        };
        let new_node_id = self
            .key_tree
            .add_node(
                Node::new_with_auto_id(Some(key.clone())),
                Some(parent_node_id),
            )
            .unwrap();
        self.key_to_node_id.insert(key, new_node_id);
    }

    fn add_subchannel(&mut self, key: KeyType, parent_key: &KeyType) {
        if self.key_to_node_id.contains_key(&key) {
            return;
        }
        self.channels.insert(
            key.clone(),
            Box::new(Channel::<MessageType, NodeIdType, ConditionArgType>::new(
                self.time_round,
            )),
        );
        let parent_node_id = self
            .key_to_node_id
            .get(parent_key)
            .expect("Parent key does not exist");
        let new_node_id = self
            .key_tree
            .add_node(
                Node::new_with_auto_id(Some(key.clone())),
                Some(parent_node_id),
            )
            .unwrap();
        self.key_to_node_id.insert(key, new_node_id);
    }

    fn subscribe_to(
        &mut self,
        key: &KeyType,
        node_id: NodeIdType,
        reception_delay: f32,
    ) -> Option<Client<MessageType>> {
        match self
            .get_channel(key)
            .map(|mut channel| channel.client(node_id, reception_delay))
        {
            Some(client) => Some(client),
            None => {
                if self.meta_exists(key) {
                    warn!(
                        "Trying to subscribe to channel '{}' which is a meta channel. Use `subscribe_to_meta` instead.",
                        key
                    );
                } else {
                    warn!(
                        "Trying to subscribe to channel '{}' that does not exist",
                        key
                    );
                }
                #[cfg(feature = "debug_mode")]
                log::debug!(
                    "Available channels:\n{}",
                    self.channels.keys().map(|k| format!("- {}", k)).join("\n")
                );
                None
            }
        }
    }

    fn subscribe_to_meta(
        &mut self,
        key: &KeyType,
        reception_delay: f32,
        multiclient: &mut dyn MultiClientTrait<KeyType, MessageType, NodeIdType>,
    ) -> Result<(), String> {
        let parent_node_id = self.key_to_node_id.get(key).expect("Key does not exist");
        let mut keys = Vec::new();
        // Iterate on leaf nodes of the subtree of parent_node_id
        for node_id in self
            .key_tree
            .get_subtree(parent_node_id, None)
            .unwrap()
            .traverse(parent_node_id, TraversalStrategy::PreOrder)
            .unwrap()
            .iter()
            .filter(|id| self.key_tree.get_node_degree(id).unwrap() == 0)
        {
            keys.push(
                self.key_tree
                    .get_node_by_id(node_id)
                    .unwrap()
                    .get_value()
                    .unwrap()
                    .unwrap(),
            );
        }
        self.subscribe_to_list(&keys, reception_delay, multiclient)
    }

    fn remove_channel(&mut self, key: &KeyType) {
        self.channels.remove(key);
    }

    fn clear_channels(&mut self) {
        self.channels.clear();
        self.key_to_node_id.clear();
        self.key_tree = Tree::new(None);
        let root = self
            .key_tree
            .add_node(Node::new_with_auto_id(Some(KeyType::default())), None);
        self.key_to_node_id
            .insert(KeyType::default(), root.unwrap());
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
        self.key_tree
            .traverse(
                &self.key_to_node_id[&KeyType::default()],
                TraversalStrategy::PreOrder,
            )
            .unwrap()
            .iter()
            .map(|id| {
                let node = self.key_tree.get_node_by_id(id).unwrap();
                let key = node.get_value().unwrap().unwrap();
                let parent_key = if let Ok(Some(parent_id)) = node.get_parent_id() {
                    self.key_tree
                        .get_node_by_id(&parent_id)
                        .unwrap()
                        .get_value()
                        .unwrap()
                        .unwrap()
                } else {
                    KeyType::default()
                };
                (key, parent_key)
            })
            .unique()
            .collect()
    }

    /// Subscribe to multiple channels at once. If one of the channels does not exist, return None and do not subscribe to any channel.
    /// The channels must have the same MessageType.
    fn subscribe_to_list(
        &mut self,
        keys: &[KeyType],
        reception_delay: f32,
        multi_client: &mut dyn MultiClientTrait<KeyType, MessageType, NodeIdType>,
    ) -> Result<(), String> {
        for key in keys {
            if let Some(client) =
                self.subscribe_to(key, multi_client.node_id().clone(), reception_delay)
            {
                multi_client.add_client(key, client);
            } else {
                return Err(format!("Channel {:?} does not exist", key));
            }
        }
        Ok(())
    }
}

impl<KeyType, MessageType, NodeIdType, ConditionArgType>
    BrokerTraitExtended<KeyType, MessageType, NodeIdType, ConditionArgType>
    for Broker<KeyType, MessageType, NodeIdType, ConditionArgType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync + Debug + Display,
    MessageType: Clone + Send + 'static + Default + Debug + Sync,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
    ConditionArgType: Clone + Send + 'static + Default + Debug,
{
    fn add_channel_conditionnal<F>(&mut self, key: KeyType, condition: F)
    where
        F: Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
    {
        if self.key_to_node_id.contains_key(&key) {
            return;
        }
        self.channels.insert(
            key.clone(),
            Box::new(
                Channel::<MessageType, NodeIdType, ConditionArgType>::new_conditionnal(
                    condition,
                    self.time_round,
                ),
            ),
        );
        let new_id = self
            .key_tree
            .add_node(
                Node::new_with_auto_id(Some(key.clone())),
                Some(self.key_to_node_id.get(&KeyType::default()).unwrap()),
            )
            .unwrap();
        self.key_to_node_id.insert(key, new_id);
    }

    fn add_subchannel_conditionnal<F>(&mut self, key: KeyType, parent_key: &KeyType, condition: F)
    where
        F: Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
    {
        if self.key_to_node_id.contains_key(&key) {
            return;
        }
        self.channels.insert(
            key.clone(),
            Box::new(
                Channel::<MessageType, NodeIdType, ConditionArgType>::new_conditionnal(
                    condition,
                    self.time_round,
                ),
            ),
        );
        let parent_node_id = self
            .key_to_node_id
            .get(parent_key)
            .expect("Parent key does not exist");
        let new_node_id = self
            .key_tree
            .add_node(
                Node::new_with_auto_id(Some(key.clone())),
                Some(parent_node_id),
            )
            .unwrap();
        self.key_to_node_id.insert(key, new_node_id);
    }
}

impl<KeyType, MessageType, NodeIdType, ConditionArgType>
    BrokerTraitProcessing<KeyType, MessageType, NodeIdType, ConditionArgType>
    for Broker<KeyType, MessageType, NodeIdType, ConditionArgType>
where
    KeyType: std::cmp::Eq + std::hash::Hash + Clone + Default + Send + Sync + Debug + Display,
    MessageType: Clone + Send + 'static + Default + Debug + Sync,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
    ConditionArgType: Clone + Send + 'static + Default + Debug,
{
    fn process_messages(
        &self,
        client_condition_args: Option<&HashMap<NodeIdType, ConditionArgType>>,
    ) {
        #[cfg(feature = "debug_mode")]
        log::debug!(
            "Processing messages for broker with {} channels",
            self.channels.len()
        );
        for channel in self.channels.values() {
            channel.process_messages(client_condition_args);
        }
    }

    fn get_channel(
        &mut self,
        key: &KeyType,
    ) -> Option<Channel<MessageType, NodeIdType, ConditionArgType>> {
        self.channels
            .get_mut(key)?
            .as_any_mut()
            .downcast_mut::<Channel<MessageType, NodeIdType, ConditionArgType>>()
            .cloned()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct PathKey {
    path: Vec<String>,
    absolute: bool,
}

impl Default for PathKey {
    fn default() -> Self {
        Self {
            absolute: true,
            path: Vec::new(),
        }
    }
}

impl PathKey {
    pub fn new(path: Vec<String>, absolute: bool) -> Self {
        Self { path, absolute }
    }

    fn str_split(path: &str) -> Vec<String> {
        path.split('/')
            .filter_map(|s| {
                if s.is_empty() {
                    None
                } else {
                    Some(s.to_string())
                }
            })
            .collect()
    }

    pub fn parent(&self) -> Option<Self> {
        if self.path.len() <= 1 {
            None
        } else {
            Some(Self {
                path: self.path[..self.path.len() - 1].to_vec(),
                absolute: self.absolute,
            })
        }
    }

    pub fn absolute(&self) -> bool {
        self.absolute
    }

    pub fn to_vec(&self) -> Vec<String> {
        self.path.clone()
    }

    pub fn len(&self) -> usize {
        self.path.len()
    }

    pub fn is_empty(&self) -> bool {
        self.path.is_empty()
    }

    pub fn join(&self, other: &PathKey) -> Self {
        let mut new_path = self.path.clone();
        new_path.extend(other.path.clone());
        Self {
            path: new_path,
            absolute: self.absolute,
        }
    }

    pub fn join_str(&self, other: &str) -> Self {
        let mut new_path = self.path.clone();
        new_path.extend(Self::str_split(other));
        Self {
            path: new_path,
            absolute: self.absolute,
        }
    }

    pub fn prepend(&self, other: &PathKey) -> Self {
        let mut new_path = other.path.clone();
        new_path.extend(self.path.clone());
        Self {
            path: new_path,
            absolute: other.absolute,
        }
    }

    pub fn prepend_str(&self, other: &str) -> Self {
        let mut new_path = Self::str_split(other);
        new_path.extend(self.path.clone());
        Self {
            path: new_path,
            absolute: other.starts_with('/'),
        }
    }
}

impl Display for PathKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let str = if self.absolute {
            format!("/{}", self.path.join("/"))
        } else {
            self.path.join("/")
        };
        write!(f, "{}", str)
    }
}

impl FromStr for PathKey {
    type Err = std::convert::Infallible;
    fn from_str(path: &str) -> Result<Self, Self::Err> {
        Ok(Self {
            path: Self::str_split(path),
            absolute: path.starts_with('/'),
        })
    }
}

#[derive(Debug)]
pub struct PathBroker<MessageType, NodeIdType, ConditionArgType>
where
    MessageType: Clone + Send + 'static + Default + Debug,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
    ConditionArgType: Clone + Send + 'static + Default + Debug,
{
    broker: Broker<String, MessageType, NodeIdType, ConditionArgType>,
}

impl<MessageType, NodeIdType, ConditionArgType>
    PathBroker<MessageType, NodeIdType, ConditionArgType>
where
    MessageType: Clone + Send + 'static + Default + Debug,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
    ConditionArgType: Clone + Send + 'static + Default + Debug,
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

impl<MessageType, NodeIdType, ConditionArgType> BrokerTrait<PathKey, MessageType, NodeIdType>
    for PathBroker<MessageType, NodeIdType, ConditionArgType>
where
    MessageType: Clone + Send + 'static + Default + Debug + Sync,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
    ConditionArgType: Clone + Send + 'static + Default + Debug,
{
    fn add_channel(&mut self, key: PathKey) {
        if let Some(parent) = key.parent() {
            self.add_metachannel(parent, None);
        }
        self.broker.add_subchannel(
            key.to_string(),
            &key.parent().map(|p| p.to_string()).unwrap_or_default(),
        );
    }

    fn add_metachannel(&mut self, key: PathKey, parent_key: Option<&PathKey>) {
        if key.parent() != parent_key.cloned() {
            if let Some(local_parent) = key.parent() {
                self.add_metachannel(local_parent, parent_key);
            } else {
                panic!("The new key should be a descendant of the parent key");
            }
        }

        self.broker.add_metachannel(
            key.to_string(),
            key.parent().map(|k| k.to_string()).as_ref(),
            // parent_key.map(|k| k.to_string()).as_ref(),
        );
    }

    fn add_subchannel(&mut self, key: PathKey, parent_key: &PathKey) {
        assert!(
            parent_key.absolute(),
            "Only absolute keys can be used as parent keys"
        );
        self.add_metachannel(parent_key.clone(), None);
        self.broker
            .add_subchannel(key.to_string(), &parent_key.to_string());
    }

    fn channel_exists(&self, key: &PathKey) -> bool {
        assert!(
            key.absolute(),
            "Only absolute keys can be used to check channel existence"
        );
        self.broker.channel_exists(&key.to_string())
    }

    fn meta_exists(&self, key: &PathKey) -> bool {
        assert!(
            key.absolute(),
            "Only absolute keys can be used to check meta channel existence"
        );
        self.broker.meta_exists(&key.to_string())
    }

    fn channel_list(&self) -> Vec<PathKey> {
        self.broker
            .channel_list()
            .iter()
            .map(|s| PathKey::from_str(s).unwrap())
            .collect()
    }

    fn meta_tree(&self) -> Vec<(PathKey, PathKey)> {
        self.broker
            .key_tree
            .traverse(
                &self.broker.key_to_node_id[&String::default()],
                TraversalStrategy::PreOrder,
            )
            .unwrap()
            .iter()
            .filter_map(|id| {
                let node = self.broker.key_tree.get_node_by_id(id).unwrap();
                let key = PathKey::from_str(&node.get_value().unwrap().unwrap()).unwrap();
                if key.len() == 0 {
                    return None;
                }
                let parent_key = if let Ok(Some(parent_id)) = node.get_parent_id() {
                    let pkey = PathKey::from_str(
                        &self
                            .broker
                            .key_tree
                            .get_node_by_id(&parent_id)
                            .unwrap()
                            .get_value()
                            .unwrap()
                            .unwrap(),
                    )
                    .unwrap();
                    if pkey.len() > 0 {
                        pkey
                    } else {
                        // Force root key to be absolute
                        PathKey::default()
                    }
                } else {
                    PathKey::default()
                };
                Some((key, parent_key))
            })
            .unique()
            .collect()
    }

    fn remove_channel(&mut self, key: &PathKey) {
        assert!(
            key.absolute(),
            "Only absolute keys can be used to remove a channel"
        );
        self.broker.remove_channel(&key.to_string());
    }

    fn clear_channels(&mut self) {
        self.broker.clear_channels();
    }

    fn subscribe_to(
        &mut self,
        key: &PathKey,
        node_id: NodeIdType,
        reception_delay: f32,
    ) -> Option<Client<MessageType>> {
        self.broker
            .subscribe_to(&key.to_string(), node_id, reception_delay)
    }

    fn subscribe_to_list(
        &mut self,
        keys: &[PathKey],
        reception_delay: f32,
        multi_client: &mut dyn MultiClientTrait<PathKey, MessageType, NodeIdType>,
    ) -> Result<(), String> {
        for key in keys {
            let key = multi_client.transform_key(key);
            if self.channel_exists(&key)
                && let Some(client) =
                    self.subscribe_to(&key, multi_client.node_id().clone(), reception_delay)
            {
                multi_client.add_client(&key, client);
            } else if self.meta_exists(&key) {
                self.subscribe_to_meta(&key, reception_delay, multi_client)?;
            } else {
                return Err(format!("Failed to subscribe to key: {}", key));
            }
        }
        Ok(())
    }

    fn subscribe_to_meta(
        &mut self,
        key: &PathKey,
        reception_delay: f32,
        multi_client: &mut dyn MultiClientTrait<PathKey, MessageType, NodeIdType>,
    ) -> Result<(), String> {
        let key = multi_client.transform_key(key);
        let parent_node_id = self
            .broker
            .key_to_node_id
            .get(&key.to_string())
            .expect("Key does not exist");
        let mut keys = Vec::new();
        // Iterate on leaf nodes of the subtree of parent_node_id
        for node_id in self
            .broker
            .key_tree
            .traverse(parent_node_id, TraversalStrategy::PreOrder)
            .unwrap()
            .iter()
            .filter(|id| self.broker.key_tree.get_node_degree(id).unwrap() == 0)
        {
            let key = PathKey::from_str(
                &self
                    .broker
                    .key_tree
                    .get_node_by_id(node_id)
                    .unwrap()
                    .get_value()
                    .unwrap()
                    .unwrap(),
            )
            .unwrap();
            keys.push(key);
        }
        for key in keys {
            if let Some(client) =
                self.subscribe_to(&key, multi_client.node_id().clone(), reception_delay)
            {
                multi_client.add_client(&key, client);
            } else {
                return Err(format!(
                    "During meta subscription, failed to subscribe to key: {}",
                    key
                ));
            }
        }
        Ok(())
    }
}

impl<MessageType, NodeIdType, ConditionArgType>
    BrokerTraitExtended<PathKey, MessageType, NodeIdType, ConditionArgType>
    for PathBroker<MessageType, NodeIdType, ConditionArgType>
where
    MessageType: Clone + Send + 'static + Default + Debug + Sync,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
    ConditionArgType: Clone + Send + 'static + Default + Debug,
{
    fn add_channel_conditionnal<F>(&mut self, key: PathKey, condition: F)
    where
        F: Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
    {
        if let Some(parent) = key.parent() {
            self.add_metachannel(parent, None);
        }
        self.broker.add_subchannel_conditionnal(
            key.to_string(),
            &key.parent().map(|p| p.to_string()).unwrap_or_default(),
            condition,
        );
    }

    fn add_subchannel_conditionnal<F>(&mut self, key: PathKey, parent_key: &PathKey, condition: F)
    where
        F: Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
    {
        self.add_metachannel(parent_key.clone(), None);
        self.broker.add_subchannel_conditionnal(
            key.to_string(),
            &parent_key.to_string(),
            condition,
        );
    }
}

impl<MessageType, NodeIdType, ConditionArgType>
    BrokerTraitProcessing<PathKey, MessageType, NodeIdType, ConditionArgType>
    for PathBroker<MessageType, NodeIdType, ConditionArgType>
where
    MessageType: Clone + Send + 'static + Default + Debug + Sync,
    NodeIdType: std::hash::Hash + Eq + Clone + Send + Sync + 'static + Debug,
    ConditionArgType: Clone + Send + 'static + Default + Debug,
{
    fn get_channel(
        &mut self,
        key: &PathKey,
    ) -> Option<Channel<MessageType, NodeIdType, ConditionArgType>> {
        self.broker.get_channel(&key.to_string())
    }

    fn process_messages(
        &self,
        client_condition_args: Option<&HashMap<NodeIdType, ConditionArgType>>,
    ) {
        self.broker.process_messages(client_condition_args);
    }
}

#[cfg(test)]
mod tests {
    use std::str::FromStr;

    use crate::pub_sub::PathKey;

    #[test]
    fn path_key_from_str() {
        let key = PathKey::from_str("hello/world/test").unwrap();
        assert_eq!(key.to_string(), "hello/world/test");
        assert_eq!(key.parent().unwrap().to_string(), "hello/world");
        assert_eq!(key.parent().unwrap().parent().unwrap().to_string(), "hello");
        assert_eq!(key.parent().unwrap().parent().unwrap().parent(), None);
        assert!(!key.absolute());

        let key = PathKey::from_str("/hello/world/test").unwrap();
        assert!(key.absolute());
        assert_eq!(
            key.to_vec(),
            vec!["hello".to_string(), "world".to_string(), "test".to_string()]
        );
        assert_eq!(key.to_string(), "/hello/world/test");

        let key = PathKey::default();
        assert!(key.absolute());
        assert_eq!(key.to_string(), "/");
    }

    #[test]
    fn path_key_from_vec() {
        let key = PathKey::new(
            vec!["hello".to_string(), "world".to_string(), "test".to_string()],
            false,
        );
        assert_eq!(key.to_string(), "hello/world/test");
        assert_eq!(key.parent().unwrap().to_string(), "hello/world");
        assert_eq!(key.parent().unwrap().parent().unwrap().to_string(), "hello");
        assert_eq!(key.parent().unwrap().parent().unwrap().parent(), None);

        let key = PathKey::new(
            vec!["hello".to_string(), "world".to_string(), "test".to_string()],
            true,
        );
        assert_eq!(key.to_string(), "/hello/world/test");
    }
}
