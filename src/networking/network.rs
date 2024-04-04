extern crate confy;
use std::collections::BTreeMap;
use std::fmt;
use std::sync::{mpsc, Arc, Mutex, RwLock};

use log::debug;
use serde_derive::{Deserialize, Serialize};
use serde_json::Value;

use crate::simulator::SimulatorMetaConfig;
use crate::turtlebot::Turtlebot;
use crate::utils::time_ordered_data::TimeOrderedData;

use super::message_handler::MessageHandler;
use super::network_manager::NetworkManager;

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct NetworkConfig {
    range: f32,
    delay: f32,
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            range: f32::INFINITY,
            delay: 0.,
        }
    }
}

#[derive(Clone)]
pub struct Network {
    from: String,
    range: f32,
    delay: f32,
    network_manager: Option<Arc<RwLock<NetworkManager>>>,
    other_emitters: BTreeMap<String, Arc<Mutex<mpsc::Sender<(String, Value, f32)>>>>,
    message_handlers: Vec<Arc<RwLock<dyn MessageHandler>>>,
    channel_receiver: Arc<Mutex<mpsc::Receiver<(String, Value, f32)>>>,
    channel_emitter: Arc<Mutex<mpsc::Sender<(String, Value, f32)>>>,
    messages_buffer: TimeOrderedData<(String, Value)>,
    next_message_time: f32,
}

impl fmt::Debug for Network {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("")
            .field(&self.from)
            .field(&self.range)
            .field(&self.delay)
            .finish()
    }
}

impl Network {
    pub fn new(from: String) -> Network {
        Network::from_config(from, &NetworkConfig::default(), SimulatorMetaConfig::new())
    }

    pub fn from_config(
        from: String,
        config: &NetworkConfig,
        meta_config: SimulatorMetaConfig,
    ) -> Network {
        let (tx, rx) = mpsc::channel::<(String, Value, f32)>();
        Network {
            from,
            range: config.range,
            delay: config.delay,
            network_manager: None,
            other_emitters: BTreeMap::new(),
            message_handlers: Vec::new(),
            channel_receiver: Arc::new(Mutex::new(rx)),
            channel_emitter: Arc::new(Mutex::new(tx)),
            messages_buffer: TimeOrderedData::new(),
            next_message_time: -1.,
        }
    }

    pub fn set_network_manager(&mut self, network_manager: Arc<RwLock<NetworkManager>>) {
        self.network_manager = Some(network_manager);
    }

    pub fn get_emitter(&mut self) -> mpsc::Sender<(String, Value, f32)> {
        self.channel_emitter.lock().unwrap().clone()
    }

    pub fn add_emitter(
        &mut self,
        turtle_name: String,
        emitter: mpsc::Sender<(String, Value, f32)>,
    ) {
        self.other_emitters
            .insert(turtle_name, Arc::new(Mutex::new(emitter)));
    }

    pub fn send_to(&mut self, recipient: String, message: Value, time: f32) {
        let emitter_option = self.other_emitters.get(&recipient);
        if let Some(emitter) = emitter_option {
            let _ = emitter
                .lock()
                .unwrap()
                .send((self.from.clone(), message, time));
        }
    }

    pub fn broadcast(&mut self, message: Value, time: f32) {
        for (_, emitter) in &self.other_emitters {
            let _ = emitter
                .lock()
                .unwrap()
                .send((self.from.clone(), message.clone(), time));
        }
    }

    pub fn process_messages(&mut self) {
        let rx = self.channel_receiver.lock().unwrap();
        for (from, message, time) in rx.try_iter() {
            debug!("[{}] Add new message from {from} at time {time}", self.from);
            self.messages_buffer.insert(time, (from, message), false);
        }
    }

    pub fn next_message_time(&self) -> Option<f32> {
        self.messages_buffer.min_time()
    }

    pub fn handle_message_at_time(&mut self, turtle: &mut Turtlebot, time: f32) {
        let mut elements_to_remove = Vec::<usize>::new();
        let mut i: usize = 0;
        while let Some((msg_time, (from, message))) = self.messages_buffer.remove(time) {
            debug!("[{}] Receive message from {from}: {:?}", self.from, message);
            // TODO: add delay and range
            debug!(
                "[{}] Handler list size: {}",
                self.from,
                self.message_handlers.len()
            );
            for handler in &self.message_handlers {
                debug!(
                    "[{}] Handler available: {:?}",
                    self.from,
                    handler.try_write().is_ok()
                );
                if handler
                    .write()
                    .unwrap()
                    .handle_message(turtle, &from, &message, time)
                    .is_ok()
                {
                    debug!("[{}] Found handler", self.from);
                    break;
                }
            }
        }

        debug!(
            "[{}] New next_message_time: {}",
            self.from,
            self.messages_buffer.min_time().unwrap_or(-1.)
        );

        debug!(
            "[{}] Messages remaining: {}",
            self.from,
            self.messages_buffer.len()
        );
    }

    pub fn subscribe(&mut self, handler: Arc<RwLock<dyn MessageHandler>>) {
        self.message_handlers.push(handler);
    }
}
