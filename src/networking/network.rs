extern crate confy;
use std::sync::{Arc, RwLock, mpsc, Mutex};
use std::fmt;
use std::collections::BTreeMap;

use serde_derive::{Serialize, Deserialize};
use serde_json::Value;

use crate::simulator::SimulatorMetaConfig;
use crate::turtlebot::Turtlebot;

use super::message_handler::MessageHandler;
use super::network_manager::NetworkManager;

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct NetworkConfig {
    range: f32,
    delay: f32
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            range: f32::INFINITY,
            delay: 0.
        }
    }
}

#[derive(Clone)]
pub struct Network {
    from: String,
    range: f32,
    delay: f32,
    network_manager: Option<Arc<RwLock<NetworkManager>>>,
    other_emitters: BTreeMap<String, Arc<Mutex<mpsc::Sender<(String, Value)>>>>,
    message_handlers: Vec<Arc<RwLock<dyn MessageHandler>>>,
    channel_receiver: Arc<Mutex<mpsc::Receiver<(String, Value)>>>,
    channel_emitter: Arc<Mutex<mpsc::Sender<(String, Value)>>>,
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
        Network::from_config(from,&NetworkConfig::default(), SimulatorMetaConfig::new())
    }

    pub fn from_config(from: String, config: &NetworkConfig, meta_config: SimulatorMetaConfig) -> Network {
        let (tx, rx) = mpsc::channel::<(String, Value)>();
        Network {
            from,
            range: config.range,
            delay: config.delay,
            network_manager: None,
            other_emitters: BTreeMap::new(),
            message_handlers: Vec::new(),
            channel_receiver: Arc::new(Mutex::new(rx)),
            channel_emitter: Arc::new(Mutex::new(tx))
        }
    }

    pub fn set_network_manager(&mut self, network_manager: Arc<RwLock<NetworkManager>>) {
        self.network_manager = Some(network_manager);
    }

    pub fn get_emitter(&mut self) -> mpsc::Sender<(String, Value)> {
        self.channel_emitter.lock().unwrap().clone()
    }

    pub fn add_emitter(&mut self, turtle_name: String, emitter: mpsc::Sender<(String, Value)>) {
        self.other_emitters.insert(turtle_name, Arc::new(Mutex::new(emitter)));
    }

    pub fn send_to(&mut self, recipient: String, message: Value) {
        let emitter_option = self.other_emitters.get(&recipient);
        if let Some(emitter) = emitter_option {
            let _ = emitter.lock().unwrap().send((self.from.clone(), message));
        }
    }

    pub fn broadcast(&mut self, message: Value) {
        for (_, emitter) in &self.other_emitters {
            let _ = emitter.lock().unwrap().send((self.from.clone(), message.clone()));
        }
    }

    pub fn handle_messages(&mut self, turtle: &mut Turtlebot) {
        println!("Handle messages by {}", self.from);
        let rx = self.channel_receiver.lock().unwrap();
        for (from, message) in rx.try_iter() {
            println!("Receive message from {from}: {:?}", message);
            // TODO: add delay and range
            println!("Handler list size: {}", self.message_handlers.len());
            for handler in &self.message_handlers {
                println!("Handler available: {:?}", handler.try_write().is_ok());
                if handler.write().unwrap().handle_message(turtle, &from, &message).is_ok() {
                    println!("Found handler");
                    break;
                }
            }
            println!("End of handler list");
        }
        println!("End of message handling by {}", self.from);
    }

    pub fn subscribe(&mut self, handler: Arc<RwLock<dyn MessageHandler>>) {
        self.message_handlers.push(handler);
    }
}