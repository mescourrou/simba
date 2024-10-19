/*!
Provide the [`Network`] struct, allowing communication between robots, and
the configuration struct [`NetworkConfig`].
*/

extern crate confy;
use std::collections::BTreeMap;
use std::fmt;
use std::sync::{mpsc, Arc, Condvar, Mutex, RwLock};

use log::debug;
use serde_derive::{Deserialize, Serialize};
use serde_json::Value;

use crate::simulator::SimulatorMetaConfig;
use crate::turtlebot::Turtlebot;
use crate::utils::determinist_random_variable::{
    DeterministRandomVariable, DeterministRandomVariableFactory,
};
use crate::utils::time_ordered_data::TimeOrderedData;

use super::message_handler::MessageHandler;
use super::network_manager::NetworkManager;

/// Configuration for the [`Network`].
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct NetworkConfig {
    /// Limit range communication, 0 or `f32::INFINITY` for no limit.
    pub range: f32,
    /// Communication delay (fixed). 0 for no delay.
    pub delay: f32,
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            range: f32::INFINITY,
            delay: 0.,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Default, PartialEq, Debug)]
pub enum MessageMode {
    #[default]
    Default,
    God,
}

/// Network interface for [`Turtlebot`].
///
/// Each [`Turtlebot`] should have a [`Network`] instance. Through this interface,
/// the robots can send messages to other robots using pair-to-pair communication,
/// or broadcast diffusion.
#[derive(Clone)]
pub struct Network {
    /// Name of the robot (will be in the 'from' field of the sent messages).
    from: String,
    /// Range limitation (not implemented yet).
    range: f32,
    /// Added delay to the messages.
    delay: f32,
    /// Reference to the simulator [`NetworkManager`].
    network_manager: Option<Arc<RwLock<NetworkManager>>>,
    /// List of the other robots associated to their asynchronous Sender.
    ///
    /// Map[Name of the robot, Sender].
    other_emitters: BTreeMap<String, Arc<Mutex<mpsc::Sender<(String, Value, f32, MessageMode)>>>>,
    /// List of handler. First handler returning Ok has priority.
    message_handlers: Vec<Arc<RwLock<dyn MessageHandler>>>,
    /// Asynchronous receiver for the current [`Network`].
    channel_receiver: Arc<Mutex<mpsc::Receiver<(String, Value, f32, MessageMode)>>>,
    /// Asynchronous sender for the current [`Network`].
    channel_emitter: Arc<Mutex<mpsc::Sender<(String, Value, f32, MessageMode)>>>,
    /// Message list
    messages_buffer: TimeOrderedData<(String, Value)>,

    time_cv: Arc<(Mutex<usize>, Condvar)>,
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
    /// Create a new default Network.
    pub fn new(from: String, time_cv: Arc<(Mutex<usize>, Condvar)>) -> Network {
        Network::from_config(
            from,
            &NetworkConfig::default(),
            SimulatorMetaConfig::default(),
            &DeterministRandomVariableFactory::default(),
            time_cv,
        )
    }

    /// Makes a new Network from the given config.
    pub fn from_config(
        from: String,
        config: &NetworkConfig,
        _meta_config: SimulatorMetaConfig,
        _va_factory: &DeterministRandomVariableFactory,
        time_cv: Arc<(Mutex<usize>, Condvar)>,
    ) -> Network {
        let (tx, rx) = mpsc::channel::<(String, Value, f32, MessageMode)>();
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
            time_cv,
        }
    }

    /// Set the `network_manager` reference.
    pub fn set_network_manager(&mut self, network_manager: Arc<RwLock<NetworkManager>>) {
        self.network_manager = Some(network_manager);
    }

    /// Get the Sender, to be able to send message to this [`Network`].
    pub fn get_emitter(&mut self) -> mpsc::Sender<(String, Value, f32, MessageMode)> {
        self.channel_emitter.lock().unwrap().clone()
    }

    /// Add a new `emitter`, to send messages to the robot `turtle_name`.
    pub fn add_emitter(
        &mut self,
        turtle_name: String,
        emitter: mpsc::Sender<(String, Value, f32, MessageMode)>,
    ) {
        self.other_emitters
            .insert(turtle_name, Arc::new(Mutex::new(emitter)));
    }

    /// Check whether the recipient is in range or not.
    fn can_send(&self, recipient: &String, time: f32, message_mode: &MessageMode) -> bool {
        if message_mode == &MessageMode::God {
            return true;
        }
        if self.range <= 0. {
            return true;
        }
        let distance = self
            .network_manager
            .as_ref()
            .unwrap()
            .read()
            .unwrap()
            .distance_between(&self.from, recipient, time);
        return distance < self.range;
    }

    /// Send a `message` to the given `recipient`. `time` is the send message time, before delays.
    /// The message is sent if in range.
    pub fn send_to(
        &mut self,
        recipient: String,
        message: Value,
        time: f32,
        message_mode: MessageMode,
    ) {
        if !self.can_send(&recipient, time, &message_mode) {
            return;
        }
        let emitter_option = self.other_emitters.get(&recipient);
        if let Some(emitter) = emitter_option {
            let _lk = self.time_cv.0.lock().unwrap();
            let _ = emitter.lock().unwrap().send((
                self.from.clone(),
                message,
                time,
                message_mode.clone(),
            ));
            self.time_cv.1.notify_all();
        }
    }

    /// Send a `message` to all available robots. `time` is the send message time, before delays.
    pub fn broadcast(&mut self, message: Value, time: f32, message_mode: MessageMode) {
        for (recipient, emitter) in &self.other_emitters {
            if !self.can_send(&recipient, time, &message_mode) {
                continue;
            }
            let _lk = self.time_cv.0.lock().unwrap();
            let _ = emitter.lock().unwrap().send((
                self.from.clone(),
                message.clone(),
                time,
                message_mode.clone(),
            ));
            self.time_cv.1.notify_all();
        }
    }

    /// Unstack the waiting messages in the asynchronous receiver and put them in the buffer.
    /// Adds the delay.
    pub fn process_messages(&mut self) -> usize {
        let rx = self.channel_receiver.lock().unwrap();
        for (from, message, time, message_mode) in rx.try_iter() {
            let time = match message_mode {
                MessageMode::Default => time + self.delay,
                MessageMode::God => time,
            };
            debug!("[{}] Add new message from {from} at time {time}", self.from);
            self.messages_buffer.insert(time, (from, message), false);
        }
        self.messages_buffer.len()
    }

    /// Get the minimal time among all waiting messages.
    pub fn next_message_time(&self) -> Option<f32> {
        self.messages_buffer.min_time()
    }

    /// Handle the messages which are received at the given `time`.
    ///
    /// ## Arguments
    /// * `turtle` - Reference to the robot to give to the handlers.
    /// * `time` - Time of the messages to handle.
    pub fn handle_message_at_time(&mut self, turtle: &mut Turtlebot, time: f32) {
        while let Some((msg_time, (from, message))) = self.messages_buffer.remove(time) {
            debug!("[{}] Receive message from {from}: {:?}", self.from, message);
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
                    .handle_message(turtle, &from, &message, msg_time)
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

    /// Add a new handler to the [`Network`].
    pub fn subscribe(&mut self, handler: Arc<RwLock<dyn MessageHandler>>) {
        self.message_handlers.push(handler);
    }
}
