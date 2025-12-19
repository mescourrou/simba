/*!
Provide the [`Network`] struct, allowing communication between robots, and
the configuration struct [`NetworkConfig`].
*/

extern crate confy;
use core::f32;
use std::fmt;
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{Arc, Mutex};

use log::debug;
use pyo3::pyclass;
use serde_derive::{Deserialize, Serialize};
use serde_json::Value;
use simba_macros::config_derives;

use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
use crate::logger::is_enabled;
use crate::node::Node;
use crate::simulator::{SimulatorConfig, TimeCv};
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::time_ordered_data::TimeOrderedData;
#[cfg(feature = "gui")]
use crate::{constants::TIME_ROUND, gui::UIComponent};

use super::network_manager::{MessageSendMethod, NetworkMessage};

/// Configuration for the [`Network`].
#[config_derives]
pub struct NetworkConfig {
    /// Limit range communication, 0 for no limit.
    #[check(ge(0.))]
    pub range: f32,
    /// Communication delay (fixed). 0 for no delay (not stable).
    #[check(ge(0.))]
    pub reception_delay: f32,
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            range: 0.,
            reception_delay: 0.,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for NetworkConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &SimulatorConfig,
        _current_node_name: Option<&String>,
        _unique_id: &str,
    ) {
        egui::CollapsingHeader::new("Network").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Range (0 for no limit): ");
                if self.range < 0. {
                    self.range = 0.;
                }
                ui.add(egui::DragValue::new(&mut self.range));
            });

            ui.horizontal(|ui| {
                ui.label("Reception delay: ");
                if self.reception_delay < 0. {
                    self.reception_delay = 0.;
                }
                ui.add(
                    egui::DragValue::new(&mut self.reception_delay)
                        .max_decimals((1. / TIME_ROUND) as usize),
                );
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        egui::CollapsingHeader::new("Network").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!("Range (0 for no limit): {}", self.range));
            });

            ui.horizontal(|ui| {
                ui.label(format!("Reception delay: {}", self.reception_delay));
            });
        });
    }
}

/// Transmission mode for messages.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[pyclass(get_all, set_all, eq, eq_int)]
pub enum MessageFlag {
    /// God mode, messages are instaneous.
    God,
    /// Ask to unsubscribe
    Unsubscribe,
    /// Ask to kill the receiving node
    Kill,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Envelope {
    pub from: String,
    pub message: Value,
    pub timestamp: f32,
}

/// Network interface for [`Node`].
///
/// Each [`Node`] should have a [`Network`] instance. Through this interface,
/// the nodes can send messages to other nodes using pair-to-pair communication,
/// or broadcast diffusion.
pub struct Network {
    /// Name of the robot (will be in the 'from' field of the sent messages).
    from: String,
    /// Range limitation (not implemented yet).
    range: f32,
    /// Added delay to the messages.
    reception_delay: f32,
    /// List of subscribed letter boxes.
    letter_boxes: Vec<Sender<Envelope>>,
    to_network_manager: Option<Sender<NetworkMessage>>,
    from_network_manager: Option<Arc<Mutex<Receiver<NetworkMessage>>>>,
    /// Message list
    messages_buffer: TimeOrderedData<(String, Value, Vec<MessageFlag>)>,

    time_cv: Arc<TimeCv>,
}

impl fmt::Debug for Network {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("")
            .field(&self.from)
            .field(&self.range)
            .field(&self.reception_delay)
            .finish()
    }
}

impl Network {
    /// Create a new default Network.
    pub fn new(from: String, time_cv: Arc<TimeCv>) -> Network {
        Network::from_config(
            from,
            &NetworkConfig::default(),
            &SimulatorConfig::default(),
            &DeterministRandomVariableFactory::default(),
            time_cv,
            0.0,
        )
    }

    /// Makes a new Network from the given config.
    pub fn from_config(
        from: String,
        config: &NetworkConfig,
        _global_config: &SimulatorConfig,
        _va_factory: &DeterministRandomVariableFactory,
        time_cv: Arc<TimeCv>,
        _initial_time: f32,
    ) -> Network {
        Network {
            from,
            range: config.range,
            reception_delay: config.reception_delay,
            letter_boxes: Vec::new(),
            messages_buffer: TimeOrderedData::new(),
            time_cv,
            to_network_manager: None,
            from_network_manager: None,
        }
    }

    /// Set the `network_manager` reference.
    pub fn set_network_manager_link(
        &mut self,
        to_network_manager: Sender<NetworkMessage>,
        from_network_manager: Receiver<NetworkMessage>,
    ) {
        self.to_network_manager = Some(to_network_manager);
        self.from_network_manager = Some(Arc::new(Mutex::new(from_network_manager)));
    }

    /// Send a `message` to the given `recipient`. `time` is the send message time, before delays.
    /// The message is sent if in range.
    pub fn send_to(
        &mut self,
        recipient: String,
        message: Value,
        time: f32,
        message_flags: Vec<MessageFlag>,
    ) -> SimbaResult<()> {
        if self.to_network_manager.is_none() {
            return Err(SimbaError::new(
                SimbaErrorTypes::ImplementationError,
                "Network is not properly setup: `set_network_manager_link` should be called."
                    .to_string(),
            ));
        }
        {
            let mut circulating_messages = self.time_cv.circulating_messages.lock().unwrap();
            *circulating_messages += 1;
        }
        self.to_network_manager
            .as_ref()
            .unwrap()
            .send(NetworkMessage {
                from: self.from.clone(),
                range: self.range,
                time,
                to: MessageSendMethod::Recipient(recipient),
                value: message,
                message_flags,
            })
            .unwrap();
        Ok(())
    }

    /// Send a `message` to all available robots. `time` is the send message time, before delays.
    pub fn broadcast(
        &mut self,
        message: Value,
        time: f32,
        message_flags: Vec<MessageFlag>,
    ) -> SimbaResult<()> {
        if self.to_network_manager.is_none() {
            return Err(SimbaError::new(
                SimbaErrorTypes::ImplementationError,
                "Network is not properly setup: `set_network_manager_link` should be called."
                    .to_string(),
            ));
        }
        {
            let mut circulating_messages = self.time_cv.circulating_messages.lock().unwrap();
            *circulating_messages += 1;
        }
        self.to_network_manager
            .as_ref()
            .unwrap()
            .send(NetworkMessage {
                from: self.from.clone(),
                range: self.range,
                time,
                to: MessageSendMethod::Broadcast,
                value: message,
                message_flags,
            })
            .unwrap();
        Ok(())
    }

    /// Unstack the waiting messages in the asynchronous receiver and put them in the buffer.
    /// Adds the delay.
    pub fn process_messages(&mut self) -> SimbaResult<usize> {
        if self.from_network_manager.is_none() {
            return Err(SimbaError::new(
                SimbaErrorTypes::ImplementationError,
                "Network is not properly setup: `set_network_manager_link` should be called."
                    .to_string(),
            ));
        }
        let mut new_msgs = 0;
        for msg in self
            .from_network_manager
            .as_ref()
            .unwrap()
            .lock()
            .unwrap()
            .try_iter()
        {
            {
                let mut circulating_messages = self.time_cv.circulating_messages.lock().unwrap();
                *circulating_messages -= 1;
            }
            let time = if msg.message_flags.contains(&MessageFlag::God) {
                msg.time
            } else {
                msg.time + self.reception_delay
            };
            if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                debug!("Add new message from {} at time {time}", msg.from);
            }
            self.messages_buffer
                .insert(time, (msg.from, msg.value, msg.message_flags), false);
            new_msgs += 1;
        }
        Ok(new_msgs)
    }

    /// Get the minimal time among all waiting messages. Bool is true if the message is read only.
    pub fn next_message_time(&self) -> Option<f32> {
        self.messages_buffer.min_time().map(|tpl| tpl.0)
    }

    /// Handle the messages which are received at the given `time`.
    ///
    /// ## Arguments
    /// * `robot` - Reference to the robot to give to the handlers.
    /// * `time` - Time of the messages to handle.
    pub fn handle_message_at_time(&mut self, node: &mut Node, time: f32) {
        if is_enabled(crate::logger::InternalLog::NetworkMessages) {
            debug!("Handling messages at time {time}");
        }
        while let Some((msg_time, (from, message, message_flags))) =
            self.messages_buffer.remove(time)
        {
            if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                debug!("Receive message from {from}: {:?}", message);
                debug!("Letter box list size: {}", self.letter_boxes.len());
            }
            let mut throw_message = false;
            for flag in &message_flags {
                if let MessageFlag::Kill = flag {
                    if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                        debug!("Receive message from {from} to be killed");
                    }
                    node.pre_kill();
                    throw_message = true;
                }
            }
            if throw_message {
                continue;
            }
            for letter_box in &self.letter_boxes {
                letter_box
                    .send(Envelope {
                        from: from.clone(),
                        message: message.clone(),
                        timestamp: msg_time,
                    })
                    .unwrap();
            }
        }

        if is_enabled(crate::logger::InternalLog::NetworkMessages) {
            debug!(
                "New next_message_time: {}",
                match self.messages_buffer.min_time() {
                    Some((time, _)) => time,
                    None => -1.,
                }
            );

            debug!("Messages remaining: {}", self.messages_buffer.len());
        }
    }

    /// Add a new handler to the [`Network`].
    pub fn subscribe(&mut self, letter_box: Option<Sender<Envelope>>) {
        if let Some(letter_box) = letter_box {
            self.letter_boxes.push(letter_box);
        }
    }

    pub fn unsubscribe_node(&self) -> SimbaResult<()> {
        if self.to_network_manager.is_none() {
            return Err(SimbaError::new(
                SimbaErrorTypes::ImplementationError,
                "Network is not properly setup: `set_network_manager_link` should be called."
                    .to_string(),
            ));
        }
        {
            let mut circulating_messages = self.time_cv.circulating_messages.lock().unwrap();
            *circulating_messages += 1;
        }
        self.to_network_manager
            .as_ref()
            .unwrap()
            .send(NetworkMessage {
                from: self.from.clone(),
                range: f32::INFINITY,
                time: 0.,
                to: MessageSendMethod::Manager,
                value: Value::Null,
                message_flags: vec![MessageFlag::Unsubscribe],
            })
            .unwrap();
        Ok(())
    }
}
