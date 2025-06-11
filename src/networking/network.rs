/*!
Provide the [`Network`] struct, allowing communication between robots, and
the configuration struct [`NetworkConfig`].
*/

extern crate confy;
use std::collections::BTreeMap;
use std::fmt;
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{mpsc, Arc, Condvar, Mutex, RwLock};

use config_checker::macros::Check;
use log::debug;
use serde_derive::{Deserialize, Serialize};
use serde_json::Value;

use crate::constants::TIME_ROUND;
use crate::errors::{SimbaError, SimbaErrorTypes, SimbaResult};
use crate::gui::UIComponent;
use crate::logger::is_enabled;
use crate::node::Node;
use crate::simulator::{SimulatorConfig, TimeCv};
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::time_ordered_data::TimeOrderedData;

use super::message_handler::MessageHandler;
use super::network_manager::{MessageSendMethod, NetworkManager, NetworkMessage};

/// Configuration for the [`Network`].
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
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
            reception_delay: TIME_ROUND,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for NetworkConfig {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::HashMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
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
                ui.label("Reception delay (0 not stable): ");
                if self.reception_delay < 0. {
                    self.reception_delay = 0.;
                }
                ui.add(egui::DragValue::new(&mut self.reception_delay).fixed_decimals(3));
            });
        });
    }
}

/// Transmission mode for messages.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
pub enum MessageFlag {
    /// God mode, messages are instaneous.
    God,
    /// Read only: the recipient will do a quick jump in time, and will go back to its time
    ReadOnly,
}

/// Network interface for [`Robot`].
///
/// Each [`Robot`] should have a [`Network`] instance. Through this interface,
/// the robots can send messages to other robots using pair-to-pair communication,
/// or broadcast diffusion.
pub struct Network {
    /// Name of the robot (will be in the 'from' field of the sent messages).
    from: String,
    /// Range limitation (not implemented yet).
    range: f32,
    /// Added delay to the messages.
    reception_delay: f32,
    /// List of handler. First handler returning Ok has priority.
    message_handlers: Vec<Arc<RwLock<dyn MessageHandler>>>,
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
        )
    }

    /// Makes a new Network from the given config.
    pub fn from_config(
        from: String,
        config: &NetworkConfig,
        _global_config: &SimulatorConfig,
        _va_factory: &DeterministRandomVariableFactory,
        time_cv: Arc<TimeCv>,
    ) -> Network {
        Network {
            from,
            range: config.range,
            reception_delay: config.reception_delay,
            message_handlers: Vec::new(),
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
                "Network is not properly setup: `set_network_manager_link` should be called.",
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
                "Network is not properly setup: `set_network_manager_link` should be called.",
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
                "Network is not properly setup: `set_network_manager_link` should be called.",
            ));
        }
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
        }
        Ok(self.messages_buffer.len())
    }

    /// Get the minimal time among all waiting messages. Bool is true if the message is read only.
    pub fn next_message_time(&self) -> Option<(f32, bool)> {
        let tpl_option = self.messages_buffer.min_time();
        match tpl_option {
            Some(tpl) => Some((tpl.0, tpl.1 .2.contains(&MessageFlag::ReadOnly))),
            None => None,
        }
    }

    /// Handle the messages which are received at the given `time`.
    ///
    /// ## Arguments
    /// * `robot` - Reference to the robot to give to the handlers.
    /// * `time` - Time of the messages to handle.
    pub fn handle_message_at_time(&mut self, robot: &mut Node, time: f32) {
        if is_enabled(crate::logger::InternalLog::NetworkMessages) {
            debug!("Handling messages at time {time}");
        }
        while let Some((msg_time, (from, message, _message_flags))) =
            self.messages_buffer.remove(time)
        {
            if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                debug!("Receive message from {from}: {:?}", message);
                debug!("Handler list size: {}", self.message_handlers.len());
            }
            for handler in &self.message_handlers {
                if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                    debug!("Handler available: {:?}", handler.try_write().is_ok());
                }
                if handler
                    .write()
                    .unwrap()
                    .handle_message(robot, &from, &message, msg_time)
                    .is_ok()
                {
                    if is_enabled(crate::logger::InternalLog::NetworkMessages) {
                        debug!("Found handler");
                    }
                    break;
                }
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
    pub fn subscribe(&mut self, handler: Arc<RwLock<dyn MessageHandler>>) {
        self.message_handlers.push(handler);
    }
}
