use std::{
    fmt::Debug, sync::{
        Arc, Mutex,
        mpsc::{self, Receiver, Sender},
    }
};

use crate::pub_sub::client::Client;

pub trait ChannelProcessing: Send + Sync + Debug {
    fn process_messages(&self);
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;
}


///
/// Lock order: receivers then senders
#[derive(Clone)]
pub struct Channel<MessageType: Clone + Send + 'static, ConditionArgType: Clone + Send + 'static = u8> {
    senders: Arc<Mutex<Vec<Sender<(MessageType, ConditionArgType, f32)>>>>,
    receivers: Arc<Mutex<Vec<Receiver<(MessageType, ConditionArgType, f32)>>>>,
    condition: Arc<Mutex<dyn Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static>>,
    time_round: f32,
}

impl<MessageType: Clone + Send + 'static, ConditionArgType: Clone + Send + 'static> Debug for Channel<MessageType, ConditionArgType> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Channel")
            .field("senders_count", &self.senders.lock().unwrap().len())
            .field("receivers_count", &self.receivers.lock().unwrap().len())
            .finish()
    }
}

impl<MessageType: Clone + Send + 'static, ConditionArgType: Clone + Send + 'static> Channel<MessageType, ConditionArgType> {
    pub fn new(time_round: f32) -> Self {
        Self {
            senders: Arc::new(Mutex::new(Vec::new())),
            receivers: Arc::new(Mutex::new(Vec::new())),
            condition: Arc::new(Mutex::new(|_, _| true)),
            time_round,
        }
    }

    pub fn new_conditionnal(
        condition: impl Fn(ConditionArgType, ConditionArgType) -> bool + Send + 'static + Clone,
        time_round: f32,
    ) -> Self {
        Self {
            senders: Arc::new(Mutex::new(Vec::new())),
            receivers: Arc::new(Mutex::new(Vec::new())),
            condition: Arc::new(Mutex::new(condition)),
            time_round,
        }
    }

    pub fn client(&mut self, reception_delay: f32) -> Client<MessageType, ConditionArgType> {
        let (to_client_tx, to_client_rx) = mpsc::channel();
        let (from_client_tx, from_client_rx) = mpsc::channel();
        self.receivers.lock().unwrap().push(from_client_rx);
        self.senders.lock().unwrap().push(to_client_tx);
        Client::new(
            from_client_tx,
            to_client_rx,
            self.condition.clone(),
            reception_delay,
            self.time_round,
        )
    }
}

impl<MessageType: Clone + Send + 'static, ConditionArgType: Clone + Send + 'static> ChannelProcessing
    for Channel<MessageType, ConditionArgType>
{
    fn process_messages(&self) {
        for receiver in self.receivers.lock().unwrap().iter() {
            while let Ok(message) = receiver.try_recv() {
                for sender in self.senders.lock().unwrap().iter() {
                    sender.send(message.clone()).unwrap();
                }
            }
        }
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }

}
