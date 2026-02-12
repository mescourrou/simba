use std::{
    fmt::Debug,
    sync::{
        Arc, Mutex,
        mpsc::{Receiver, Sender},
    },
};

use crate::time_ordered_data::TimeOrderedData;

pub struct Client<MessageType: Clone + Default> {
    sender: Sender<(MessageType, f32)>,
    receiver: Arc<Mutex<Receiver<(MessageType, f32)>>>,
    reception_delay: f32,
    message_buffer: Mutex<TimeOrderedData<MessageType>>,
    time_round: f32,
}

impl<MessageType: Clone + Default> Client<MessageType> {
    pub(crate) fn new(
        sender: Sender<(MessageType, f32)>,
        receiver: Receiver<(MessageType, f32)>,
        reception_delay: f32,
        time_round: f32,
    ) -> Self {
        Self {
            sender,
            receiver: Arc::new(Mutex::new(receiver)),
            reception_delay,
            message_buffer: Mutex::new(TimeOrderedData::new(time_round)),
            time_round,
        }
    }

    pub fn send(&self, message: MessageType, time: f32) {
        if let Err(e) = self.sender.send((message, time)) {
            panic!("Failed to send message: {:?}", e.to_string());
        }
    }

    fn refresh_buffer(&self) {
        let mut message_buffer = self.message_buffer.lock().unwrap();
        while let Ok((message, msg_time)) = self.receiver.lock().unwrap().try_recv() {
            if msg_time < 0. {
                return;
            }
            message_buffer.insert(msg_time + self.reception_delay, message, false);
        }
    }

    pub fn try_receive(&self, time: f32) -> Option<MessageType> {
        self.refresh_buffer();
        let mut message_buffer = self.message_buffer.lock().unwrap();
        let min_time_buffer = message_buffer.min_time().map(|(t, _)| t);
        // Check first if there is a message in the buffer that can be received before trying to receive new messages, to avoid receiving messages that should be received later
        if let Some(min_time) = min_time_buffer
            && min_time - time <= self.time_round
        {
            let message = message_buffer.remove(min_time).unwrap().1;
            return Some(message);
        }
        None
    }

    pub fn receive(&self, time: f32) -> MessageType {
        let mut message_buffer = self.message_buffer.lock().unwrap();
        let min_time_buffer = message_buffer.min_time().map(|(t, _)| t);
        // Check first if there is a message in the buffer that can be received before trying to receive new messages, to avoid receiving messages that should be received later
        if let Some(min_time) = min_time_buffer
            && min_time - time <= self.time_round
        {
            let message = message_buffer.remove(min_time).unwrap().1;
            return message;
        }
        loop {
            if let Ok((message, msg_time)) = self.receiver.lock().unwrap().recv() {
                if msg_time < 0. {
                    continue;
                }
                if msg_time + self.reception_delay - time <= self.time_round {
                    return message;
                } else {
                    message_buffer.insert(msg_time + self.reception_delay, message, false);
                }
            }
        }
    }

    pub fn next_message_time(&self) -> Option<f32> {
        self.refresh_buffer();
        let message_buffer = self.message_buffer.lock().unwrap();
        message_buffer.min_time().map(|(t, _)| t)
    }
}

impl<MessageType> Debug for Client<MessageType>
where
    MessageType: Clone + Default,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Client")
            .field("reception_delay", &self.reception_delay)
            .field(
                "message_buffer_length",
                &self.message_buffer.lock().unwrap().len(),
            )
            .finish()
    }
}
