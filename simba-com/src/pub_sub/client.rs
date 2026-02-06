use std::sync::{
    Arc, Mutex,
    mpsc::{Receiver, Sender},
};

use crate::time_ordered_data::TimeOrderedData;

pub struct Client<MessageType: Clone, ConditionArgType: Clone = u8> {
    sender: Sender<(MessageType, ConditionArgType, f32)>,
    receiver: Receiver<(MessageType, ConditionArgType, f32)>,
    condition: Arc<Mutex<dyn Fn(ConditionArgType, ConditionArgType) -> bool>>,
    reception_delay: f32,
    message_buffer: Mutex<TimeOrderedData<(MessageType, ConditionArgType)>>,
    time_round: f32,
}

impl<MessageType: Clone, ConditionArgType: Clone> Client<MessageType, ConditionArgType> {
    pub(crate) fn new(
        sender: Sender<(MessageType, ConditionArgType, f32)>,
        receiver: Receiver<(MessageType, ConditionArgType, f32)>,
        condition: Arc<Mutex<dyn Fn(ConditionArgType, ConditionArgType) -> bool>>,
        reception_delay: f32,
        time_round: f32,
    ) -> Self {
        Self {
            sender,
            receiver,
            condition,
            reception_delay,
            message_buffer: Mutex::new(TimeOrderedData::new(time_round)),
            time_round,
        }
    }

    pub fn send(&self, message: MessageType, condition_arg: ConditionArgType, time: f32) {
        let _ = self.sender.send((message, condition_arg, time));
    }

    pub fn try_receive(&self, condition_arg: &ConditionArgType, time: f32) -> Option<MessageType> {
        let mut message_buffer = self.message_buffer.lock().unwrap();
        let min_time_buffer = message_buffer.min_time().map(|(t, _)| t);
        // Check first if there is a message in the buffer that can be received before trying to receive new messages, to avoid receiving messages that should be received later
        if let Some(min_time) = min_time_buffer {
            if min_time - time <= self.time_round {
                let (message, msg_condition_arg) = message_buffer.remove(min_time).unwrap().1;
                if (self.condition.lock().unwrap())(condition_arg.clone(), msg_condition_arg) {
                    return Some(message);
                }
            }
        }
        while let Ok((message, msg_condition_arg, msg_time)) = self.receiver.try_recv() {
            if msg_time + self.reception_delay - time <= self.time_round && (self.condition.lock().unwrap())(condition_arg.clone(), msg_condition_arg.clone()) {
                return Some(message);
            } else {
                message_buffer.insert(
                    msg_time + self.reception_delay,
                    (message, msg_condition_arg),
                    false,
                );
            }
        }
        None
    }

    pub fn receive(&self, condition_arg: &ConditionArgType, time: f32) -> MessageType {
        let mut message_buffer = self.message_buffer.lock().unwrap();
        let min_time_buffer = message_buffer.min_time().map(|(t, _)| t);
        // Check first if there is a message in the buffer that can be received before trying to receive new messages, to avoid receiving messages that should be received later
        if let Some(min_time) = min_time_buffer {
            if min_time - time <= self.time_round {
                let (message, msg_condition_arg) = message_buffer.remove(min_time).unwrap().1;
                if (self.condition.lock().unwrap())(condition_arg.clone(), msg_condition_arg) {
                    return message;
                }
            }
        }
        loop {
            if let Ok((message, msg_condition_arg, msg_time)) = self.receiver.recv() {
                if msg_time + self.reception_delay - time <= self.time_round && (self.condition.lock().unwrap())(condition_arg.clone(), msg_condition_arg.clone()) {
                    return message;
                } else {
                    message_buffer.insert(
                        msg_time + self.reception_delay,
                        (message, msg_condition_arg),
                        false,
                    );
                }
            }
        }
    }
}
