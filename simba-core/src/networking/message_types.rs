//! Definitions of message types and transport envelopes used in the networking layer.

use simba_macros::EnumToString;

use crate::{
    navigators::go_to::GoToMessage,
    scenario::config::EventRecord,
    sensors::{Observation, sensor_manager::SensorTriggerMessage},
};

/// Payload variants that can transit through the network.
///
/// This enum is exposed to Python through `pyo3` and is serializable for transport.
#[derive(Debug, Clone, EnumToString)]
pub enum MessageTypes {
    /// Arbitrary UTF-8 textual payload.
    String(String),
    /// Navigation payload used by [`GoToMessage`].
    GoTo(GoToMessage),
    /// Sensor event payload used by [`SensorTriggerMessage`].
    SensorTrigger(SensorTriggerMessage),
    /// Observations payload used by [`Observation`].
    Observations(Vec<Observation>),
    /// Scenario event record payload used by [`EventRecord`].
    Event(EventRecord),
    /// Custom payloads that can be serialized and deserialized as needed.
    Custom(Vec<u8>),
}

macro_rules! impl_message_types_from {
    ($($variant:ident, $type:ty),* $(,)?) => {
        $(
            impl From<$type> for MessageTypes {
                fn from(value: $type) -> Self {
                    MessageTypes::$variant(value)
                }
            }

            impl From<MessageTypes> for $type {
                fn from(value: MessageTypes) -> Self {
                    match value {
                        MessageTypes::$variant(inner) => inner,
                        _ => panic!("Cannot convert MessageTypes::{} into {}", value, stringify!($type)),
                    }
                }
            }
        )*
    };
}

impl Default for MessageTypes {
    fn default() -> Self {
        MessageTypes::String(String::new())
    }
}

impl_message_types_from!(
    String,
    String,
    GoTo,
    GoToMessage,
    SensorTrigger,
    SensorTriggerMessage,
    Observations,
    Vec<Observation>,
    Event,
    EventRecord,
    Custom,
    Vec<u8>,
);

impl From<&str> for MessageTypes {
    fn from(value: &str) -> Self {
        MessageTypes::String(value.to_string())
    }
}
