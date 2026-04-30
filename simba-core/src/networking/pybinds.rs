use pyo3::prelude::*;
use simba_macros::EnumToString;

use crate::{
    navigators::go_to::GoToMessage, networking::message_types::MessageTypes,
    pywrappers::ObservationWrapper, scenario::config::EventRecord,
    sensors::sensor_manager::SensorTriggerMessage,
};

#[derive(Clone, Debug, EnumToString)]
#[pyclass]
#[pyo3(name = "MessageTypes")]
/// Python wrapper for the Rust enum [`MessageTypes`], which represents different types of messages that can be sent through the network.
pub enum MessageTypesWrapper {
    /// Arbitrary UTF-8 textual payload.
    String(String),
    /// Navigation payload used by [`GoToMessage`].
    GoTo(GoToMessage),
    /// Sensor event payload used by [`SensorTriggerMessage`].
    SensorTrigger(SensorTriggerMessage),

    Observations(Vec<ObservationWrapper>),

    Event(String),
    /// Custom binary payload.
    Custom(Vec<u8>),
}
#[pymethods]
impl MessageTypesWrapper {
    /// Creates a [`MessageTypes::GoTo`] from a [`GoToMessage`].
    #[staticmethod]
    pub fn from_goto(message: GoToMessage) -> Self {
        MessageTypesWrapper::GoTo(message)
    }

    /// Creates a [`MessageTypes::SensorTrigger`] from a [`SensorTriggerMessage`].
    #[staticmethod]
    pub fn from_sensor_trigger(message: SensorTriggerMessage) -> Self {
        MessageTypesWrapper::SensorTrigger(message)
    }

    /// Creates a [`MessageTypes::String`] from a string.
    #[staticmethod]
    pub fn from_string(message: String) -> Self {
        MessageTypesWrapper::String(message)
    }

    /// Creates a [`MessageTypes::Observation`] from an [`ObservationWrapper`].
    #[staticmethod]
    pub fn from_observation(observations: Vec<ObservationWrapper>) -> Self {
        MessageTypesWrapper::Observations(observations)
    }

    /// Creates a [`MessageTypes::Custom`] from binary data.
    #[staticmethod]
    pub fn from_custom(data: Vec<u8>) -> Self {
        MessageTypesWrapper::Custom(data)
    }

    /// Returns the contained [`GoToMessage`] when this value is [`MessageTypes::GoTo`].
    pub fn as_goto(&self) -> Option<GoToMessage> {
        match self {
            MessageTypesWrapper::GoTo(msg) => Some(msg.clone()),
            _ => None,
        }
    }

    /// Returns the contained [`SensorTriggerMessage`] when this value is
    /// [`MessageTypes::SensorTrigger`].
    pub fn as_sensor_trigger(&self) -> Option<SensorTriggerMessage> {
        match self {
            MessageTypesWrapper::SensorTrigger(msg) => Some(msg.clone()),
            _ => None,
        }
    }

    /// Returns the contained string when this value is [`MessageTypes::String`].
    pub fn as_string(&self) -> Option<String> {
        match self {
            MessageTypesWrapper::String(s) => Some(s.clone()),
            _ => None,
        }
    }

    /// Returns the contained [`ObservationWrapper`] when this value is [`MessageTypes::Observation`].
    pub fn as_observation(&self) -> Option<Vec<ObservationWrapper>> {
        match self {
            MessageTypesWrapper::Observations(obs) => Some(obs.clone()),
            _ => None,
        }
    }

    /// Returns the contained string when this value is [`MessageTypes::Event`].
    pub fn as_event(&self) -> Option<String> {
        match self {
            MessageTypesWrapper::Event(s) => Some(s.clone()),
            _ => None,
        }
    }

    /// Returns the contained binary data when this value is [`MessageTypes::Custom`].
    pub fn as_custom(&self) -> Option<Vec<u8>> {
        match self {
            MessageTypesWrapper::Custom(data) => Some(data.clone()),
            _ => None,
        }
    }

    /// Returns the variant discriminator as a string.
    #[getter]
    pub fn kind(&self) -> String {
        self.to_string()
    }
}

impl MessageTypesWrapper {
    /// Converts this Python wrapper back into the original Rust enum [`MessageTypes`].
    pub fn to_rust(&self) -> MessageTypes {
        match self {
            MessageTypesWrapper::String(s) => MessageTypes::String(s.clone()),
            MessageTypesWrapper::GoTo(msg) => MessageTypes::GoTo(msg.clone()),
            MessageTypesWrapper::SensorTrigger(msg) => MessageTypes::SensorTrigger(msg.clone()),
            MessageTypesWrapper::Observations(observations) => MessageTypes::Observations(
                observations
                    .iter()
                    .map(ObservationWrapper::to_rust)
                    .collect(),
            ),
            MessageTypesWrapper::Event(event) => MessageTypes::Event(
                serde_json::from_str(event).expect("Failed to parse EventRecord from JSON string"),
            ),
            MessageTypesWrapper::Custom(data) => MessageTypes::Custom(data.clone()),
        }
    }

    /// Creates a [`MessageTypesWrapper`] from a Rust enum [`MessageTypes`].
    pub fn from_rust(msg: &MessageTypes) -> Self {
        match msg {
            MessageTypes::String(s) => MessageTypesWrapper::String(s.clone()),
            MessageTypes::GoTo(msg) => MessageTypesWrapper::GoTo(msg.clone()),
            MessageTypes::SensorTrigger(msg) => MessageTypesWrapper::SensorTrigger(msg.clone()),
            MessageTypes::Observations(observations) => MessageTypesWrapper::Observations(
                observations
                    .iter()
                    .map(ObservationWrapper::from_rust)
                    .collect(),
            ),
            MessageTypes::Event(event) => MessageTypesWrapper::Event(
                serde_json::to_string(event)
                    .expect("Failed to serialize EventRecord to JSON string"),
            ),
            MessageTypes::Custom(data) => MessageTypesWrapper::Custom(data.clone()),
        }
    }
}
