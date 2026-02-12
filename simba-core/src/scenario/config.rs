use serde::{Deserialize, Serialize};
use simba_macros::config_derives;

use crate::config::NumberConfig;

#[config_derives]
#[derive(Default)]
pub struct ScenarioConfig {
    #[check]
    pub events: Vec<EventConfig>,
}

#[config_derives]
#[derive(Default)]
pub struct EventConfig {
    /// Names of the nodes that can trigger this event. If empty, any node can trigger it.
    /// Regexp patterns are supported.
    /// Only applied to non-time triggers.
    pub triggering_nodes: Vec<String>,
    pub trigger: EventTriggerConfig,
    pub event_type: EventTypeConfig,
}

#[config_derives(tag_content)]
pub enum EventTriggerConfig {
    Time(TimeEventTriggerConfig),
    Proximity(ProximityEventTriggerConfig),
    Area(AreaEventTriggerConfig),
}

impl Default for EventTriggerConfig {
    fn default() -> Self {
        Self::Time(TimeEventTriggerConfig::default())
    }
}

#[config_derives]
pub struct TimeEventTriggerConfig {
    pub time: NumberConfig,
    /// Two cases:
    /// - if `time` is a fixed number, it turns into a period and `occurences` is how many times the event will be triggered. Use 0 for infinite.
    /// - if `time` is a random variable, `occurences` is how many samples will be drawn from it to schedule the event
    /// - if `time` is a random variable with multiple dimensions, `occurences` is the number of repetitions of the full set of samples:
    ///   if `time` draws N samples, and `occurences` is M, then M*N event times will be scheduled
    pub occurences: NumberConfig,
}

impl Default for TimeEventTriggerConfig {
    fn default() -> Self {
        Self {
            time: NumberConfig::Num(0.0),
            occurences: NumberConfig::Num(1.0),
        }
    }
}

#[config_derives]
pub enum AreaEventTriggerConfig {
    Rect(RectAreaEventTriggerConfig),
    Circle(CircleAreaEventTriggerConfig),
}

impl Default for AreaEventTriggerConfig {
    fn default() -> Self {
        Self::Rect(RectAreaEventTriggerConfig::default())
    }
}

#[config_derives]
pub struct RectAreaEventTriggerConfig {
    pub bottom_left: (f32, f32),
    pub top_right: (f32, f32),
    pub inside: bool,
}

impl Default for RectAreaEventTriggerConfig {
    fn default() -> Self {
        Self {
            bottom_left: (0.0, 0.0),
            top_right: (1.0, 1.0),
            inside: true,
        }
    }
}

#[config_derives]
pub struct CircleAreaEventTriggerConfig {
    pub center: (f32, f32),
    pub radius: f32,
    pub inside: bool,
}

impl Default for CircleAreaEventTriggerConfig {
    fn default() -> Self {
        Self {
            center: (0.0, 0.0),
            radius: 1.0,
            inside: true,
        }
    }
}

#[config_derives]
pub struct ProximityEventTriggerConfig {
    pub protected_target: Option<String>,
    pub distance: f32,
    pub inside: bool,
}

impl Default for ProximityEventTriggerConfig {
    fn default() -> Self {
        Self {
            protected_target: None,
            distance: 1.0,
            inside: true,
        }
    }
}

/// Defines the type of event to execute.
///
/// The name provided in the variants refers to the name of the nodes or use $0 notation for
/// dynamic referencing (e.g., the robot that triggered the event is $0).
#[config_derives(tag_content)]
pub enum EventTypeConfig {
    Spawn(SpawnEventConfig),
    Kill(String),
}

impl Default for EventTypeConfig {
    fn default() -> Self {
        Self::Kill("$0".to_string())
    }
}

#[config_derives]
pub struct SpawnEventConfig {
    pub model_name: String,
    pub node_name: String,
}

impl Default for SpawnEventConfig {
    fn default() -> Self {
        Self {
            model_name: "default_robot".to_string(),
            node_name: "my_new_robot".to_string(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EventRecord {
    pub trigger: EventTriggerConfig,
    pub event: EventTypeConfig,
}
