//! Scenario event configuration types.
//!
//! This module defines the configuration schema used to declare scenario-driven events,
//! including trigger conditions and resulting actions.
//! Triggers are represented by [`EventTriggerConfig`], actions by [`EventTypeConfig`], and
//! complete entries by [`EventConfig`] inside [`ScenarioConfig`].

use serde::{Deserialize, Serialize};
use simba_macros::config_derives;

use crate::config::NumberConfig;

/// Root scenario configuration.
///
/// Contains the list of event declarations evaluated by the scenario system.
///
/// The events will be published on the chanel `/simba/scenario` when they are triggered, with the content of the event record (see [`EventRecord`]).
///
/// Default values:
/// - `events`: empty vector
#[config_derives]
#[derive(Default)]
pub struct ScenarioConfig {
    /// Event definitions evaluated by the scenario engine.
    #[check]
    pub events: Vec<EventConfig>,
}

/// Configuration of a single scenario event.
///
/// An event combines a trigger condition ([`EventTriggerConfig`]) and an action
/// ([`EventTypeConfig`]).
///
/// Default values:
/// - `triggering_nodes`: empty vector
/// - `trigger`: [`EventTriggerConfig::default`] (time trigger)
/// - `event_type`: [`EventTypeConfig::default`] (kill `"$0"`)
#[config_derives]
#[derive(Default)]
pub struct EventConfig {
    /// Names of the nodes that can trigger this event. If empty, any node can trigger it.
    /// Regexp patterns are supported.
    /// Only applied to non-time triggers.
    pub triggering_nodes: Vec<String>,
    /// Trigger condition for the event.
    pub trigger: EventTriggerConfig,
    /// Action executed when the trigger condition is met.
    pub event_type: EventTypeConfig,
}

/// Trigger condition for scenario events.
///
/// Default value: [`EventTriggerConfig::Time`] with [`TimeEventTriggerConfig::default`].
#[config_derives(tag_content)]
pub enum EventTriggerConfig {
    /// Time-based trigger.
    #[check]
    Time(TimeEventTriggerConfig),
    /// Distance-based trigger relative to another target.
    #[check]
    Proximity(ProximityEventTriggerConfig),
    /// Geometric-area trigger.
    #[check]
    Area(AreaEventTriggerConfig),
}

impl Default for EventTriggerConfig {
    fn default() -> Self {
        Self::Time(TimeEventTriggerConfig::default())
    }
}

/// Time-based event trigger configuration.
///
/// Default values:
/// - `time`: `NumberConfig::Num(0.0)`
/// - `occurences`: `NumberConfig::Num(1.0)`
#[config_derives]
pub struct TimeEventTriggerConfig {
    /// Time specification used to schedule events.
    pub time: NumberConfig,
    /// Four cases:
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

/// Area-based trigger configuration.
///
/// Default value: [`AreaEventTriggerConfig::Rect`] with [`RectAreaEventTriggerConfig::default`].
#[config_derives]
pub enum AreaEventTriggerConfig {
    /// Axis-aligned rectangle trigger area.
    Rect(RectAreaEventTriggerConfig),
    /// Circular trigger area.
    Circle(CircleAreaEventTriggerConfig),
}

impl Default for AreaEventTriggerConfig {
    fn default() -> Self {
        Self::Rect(RectAreaEventTriggerConfig::default())
    }
}

/// Rectangular area trigger configuration.
///
/// Default values:
/// - `bottom_left`: `(0.0, 0.0)`
/// - `top_right`: `(1.0, 1.0)`
/// - `inside`: `true`
#[config_derives]
pub struct RectAreaEventTriggerConfig {
    /// Bottom-left corner of the rectangle.
    pub bottom_left: (f32, f32),
    /// Top-right corner of the rectangle.
    pub top_right: (f32, f32),
    /// If `true`, trigger when inside the rectangle; otherwise when outside.
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

/// Circular area trigger configuration.
///
/// Default values:
/// - `center`: `(0.0, 0.0)`
/// - `radius`: `1.0`
/// - `inside`: `true`
#[config_derives]
pub struct CircleAreaEventTriggerConfig {
    /// Center of the circle.
    pub center: (f32, f32),
    /// Radius of the circle.
    pub radius: f32,
    /// If `true`, trigger when inside the circle; otherwise when outside.
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

/// Proximity trigger configuration.
///
/// Default values:
/// - `protected_target`: `None`
/// - `distance`: `1.0`
/// - `inside`: `true`
#[config_derives]
pub struct ProximityEventTriggerConfig {
    /// Optional target name to protect from trigger activation.
    pub protected_target: Option<String>,
    /// Distance threshold for trigger evaluation.
    pub distance: f32,
    /// If `true`, trigger when distance is below threshold; otherwise above.
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
    /// Spawns a new node according to [`SpawnEventConfig`].
    Spawn(SpawnEventConfig),
    /// Kills a node by name.
    Kill(String),
}

impl Default for EventTypeConfig {
    fn default() -> Self {
        Self::Kill("$0".to_string())
    }
}

/// Spawn event configuration.
///
/// Default values:
/// - `model_name`: `"default_robot"`
/// - `node_name`: `"my_new_robot"`
#[config_derives]
pub struct SpawnEventConfig {
    /// Name of the model/template used for spawning.
    pub model_name: String,
    /// Name assigned to the spawned node.
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

/// Record emitted when an event is evaluated/executed.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EventRecord {
    /// Trigger configuration that led to this record.
    pub trigger: EventTriggerConfig,
    /// Event action associated with this record.
    pub event: EventTypeConfig,
}
