use std::{
    collections::{BTreeMap, BTreeSet},
    sync::{Arc, Mutex},
};

use log::debug;
#[cfg(not(feature = "force_hard_determinism"))]
use log::warn;
use nalgebra::ComplexField;
use regex::Regex;

use crate::{
    config::NumberConfig,
    constants::TIME_ROUND,
    errors::SimbaResult,
    logger::{InternalLog, is_enabled},
    node::NodeState,
    scenario::config::{
        AreaEventTriggerConfig, EventConfig, EventTriggerConfig, EventTypeConfig,
        ProximityEventTriggerConfig, ScenarioConfig,
    },
    simulator::{RunningParameters, Simulator, SimulatorConfig},
    state_estimators::State,
    utils::{
        determinist_random_variable::DeterministRandomVariableFactory,
        time_ordered_data::TimeOrderedData,
    },
};

#[cfg(not(feature = "force_hard_determinism"))]
use crate::networking::{network::MessageFlag, network_manager::MessageSendMethod};

pub mod config;

pub struct Scenario {
    time_events: TimeOrderedData<(usize, Event)>,
    other_events: Mutex<Vec<Event>>,
    last_executed_time: f32,
}

#[cfg_attr(feature = "force_hard_determinism", allow(dead_code))]
impl Scenario {
    pub fn from_config(
        config: &ScenarioConfig,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Self {
        let (time_events_vec, other_events): (Vec<EventConfig>, Vec<EventConfig>) = config
            .events
            .clone()
            .into_iter()
            .partition(|e| matches!(e.trigger, EventTriggerConfig::Time(_)));
        let mut time_events = TimeOrderedData::new();
        for event in &time_events_vec {
            let ts: Vec<f32> = match &event.trigger {
                EventTriggerConfig::Time(t) => {
                    let occurences = match &t.occurences {
                        NumberConfig::Num(n) => *n as usize,
                        NumberConfig::Rand(rv_config) => {
                            va_factory.make_variable(rv_config.clone()).generate(0.)[0] as usize
                        }
                    };
                    match &t.time {
                        NumberConfig::Num(n) => {
                            if occurences == 0 {
                                (0..(global_config.max_time/n).ceil() as usize).map(|i| n * (i as f32 + 1.)).collect()
                            } else {
                                (0..occurences).map(|i| n * (i as f32 + 1.)).collect()
                            }
                        }
                        NumberConfig::Rand(rv_config) => {
                            let rv = va_factory.make_variable(rv_config.clone());
                            (0..occurences)
                                .flat_map(|i| rv.generate(i as f32))
                                .collect()
                        }
                    }
                }
                _ => unreachable!(),
            };
            for (occurence, t) in ts.iter().enumerate() {
                time_events.insert(*t, (occurence, Event::from_config(&event)), false);
            }
        }
        #[cfg(not(feature = "force_hard_determinism"))]
        for event in &other_events {
            if let EventTypeConfig::Kill(_) = event.event_type {
                warn!("Kill events are not deterministic-stable yet.");
            }
        }
        Self {
            time_events,
            other_events: Mutex::new(other_events.iter().map(Event::from_config).collect()),
            last_executed_time: 0.,
        }
    }

    pub(crate) fn execute_scenario(
        &mut self,
        time: f32,
        simulator: &mut Simulator,
        state_history: &BTreeMap<String, TimeOrderedData<(State, NodeState)>>,
        running_parameters: &mut RunningParameters,
    ) -> SimbaResult<()> {
        if is_enabled(InternalLog::Scenario) {
            debug!("Check scenario");
        }
        // Time events
        for (_, event) in self
            .time_events
            .iter_from_time(self.last_executed_time)
            .take_while(|(t, _)| *t <= time)
        {
            Self::execute_event(
                &event.1,
                simulator,
                time,
                &[event.0.to_string()],
                "Time",
                running_parameters,
            )?;
        }
        // Other events
        let other_events = self.other_events.lock().unwrap();
        for event in other_events.iter() {
            match &event.trigger {
                EventTriggerConfig::Proximity(proximity_config) => {
                    let triggering_nodes =
                        self.proximity_trigger(&event.triggering_nodes, proximity_config, simulator, time, state_history);
                    for nodes in triggering_nodes {
                        Self::execute_event(
                            event,
                            simulator,
                            time,
                            &nodes,
                            "Proximity",
                            running_parameters,
                        )?;
                    }
                }
                EventTriggerConfig::Area(area_config) => {
                    let triggering_nodes =
                        self.area_trigger(&event.triggering_nodes, area_config, simulator, time, state_history);
                    for nodes in triggering_nodes {
                        Self::execute_event(
                            event,
                            simulator,
                            time,
                            &nodes,
                            "Area",
                            running_parameters,
                        )?;
                    }
                }
                EventTriggerConfig::Time(_) => unreachable!(),
            }
        }
        self.last_executed_time = time + TIME_ROUND;
        Ok(())
    }

    fn replace_variables(template_string: &str, variables: &[String]) -> String {
        let mut result_string = template_string.to_owned();
        for (i, var) in variables.iter().enumerate() {
            let var_tag = format!("${}", i);
            result_string = result_string.replace(&var_tag, &var.to_string());
        }
        result_string
    }

    #[cfg_attr(feature = "force_hard_determinism", allow(unused_variables))]
    fn execute_event(
        event: &Event,
        simulator: &mut Simulator,
        time: f32,
        trigger_variables: &[String],
        trigger_type: &str,
        running_parameters: &mut RunningParameters,
    ) -> SimbaResult<()> {
        match &event.event_type {
            #[cfg(not(feature = "force_hard_determinism"))]
            EventTypeConfig::Kill(name) => {
                warn!(
                    "Kill events are not deterministic-stable yet. Use `force_hard_determinism` feature to guarantee time and logical determinism."
                );
                let name = Self::replace_variables(name, trigger_variables);
                log::info!(
                    "Executing Kill event for node `{}` triggered by {}",
                    name,
                    trigger_type
                );
                if let Err(e) = simulator.get_network_manager_mut().send_message(
                    serde_json::Value::Null,
                    MessageSendMethod::Recipient(name.clone()),
                    time,
                    vec![MessageFlag::Kill],
                ) {
                    warn!(
                        "Ignoring error while sending Kill message to node `{}`: {}",
                        name,
                        e.detailed_error()
                    );
                }
            }
            #[cfg(feature = "force_hard_determinism")]
            EventTypeConfig::Kill(_) => {}
            #[cfg(not(feature = "force_hard_determinism"))]
            EventTypeConfig::Spawn(spawn_config) => {
                warn!(
                    "Spawn events are not deterministic-stable yet. Use `force_hard_determinism` feature to guarantee time and logical determinism."
                );
                let model_name =
                    Self::replace_variables(&spawn_config.model_name, trigger_variables);
                let node_name = Self::replace_variables(&spawn_config.node_name, trigger_variables);
                log::info!(
                    "Executing Spawn event for node `{}` of model `{}` triggered by {}",
                    node_name,
                    model_name,
                    trigger_type
                );

                if let Err(e) = simulator.spawn_node_from_name(
                    &model_name,
                    &node_name,
                    running_parameters,
                    time,
                ) {
                    warn!(
                        "Ignoring error while sending Spawn message for node `{}`: {}",
                        node_name,
                        e.detailed_error()
                    );
                }
            }
            #[cfg(feature = "force_hard_determinism")]
            EventTypeConfig::Spawn(_) => {}
        }
        Ok(())
    }

    pub fn next_event_time(&self) -> Option<f32> {
        self.time_events.min_time().map(|(a, _)| a)
    }

    fn area_trigger(
        &self,
        triggering_nodes_filter: &Vec<Regex>,
        area_config: &AreaEventTriggerConfig,
        _simulator: &mut Simulator,
        time: f32,
        state_history: &BTreeMap<String, TimeOrderedData<(State, NodeState)>>,
    ) -> Vec<Vec<String>> {
        let mut triggering_nodes = Vec::new();

        for (node_name, state_history) in state_history {
            if !triggering_nodes_filter.is_empty() && !triggering_nodes_filter
                .iter()
                .any(|re| re.is_match(node_name))
            {
                continue;
            }
            let hstate = state_history
                .iter_from_time(self.last_executed_time)
                .take_while(|(t, _)| *t <= time)
                .last();
            if hstate.is_none() {
                continue;
            }
            let state = &hstate.unwrap().1;
            if state.1 != NodeState::Running {
                // Zombie, not created or killed node
                continue;
            }
            let state = &state.0;
            match area_config {
                AreaEventTriggerConfig::Rect(rect_config) => {
                    let inside = state.pose[0] >= rect_config.bottom_left.0
                        && state.pose[0] <= rect_config.top_right.0
                        && state.pose[1] >= rect_config.bottom_left.1
                        && state.pose[1] <= rect_config.top_right.1;
                    if inside == rect_config.inside {
                        if is_enabled(InternalLog::Scenario) {
                            debug!(
                                "Node `{}` triggered an Area event at time {}",
                                node_name,
                                hstate.unwrap().0
                            );
                        }
                        triggering_nodes.push(vec![node_name.clone()]);
                    }
                }
                AreaEventTriggerConfig::Circle(circle_config) => {
                    let distance_squared = (state.pose[0] - circle_config.center.0).powi(2)
                        + (state.pose[1] - circle_config.center.1).powi(2);
                    let inside = distance_squared <= circle_config.radius.powi(2);
                    if inside == circle_config.inside {
                        triggering_nodes.push(vec![node_name.clone()]);
                    }
                }
            }
        }
        triggering_nodes
    }

    fn proximity_trigger(
        &self,
        triggering_nodes_filter: &Vec<Regex>,
        proximity_config: &ProximityEventTriggerConfig,
        _simulator: &mut Simulator,
        time: f32,
        state_history: &BTreeMap<String, TimeOrderedData<(State, NodeState)>>,
    ) -> Vec<Vec<String>> {
        let node_positions = state_history
            .iter()
            .filter_map(|(node_name, history)| {
                if !triggering_nodes_filter.is_empty() && !triggering_nodes_filter
                    .iter()
                    .any(|re| re.is_match(node_name))
                {
                    return None;
                }
                let hstate = history
                    .iter_from_time(self.last_executed_time)
                    .take_while(|(t, _)| *t <= time)
                    .last();
                hstate?;
                let state = &hstate.unwrap().1;
                if state.1 != NodeState::Running {
                    // Zombie, not created or killed node
                    return None;
                }
                let state = &state.0;
                Some((node_name.clone(), state.pose[0], state.pose[1]))
            })
            .collect::<Vec<_>>();

        let mut triggering_nodes = BTreeSet::new();

        let distance_threshold_squared = proximity_config.distance.powi(2);
        for (node1_name, node1_x, node1_y) in &node_positions {
            for (node2_name, node2_x, node2_y) in &node_positions {
                if node1_name >= node2_name {
                    continue;
                }
                if let Some(target_name) = &proximity_config.protected_target
                    && target_name != node2_name
                    && target_name != node1_name
                {
                    continue;
                }
                let distance_squared =
                    (*node1_x - *node2_x).powi(2) + (*node1_y - *node2_y).powi(2);
                let inside = distance_squared <= distance_threshold_squared;
                if inside == proximity_config.inside {
                    if is_enabled(InternalLog::Scenario) {
                        debug!(
                            "Nodes `{}` and `{}` triggered a Proximity event at time {}",
                            node1_name, node2_name, time
                        );
                    }
                    triggering_nodes.insert(node1_name.clone());
                    triggering_nodes.insert(node2_name.clone());
                }
            }
        }
        triggering_nodes.into_iter().map(|n| vec![n]).collect()
    }
}

#[derive(Debug, Clone)]
pub struct Event {
    pub triggering_nodes: Vec<Regex>,
    pub trigger: EventTriggerConfig,
    pub event_type: EventTypeConfig,
}

impl Event {
    pub fn from_config(
        config: &EventConfig,
    ) -> Self {
        let triggering_nodes = config
            .triggering_nodes
            .iter()
            .map(|pattern| Regex::new(pattern).unwrap())
            .collect();
        Self {
            triggering_nodes,
            trigger: config.trigger.clone(),
            event_type: config.event_type.clone(),
        }
    }
}