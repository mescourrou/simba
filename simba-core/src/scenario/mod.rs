use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    sync::{Arc, Mutex},
};

use log::debug;
use log::warn;
use regex::Regex;
use simba_com::{pub_sub::BrokerTrait, time_ordered_data::TimeOrderedData};

use crate::{
    config::NumberConfig,
    constants::TIME_ROUND,
    errors::SimbaResult,
    logger::{InternalLog, is_enabled},
    node::{self, NodeState},
    scenario::config::{
        AreaEventTriggerConfig, EventConfig, EventTriggerConfig, EventTypeConfig,
        ProximityEventTriggerConfig, ScenarioConfig,
    },
    simulator::{
        RunningParameters, SimbaBroker, SimbaBrokerMultiClient, Simulator, SimulatorConfig,
    },
    state_estimators::State,
    utils::{SharedRwLock, determinist_random_variable::DeterministRandomVariableFactory},
};

use crate::networking::{network::MessageFlag, network_manager::MessageSendMethod};

pub mod config;

pub struct Scenario {
    time_events: TimeOrderedData<(usize, Event)>,
    other_events: Mutex<Vec<Event>>,
    last_executed_time: f32,
    broker: SharedRwLock<SimbaBroker>,
}

impl Scenario {
    pub fn from_config(
        config: &ScenarioConfig,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        broker: &SharedRwLock<SimbaBroker>,
    ) -> Self {
        let (time_events_vec, other_events): (Vec<EventConfig>, Vec<EventConfig>) = config
            .events
            .clone()
            .into_iter()
            .partition(|e| matches!(e.trigger, EventTriggerConfig::Time(_)));
        let mut time_events = TimeOrderedData::new(TIME_ROUND);
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
                                (0..(global_config.max_time / n).ceil() as usize)
                                    .map(|i| n * (i as f32 + 1.))
                                    .collect()
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
                time_events.insert(*t, (occurence, Event::from_config(event)), false);
            }
        }
        Self {
            time_events,
            other_events: Mutex::new(other_events.iter().map(Event::from_config).collect()),
            last_executed_time: 0.,
            broker: broker.clone(),
        }
    }

    pub(crate) fn execute_scenario(
        &mut self,
        time: f32,
        simulator: &mut Simulator,
        node_states: &HashMap<String, Option<[f32; 2]>>,
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
            self.execute_event(
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
                    let triggering_nodes = self.proximity_trigger(
                        &event.triggering_nodes,
                        proximity_config,
                        simulator,
                        time,
                        node_states,
                    );
                    for nodes in triggering_nodes {
                        self.execute_event(
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
                    let triggering_nodes = self.area_trigger(
                        &event.triggering_nodes,
                        area_config,
                        simulator,
                        time,
                        node_states,
                    );
                    for nodes in triggering_nodes {
                        self.execute_event(
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

    fn execute_event(
        &self,
        event: &Event,
        simulator: &mut Simulator,
        time: f32,
        trigger_variables: &[String],
        trigger_type: &str,
        running_parameters: &mut RunningParameters,
    ) -> SimbaResult<()> {
        match &event.event_type {
            EventTypeConfig::Kill(name) => {
                use simba_com::pub_sub::PathKey;

                use crate::networking;

                let name = Self::replace_variables(name, trigger_variables);
                log::info!(
                    "Executing Kill event for node `{}` triggered by {}",
                    name,
                    trigger_type
                );
                let command_key = PathKey::from_str(networking::channels::internal::COMMAND)
                    .join_str(name.as_str());
                if !self.broker.write().unwrap().channel_exists(&command_key) {
                    warn!(
                        "Ignoring error while sending Kill message to node `{}`: this node seems to not exist",
                        name
                    );
                } else {
                    use crate::networking::network::Envelope;

                    let tmp_client =
                        self.broker
                            .write()
                            .unwrap()
                            .subscribe_to(&command_key, name, 0.);
                    tmp_client.unwrap().send(
                        Envelope {
                            from: "".to_string(),
                            message: serde_json::Value::Null,
                            message_flags: vec![MessageFlag::Kill],
                            timestamp: time,
                        },
                        time,
                    );
                }
            }
            EventTypeConfig::Spawn(spawn_config) => {
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
        }
        Ok(())
    }

    pub fn next_event_time(&self) -> Option<f32> {
        self.time_events.min_time().map(|(a, _)| a)
    }

    fn area_trigger(
        &self,
        triggering_nodes_filter: &[Regex],
        area_config: &AreaEventTriggerConfig,
        _simulator: &mut Simulator,
        time: f32,
        node_states: &HashMap<String, Option<[f32; 2]>>,
    ) -> Vec<Vec<String>> {
        let mut triggering_nodes = Vec::new();

        for (node_name, state) in node_states.iter() {
            if !triggering_nodes_filter.is_empty()
                && !triggering_nodes_filter
                    .iter()
                    .any(|re| re.is_match(node_name))
            {
                continue;
            }
            if state.is_none() {
                continue;
            }
            let state = state.unwrap();
            match area_config {
                AreaEventTriggerConfig::Rect(rect_config) => {
                    let inside = state[0] >= rect_config.bottom_left.0
                        && state[0] <= rect_config.top_right.0
                        && state[1] >= rect_config.bottom_left.1
                        && state[1] <= rect_config.top_right.1;
                    if inside == rect_config.inside {
                        if is_enabled(InternalLog::Scenario) {
                            debug!("Node `{}` triggered an Area event", node_name);
                        }
                        triggering_nodes.push(vec![node_name.clone()]);
                    }
                }
                AreaEventTriggerConfig::Circle(circle_config) => {
                    let distance_squared = (state[0] - circle_config.center.0).powi(2)
                        + (state[1] - circle_config.center.1).powi(2);
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
        triggering_nodes_filter: &[Regex],
        proximity_config: &ProximityEventTriggerConfig,
        _simulator: &mut Simulator,
        time: f32,
        node_states: &HashMap<String, Option<[f32; 2]>>,
    ) -> Vec<Vec<String>> {
        let mut triggering_nodes = BTreeSet::new();

        let distance_threshold_squared = proximity_config.distance.powi(2);
        for (node1_name, node1_pos) in node_states {
            let node1_pos = match node1_pos {
                Some(pos) => pos,
                None => continue,
            };
            for (node2_name, node2_pos) in node_states {
                if node1_name >= node2_name {
                    continue;
                }
                let node2_pos = match node2_pos {
                    Some(pos) => pos,
                    None => continue,
                };
                if let Some(target_name) = &proximity_config.protected_target
                    && target_name != node2_name
                    && target_name != node1_name
                {
                    continue;
                }

                let distance_squared =
                    (node1_pos[0] - node2_pos[0]).powi(2) + (node1_pos[1] - node2_pos[1]).powi(2);
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
    pub fn from_config(config: &EventConfig) -> Self {
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
