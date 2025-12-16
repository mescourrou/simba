use std::{
    collections::{BTreeMap, BTreeSet},
    sync::{Arc, Mutex},
};

use log::{debug, warn};

use crate::{
    config::NumberConfig,
    constants::TIME_ROUND,
    errors::SimbaResult,
    logger::{is_enabled, InternalLog},
    networking::{network::MessageFlag, network_manager::MessageSendMethod, MessageTypes},
    scenario::config::{
        AreaEventTriggerConfig, EventConfig, EventTriggerConfig, EventTypeConfig,
        ProximityEventTriggerConfig, ScenarioConfig,
    },
    simulator::Simulator,
    state_estimators::State,
    utils::{
        determinist_random_variable::DeterministRandomVariableFactory,
        time_ordered_data::TimeOrderedData,
    },
};

pub mod config;

pub struct Scenario {
    time_events: TimeOrderedData<EventConfig>,
    other_events: Mutex<Vec<EventConfig>>,
    last_executed_time: f32,
}

#[cfg_attr(feature = "force_hard_determinism", allow(dead_code))]
impl Scenario {
    pub fn from_config(
        config: &ScenarioConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Self {
        let (time_events_vec, other_events): (Vec<EventConfig>, Vec<EventConfig>) = config
            .events
            .clone()
            .into_iter()
            .partition(|e| match e.trigger {
                EventTriggerConfig::Time(_) => true,
                _ => false,
            });
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
                            (0..occurences).map(|i| n * (i as f32 + 1.)).collect()
                        }
                        NumberConfig::Rand(rv_config) => {
                            let rv = va_factory.make_variable(rv_config.clone());
                            (0..occurences)
                                .map(|i| rv.generate(i as f32))
                                .flatten()
                                .collect()
                        }
                    }
                }
                _ => unreachable!(),
            };
            for t in ts {
                time_events.insert(t, event.clone(), false);
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
            other_events: Mutex::new(other_events),
            last_executed_time: 0.,
        }
    }

    pub fn execute_scenario(
        &mut self,
        time: f32,
        simulator: &mut Simulator,
        state_history: &BTreeMap<String, TimeOrderedData<(State, bool)>>,
    ) -> SimbaResult<()> {
        // Time events
        for (_, event) in self
            .time_events
            .iter_from_time(self.last_executed_time)
            .take_while(|(t, _)| *t <= time)
        {
            Self::execute_event(&event, simulator, time, &Vec::new(), "Time")?;
        }
        // Other events
        let other_events = self.other_events.lock().unwrap();
        for event in other_events.iter() {
            match &event.trigger {
                EventTriggerConfig::Proximity(proximity_config) => {
                    let triggering_nodes =
                        self.proximity_trigger(proximity_config, simulator, time, state_history);
                    for nodes in triggering_nodes {
                        Self::execute_event(&event, simulator, time, &nodes, "Proximity")?;
                    }
                }
                EventTriggerConfig::Area(area_config) => {
                    let triggering_nodes =
                        self.area_trigger(area_config, simulator, time, state_history);
                    for nodes in triggering_nodes {
                        Self::execute_event(&event, simulator, time, &nodes, "Area")?;
                    }
                }
                EventTriggerConfig::Time(_) => unreachable!(),
            }
        }
        self.last_executed_time = time + TIME_ROUND / 2.;
        Ok(())
    }

    fn replace_variables(template_string: &String, variables: &Vec<String>) -> String {
        let mut result_string = template_string.clone();
        for (i, var) in variables.iter().enumerate() {
            let var_tag = format!("${}", i);
            result_string = result_string.replace(&var_tag, &var.to_string());
        }
        result_string
    }

    fn execute_event(
        event: &EventConfig,
        simulator: &mut Simulator,
        time: f32,
        trigger_variables: &Vec<String>,
        trigger_type: &str,
    ) -> SimbaResult<()> {
        match &event.event_type {
            #[cfg(not(feature = "force_hard_determinism"))]
            EventTypeConfig::Kill(name) => {
                warn!("Kill events are not deterministic-stable yet. Use `force_hard_determinism` feature to guarantee time and logical determinism.");
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
                        name, e
                    );
                }
            }
            #[cfg(feature = "force_hard_determinism")]
            EventTypeConfig::Kill(_) => {}
            _ => unimplemented!(),
        }
        Ok(())
    }

    pub fn next_event_time(&self) -> Option<f32> {
        self.time_events.min_time().map(|(a, _)| a)
    }

    fn area_trigger(
        &self,
        area_config: &AreaEventTriggerConfig,
        simulator: &mut Simulator,
        time: f32,
        state_history: &BTreeMap<String, TimeOrderedData<(State, bool)>>,
    ) -> Vec<Vec<String>> {
        let mut triggering_nodes = Vec::new();

        for (node_name, state_history) in state_history {
            let hstate = state_history
                .iter_from_time(self.last_executed_time)
                .take_while(|(t, _)| *t <= time)
                .last();
            if hstate.is_none() {
                continue;
            }
            let state = &hstate.unwrap().1;
            if state.1 == true {
                // Zombie node
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
                        debug!(
                            "Node `{}` triggered an Area event at time {}",
                            node_name,
                            hstate.unwrap().0
                        );
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
        proximity_config: &ProximityEventTriggerConfig,
        simulator: &mut Simulator,
        time: f32,
        state_history: &BTreeMap<String, TimeOrderedData<(State, bool)>>,
    ) -> Vec<Vec<String>> {
        let node_positions = state_history
            .iter()
            .filter_map(|(node_name, history)| {
                let hstate = history
                    .iter_from_time(self.last_executed_time)
                    .take_while(|(t, _)| *t <= time)
                    .last();
                if hstate.is_none() {
                    return None;
                }
                let state = &hstate.unwrap().1;
                if state.1 == true {
                    // Zombie node
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
                if let Some(target_name) = &proximity_config.protected_target {
                    if target_name != node2_name && target_name != node1_name {
                        continue;
                    }
                }
                let distance_squared =
                    (*node1_x - *node2_x).powi(2) + (*node1_y - *node2_y).powi(2);
                let inside = distance_squared <= distance_threshold_squared;
                if inside == proximity_config.inside {
                    debug!(
                        "Nodes `{}` and `{}` triggered a Proximity event at time {}",
                        node1_name, node2_name, time
                    );
                    triggering_nodes.insert(node1_name.clone());
                    triggering_nodes.insert(node2_name.clone());
                }
            }
        }
        triggering_nodes.into_iter().map(|n| vec![n]).collect()
    }
}
