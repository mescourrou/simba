use serde_json::Value;
use std::collections::BTreeMap as Map;

use super::state_estimator::{State, StateRecord, StateEstimator, self};
use crate::plugin_api::PluginAPI;


use serde_derive::{Serialize, Deserialize};
use crate::sensors::sensor::GenericObservation;
use super::state_estimator::StateEstimatorRecord;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ExternalEstimatorConfig {
    #[serde(flatten)]
    config: Map<String, Value>
}

impl Default for ExternalEstimatorConfig {
    fn default() -> Self {
        Self {
            config: Map::new()
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ExternalEstimatorRecord {
    pub state: StateRecord
}



use crate::physics::physic::Physic;

pub struct ExternalEstimator {
    state_estimator: Box<dyn StateEstimator>,
    last_time_update: f32,
    update_period: f32
}

impl ExternalEstimator {
    pub fn new() -> Self {
        Self::from_config(&ExternalEstimatorConfig::default(), &None)
    }

    pub fn from_config(config: &ExternalEstimatorConfig, plugin_api: &Option<Box<dyn PluginAPI>>) -> Self {
        println!("Config given: {:?}", config);
        Self {
            state_estimator: plugin_api.as_ref().expect("Plugin API not set!").get_state_estimator(&config.config),
            last_time_update: 0.,
            update_period: 0.1
        }
    }

    
}

impl std::fmt::Debug for ExternalEstimator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalEstimator {{}}")
    }
}


impl StateEstimator for ExternalEstimator {
    fn prediction_step(&mut self, time: f32, physic: &dyn Physic) {
        if time < self.next_time_step() {
            println!("Error trying to update estimate too soon !");
            return;
        }
        self.state_estimator.prediction_step(time, physic);
        self.last_time_update = time;
    }

    fn correction_step(&mut self, observations: Vec<Box<dyn GenericObservation>>, _time: f32, _physic: &dyn Physic) {
        println!("Receive observations, but I don't know what to do with them");
    }

    fn state(&self) -> State {
        State::new()
    }

    fn next_time_step(&self) -> f32 {
        self.last_time_update + self.update_period
    }

    fn record(&self) -> StateEstimatorRecord {
        StateEstimatorRecord::External(
            ExternalEstimatorRecord {
                state: self.state().record()
        })
    }
}