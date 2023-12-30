use serde_json::Value;

use super::state_estimator::{State, StateEstimator};
use crate::plugin_api::PluginAPI;


use serde_derive::{Serialize, Deserialize};
use crate::sensors::sensor::GenericObservation;
use super::state_estimator::StateEstimatorRecord;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ExternalEstimatorConfig {
    #[serde(flatten)]
    config: Value
}

impl Default for ExternalEstimatorConfig {
    fn default() -> Self {
        Self {
            config: Value::Null
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ExternalEstimatorRecord {
    #[serde(flatten)]
    pub record: Value
}



use crate::physics::physic::Physic;

pub struct ExternalEstimator {
    state_estimator: Box<dyn StateEstimator>
}

impl ExternalEstimator {
    pub fn new() -> Self {
        Self::from_config(&ExternalEstimatorConfig::default(), &None)
    }

    pub fn from_config(config: &ExternalEstimatorConfig, plugin_api: &Option<Box<dyn PluginAPI>>) -> Self {
        println!("Config given: {:?}", config);
        Self {
            state_estimator: plugin_api.as_ref().expect("Plugin API not set!").get_state_estimator(&config.config)
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
    }

    fn correction_step(&mut self, observations: Vec<Box<dyn GenericObservation>>, time: f32, physic: &dyn Physic) {
        self.state_estimator.correction_step(observations, time, physic);
    }

    fn state(&self) -> State {
        self.state_estimator.state()
    }

    fn next_time_step(&self) -> f32 {
        self.state_estimator.next_time_step()
    }

    fn record(&self) -> StateEstimatorRecord {
        self.state_estimator.record()
    }
}