use serde_json::Value;

use super::state_estimator::{State, StateEstimator};
use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorMetaConfig;
use crate::stateful::Stateful;


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
use crate::turtlebot::Turtlebot;

pub struct ExternalEstimator {
    state_estimator: Box<dyn StateEstimator>
}

impl ExternalEstimator {
    pub fn new() -> Self {
        Self::from_config(&ExternalEstimatorConfig::default(), &None, SimulatorMetaConfig::new())
    }

    pub fn from_config(config: &ExternalEstimatorConfig, plugin_api: &Option<Box<dyn PluginAPI>>, meta_config: SimulatorMetaConfig) -> Self {
        println!("Config given: {:?}", config);
        Self {
            state_estimator: plugin_api.as_ref().expect("Plugin API not set!").get_state_estimator(&config.config, meta_config)
        }
    }

    
}

impl std::fmt::Debug for ExternalEstimator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExternalEstimator {{}}")
    }
}


impl StateEstimator for ExternalEstimator {
    fn prediction_step(&mut self, turtle: &mut Turtlebot, time: f32) {
        if time < self.next_time_step() {
            println!("Error trying to update estimate too soon !");
            return;
        }
        self.state_estimator.prediction_step(turtle, time);
    }

    fn correction_step(&mut self, turtle: &mut Turtlebot, observations: Vec<Box<dyn GenericObservation>>, time: f32) {
        self.state_estimator.correction_step(turtle, observations, time);
    }

    fn state(&self) -> State {
        self.state_estimator.state()
    }

    fn next_time_step(&self) -> f32 {
        self.state_estimator.next_time_step()
    }
}

impl Stateful<StateEstimatorRecord> for ExternalEstimator {
    fn record(&self) -> StateEstimatorRecord {
        self.state_estimator.record()
    }

    fn from_record(&mut self, record: StateEstimatorRecord) {
        self.state_estimator.from_record(record);
    }
}