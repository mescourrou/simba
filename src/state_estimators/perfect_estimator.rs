// Configuration for PerfectPhysic
use serde_derive::{Serialize, Deserialize};
use super::state_estimator::{State, StateRecord};
use crate::sensors::sensor::GenericObservation;
use crate::plugin_api::PluginAPI;

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct PerfectEstimatorConfig {
    update_period: f32
}

impl Default for PerfectEstimatorConfig {
    fn default() -> Self {
        Self {
            update_period: 0.1
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct PerfectEstimatorRecord {
    pub state: StateRecord
}



use crate::physics::physic::Physic;

#[derive(Debug)]
pub struct PerfectEstimator {
    state: State,
    update_period: f32,
    last_time_update: f32
}

impl PerfectEstimator {
    pub fn new() -> Self {
        Self::from_config(&PerfectEstimatorConfig::default(), &None)
    }

    pub fn from_config(config: &PerfectEstimatorConfig, _plugin_api: &Option<Box<dyn PluginAPI>>) -> Self {
        Self {
            update_period: config.update_period,
            state: State::new(),
            last_time_update: 0.
        }
    }

    
}

use super::state_estimator::{StateEstimator, StateEstimatorRecord};

impl StateEstimator for PerfectEstimator {
    fn prediction_step(&mut self, time: f32, physic: &dyn Physic) {
        if time < self.next_time_step() {
            println!("Error trying to update estimate too soon !");
            return;
        }
        self.state = physic.state(time).clone();
        self.last_time_update = time;
    }

    fn correction_step(&mut self, observations: Vec<Box<dyn GenericObservation>>, _time: f32, _physic: &dyn Physic) {
        println!("Receive observations, but not needed, I'm perfect !\n{:?}", observations);
    }

    fn state(&self) -> State {
        self.state.clone()
    }

    fn next_time_step(&self) -> f32 {
        self.last_time_update + self.update_period
    }

    fn record(&self) -> StateEstimatorRecord {
        StateEstimatorRecord::Perfect(
            PerfectEstimatorRecord {
                state: self.state.record()
        })
    }
}