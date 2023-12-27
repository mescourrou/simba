use std::path::Path;

use dlopen::wrapper::{Container, WrapperApi};

#[derive(WrapperApi)]
struct PluginApi {
    run: extern fn(),
}

use serde_derive::{Serialize, Deserialize};
use super::state_estimator::{State, StateRecord};
use crate::sensors::sensor::GenericObservation;

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct ExternalEstimatorConfig {
    plugin_path: String
}

impl Default for ExternalEstimatorConfig {
    fn default() -> Self {
        Self {
            plugin_path: String::from(".")
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ExternalEstimatorRecord {
    pub state: StateRecord
}



use crate::physics::physic::Physic;

pub struct ExternalEstimator {
    plugin: Container<PluginApi>,
    last_time_update: f32,
    update_period: f32
}

impl ExternalEstimator {
    pub fn new() -> Self {
        Self::from_config(&ExternalEstimatorConfig::default())
    }

    pub fn from_config(config: &ExternalEstimatorConfig) -> Self {
        Self {
            plugin: unsafe { Container::load(config.plugin_path.clone()) }.unwrap(),
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

use super::state_estimator::{StateEstimator, StateEstimatorRecord};

impl StateEstimator for ExternalEstimator {
    fn prediction_step(&mut self, time: f32, physic: &dyn Physic) {
        if time < self.next_time_step() {
            println!("Error trying to update estimate too soon !");
            return;
        }
        self.plugin.run();
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