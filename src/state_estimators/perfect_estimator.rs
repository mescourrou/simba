use log::error;
// Configuration for PerfectPhysic
use serde_derive::{Serialize, Deserialize};
use super::state_estimator::{State, StateRecord};
use crate::simulator::SimulatorMetaConfig;
use crate::stateful::Stateful;
use crate::{physics::physic, sensors::sensor::GenericObservation};
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

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PerfectEstimatorRecord {
    pub state: StateRecord,
    pub last_time_update: f32
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
        Self::from_config(&PerfectEstimatorConfig::default(), &None, SimulatorMetaConfig::new())
    }

    pub fn from_config(config: &PerfectEstimatorConfig, _plugin_api: &Option<Box<dyn PluginAPI>>, meta_config: SimulatorMetaConfig) -> Self {
        Self {
            update_period: config.update_period,
            state: State::new(),
            last_time_update: 0.
        }
    }

    
}

use super::state_estimator::{StateEstimator, StateEstimatorRecord};
use crate::turtlebot::Turtlebot;

impl StateEstimator for PerfectEstimator {
    fn prediction_step(&mut self, turtle: &mut Turtlebot, time: f32) {
        let arc_physic = turtle.physics();
        let mut physic = arc_physic.read().unwrap();
        if time < self.next_time_step() {
            error!("Error trying to update estimate too soon !");
            return;
        }
        self.state = physic.state(time).clone();
        self.last_time_update = time;
    }

    fn correction_step(&mut self, turtle: &mut Turtlebot, observations: Vec<Box<dyn GenericObservation>>, _time: f32) {
        
    }

    fn state(&self) -> State {
        self.state.clone()
    }

    fn next_time_step(&self) -> f32 {
        self.last_time_update + self.update_period
    }

}

impl Stateful<StateEstimatorRecord> for PerfectEstimator {
    fn record(&self) -> StateEstimatorRecord {
        StateEstimatorRecord::Perfect(
            PerfectEstimatorRecord {
                state: self.state.record(),
                last_time_update: self.last_time_update
        })
    }
    
    fn from_record(&mut self, record: StateEstimatorRecord) {
        if let StateEstimatorRecord::Perfect(record_state_estimator) = record {
            self.state.from_record(record_state_estimator.state);
            self.last_time_update = record_state_estimator.last_time_update;
        } else {
            error!("Using a PhysicRecord type which does not match the used Physic (PerfectPhysic)");
        }
    }
}