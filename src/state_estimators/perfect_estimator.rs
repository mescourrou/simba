/*!
Module providing the [`PerfectEstimator`] strategy. This strategy uses directly
the groundtruth to provide the estimation. It can be used when the state used
by the controller should be perfect.
*/

use super::state_estimator::{State, StateRecord};
use crate::sensors::sensor::GenericObservation;
use crate::simulator::SimulatorMetaConfig;
use crate::stateful::Stateful;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};
use log::error;
use serde_derive::{Deserialize, Serialize};

/// Configuration for [`PerfectEstimator`].
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct PerfectEstimatorConfig {
    /// Prediction period.
    pub update_period: f32,
}

impl Default for PerfectEstimatorConfig {
    fn default() -> Self {
        Self { update_period: 0.1 }
    }
}

/// Record for [`PerfectEstimator`].
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PerfectEstimatorRecord {
    /// Current state estimated
    pub state: StateRecord,
    /// Last change of state
    pub last_time_update: f32,
}

/// Estimation strategy without any error.
#[derive(Debug)]
pub struct PerfectEstimator {
    /// Estimation of the state on the `last_time_update`.
    state: State,
    /// Update period, in seconds.
    update_period: f32,
    /// Last time the state was updated/predicted.
    last_time_update: f32,
}

impl PerfectEstimator {
    /// Create a new [`PerfectEstimator`] using default [`PerfectEstimatorConfig`].
    pub fn new() -> Self {
        Self::from_config(
            &PerfectEstimatorConfig::default(),
            &None,
            SimulatorMetaConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Creates a new [`PerfectEstimator`] from the given `config`.
    pub fn from_config(
        config: &PerfectEstimatorConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        _meta_config: SimulatorMetaConfig,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        Self {
            update_period: config.update_period,
            state: State::new(),
            last_time_update: 0.,
        }
    }
}

use super::state_estimator::{StateEstimator, StateEstimatorRecord};
use crate::turtlebot::Turtlebot;

impl StateEstimator for PerfectEstimator {
    fn prediction_step(&mut self, turtle: &mut Turtlebot, time: f32) {
        let arc_physic = turtle.physics();
        let physic = arc_physic.read().unwrap();
        if time < self.next_time_step() {
            error!("Error trying to update estimate too soon !");
            return;
        }
        self.state = physic.state(time).clone();
        self.last_time_update = time;
    }

    fn correction_step(
        &mut self,
        _turtle: &mut Turtlebot,
        _observations: &Vec<Box<dyn GenericObservation>>,
        _time: f32,
    ) {
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
        StateEstimatorRecord::Perfect(PerfectEstimatorRecord {
            state: self.state.record(),
            last_time_update: self.last_time_update,
        })
    }

    fn from_record(&mut self, record: StateEstimatorRecord) {
        if let StateEstimatorRecord::Perfect(record_state_estimator) = record {
            self.state.from_record(record_state_estimator.state);
            self.last_time_update = record_state_estimator.last_time_update;
        } else {
            error!(
                "Using a StateEstimatorRecord type which does not match the used StateEstimator (PerfectEstimator)"
            );
        }
    }
}
