/*!
Module providing the [`PerfectEstimator`] strategy. This strategy uses directly
the groundtruth to provide the estimation. It can be used when the state used
by the controller should be perfect.
*/

use super::state_estimator::{State, StateRecord};
use crate::constants::TIME_ROUND;
use crate::sensors::sensor::Observation;
use crate::simulator::SimulatorConfig;
use crate::stateful::Stateful;
use crate::utils::maths::round_precision;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};
use config_checker::macros::Check;
use log::error;
use serde_derive::{Deserialize, Serialize};

/// Configuration for [`PerfectEstimator`].
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct PerfectEstimatorConfig {
    /// Prediction period.
    #[check(ge(0.))]
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
            &SimulatorConfig::default(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Creates a new [`PerfectEstimator`] from the given `config`.
    pub fn from_config(
        config: &PerfectEstimatorConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        _global_config: &SimulatorConfig,
        _va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        Self {
            update_period: config.update_period,
            state: State::new(),
            last_time_update: 0.,
        }
    }
}

use super::state_estimator::{StateEstimator, StateEstimatorRecord};
use crate::node::Node;

impl StateEstimator for PerfectEstimator {
    fn prediction_step(&mut self, node: &mut Node, time: f32) {
        let arc_physic = node
            .physics()
            .expect("Node with state_estimator should have physics");
        let physic = arc_physic.read().unwrap();
        if (time - self.next_time_step()).abs() > TIME_ROUND / 2. {
            error!("Error trying to update estimate too soon !");
            return;
        }
        self.state = physic.state(time).clone();
        self.last_time_update = time;
    }

    fn correction_step(&mut self, _node: &mut Node, _observations: &Vec<Observation>, _time: f32) {}

    fn state(&self) -> State {
        self.state.clone()
    }

    fn next_time_step(&self) -> f32 {
        round_precision(self.last_time_update + self.update_period, TIME_ROUND).unwrap()
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
