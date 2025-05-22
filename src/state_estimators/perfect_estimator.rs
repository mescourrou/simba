/*!
Module providing the [`PerfectEstimator`] strategy. This strategy uses directly
the groundtruth to provide the estimation. It can be used when the state used
by the controller should be perfect.
*/

use std::collections::HashMap;

use super::state_estimator::{State, StateRecord};
use crate::constants::TIME_ROUND;
use crate::sensors::sensor::{Observation, SensorObservation};
use crate::simulator::SimulatorConfig;
use crate::stateful::Stateful;
use crate::utils::maths::round_precision;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};
use config_checker::macros::Check;
use log::{debug, error, info};
use serde_derive::{Deserialize, Serialize};

/// Configuration for [`PerfectEstimator`].
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct PerfectEstimatorConfig {
    /// Prediction period.
    #[check(ge(0.))]
    pub prediction_period: f32,
    pub targets: Vec<String>,
}

impl Default for PerfectEstimatorConfig {
    fn default() -> Self {
        Self {
            prediction_period: 0.1,
            targets: vec!["self".to_string()],
        }
    }
}

/// Record for [`PerfectEstimator`].
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PerfectEstimatorRecord {
    /// Current state estimated
    pub states: Vec<(String, StateRecord)>,
    /// Last change of state
    pub last_time_update: f32,
}

/// Estimation strategy without any error.
#[derive(Debug)]
pub struct PerfectEstimator {
    /// Estimation of the state on the `last_time_update`.
    states: HashMap<String, State>,
    /// Prediction period, in seconds.
    prediction_period: f32,
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
        let mut states = HashMap::new();
        for target in &config.targets {
            states.insert(target.clone(), State::new());
        }
        Self {
            prediction_period: config.prediction_period,
            states,
            last_time_update: 0.,
        }
    }
}

use super::state_estimator::{StateEstimator, StateEstimatorRecord};
use crate::node::Node;

impl StateEstimator for PerfectEstimator {
    fn prediction_step(&mut self, node: &mut Node, time: f32) {
        if (time - self.next_time_step()).abs() > TIME_ROUND / 2. {
            error!("Error trying to update estimate too soon !");
            return;
        }
        info!("Doing prediction step");
        for (target, state) in &mut self.states {
            if target.to_lowercase() == "self".to_string() {
                let arc_physic = node
                    .physics()
                    .expect("Node with state_estimator should have physics");
                let physic = arc_physic.read().unwrap();

                *state = physic.state(time).clone();
            } else {
                *state = node
                    .service_manager()
                    .read()
                    .unwrap()
                    .get_real_state(target, node, time)
                    .expect(
                        format!(
                            "{target} does not have physics, no perfect state can be computed!"
                        )
                        .as_str(),
                    );
            }
        }
        self.last_time_update = time;
    }

    fn correction_step(&mut self, _node: &mut Node, _observations: &Vec<Observation>, _time: f32) {
        info!("Got observations at time {_time}: {:?}", _observations);
    }

    fn state(&self) -> State {
        if self.states.contains_key(&"self".to_string()) {
            self.states["self"].clone()
        } else {
            panic!(
                "PerfectEstimator should contain the target 'self' to be used in the control loop."
            );
        }
    }

    fn next_time_step(&self) -> f32 {
        round_precision(self.last_time_update + self.prediction_period, TIME_ROUND).unwrap()
    }
}

impl Stateful<StateEstimatorRecord> for PerfectEstimator {
    fn record(&self) -> StateEstimatorRecord {
        let mut state_records = Vec::new();
        for (target, state) in &self.states {
            state_records.push((target.clone(), state.record()));
        }
        StateEstimatorRecord::Perfect(PerfectEstimatorRecord {
            states: state_records,
            last_time_update: self.last_time_update,
        })
    }

    fn from_record(&mut self, record: StateEstimatorRecord) {
        if let StateEstimatorRecord::Perfect(record_state_estimator) = record {
            for (target, state_record) in &record_state_estimator.states {
                self.states
                    .get_mut(target)
                    .expect(
                        format!("Target {target} not found among the PerfectEstimator targets")
                            .as_str(),
                    )
                    .from_record(state_record.clone());
                self.last_time_update = record_state_estimator.last_time_update;
            }
        } else {
            error!(
                "Using a StateEstimatorRecord type which does not match the used StateEstimator (PerfectEstimator)"
            );
        }
    }
}
