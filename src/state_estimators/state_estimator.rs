/*!
Module defining the [`StateEstimator`] trait, which need to be implemented
for the state estimation strategies.
*/

extern crate nalgebra as na;
use config_checker::macros::Check;
use na::SVector;

extern crate confy;
use serde_derive::{Deserialize, Serialize};

/// Configuration for [`State`] in order to load a state from the configuration.
///
/// The pose should contain 3 elements.
/// TODO: Make a config validation scheme.
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct StateConfig {
    /// Position and orientation of the robot
    pub pose: Vec<f32>,
    /// Linear velocity
    #[check(ge(0.))]
    pub velocity: f32,
}

impl Default for StateConfig {
    fn default() -> Self {
        Self {
            pose: vec![0., 0., 0.],
            velocity: 0.,
        }
    }
}

/// Record for [`State`] in order to record a state.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct StateRecord {
    /// Position and orientation of the robot
    pub pose: Vec<f32>,
    /// Linear velocity.
    pub velocity: f32,
}

impl Default for StateRecord {
    fn default() -> Self {
        Self {
            pose: vec![0., 0., 0.],
            velocity: 0.,
        }
    }
}

/// State to be estimated.
#[derive(Debug, Clone)]
pub struct State {
    /// Pose of the robot [x, y, orientation]
    pub pose: SVector<f32, 3>,
    /// Linear velocity of the robot (in the longitudinal direction).
    pub velocity: f32,
}

impl State {
    /// Creates a new [`State`], with all values to 0.
    pub fn new() -> Self {
        Self {
            pose: SVector::<f32, 3>::new(0., 0., 0.),
            velocity: 0.,
        }
    }

    pub fn from_vector(vec: Vec<f32>) -> Self {
        let mut state = State::new();
        if vec.len() >= 1 {
            state.pose.x = vec[0];
        }
        if vec.len() >= 2 {
            state.pose.y = vec[1];
        }
        if vec.len() >= 3 {
            state.pose.z = vec[2];
        }
        state
    }

    /// Load a [`State`] from the `config` ([`StateConfig`]).
    pub fn from_config(config: &StateConfig) -> Self {
        let mut state = Self::new();

        let mut i: usize = 0;
        for coord in &config.pose {
            if i >= 3 {
                break;
            }
            state.pose[i] = *coord;
            i += 1;
        }
        state.velocity = config.velocity;
        state
    }

    pub fn theta_modulo(mut self) -> Self {
        self.pose.z = mod2pi(self.pose.z);
        self
    }
}

impl Stateful<StateRecord> for State {
    fn record(&self) -> StateRecord {
        StateRecord {
            pose: {
                let mut ve: Vec<f32> = vec![];
                for coord in &self.pose {
                    ve.push(*coord);
                }
                ve
            },
            velocity: self.velocity,
        }
    }

    fn from_record(&mut self, record: StateRecord) {
        self.velocity = record.velocity;
        let mut i: usize = 0;
        for coord in &record.pose {
            self.pose[i] = *coord;
            i += 1;
        }
    }
}

use std::fmt;

impl fmt::Display for State {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            formatter,
            "pose: [{}, {}, {}], v: {}",
            self.pose.x, self.pose.y, self.pose.z, self.velocity
        )?;
        Ok(())
    }
}

use super::perfect_estimator::PerfectEstimatorConfig;
use super::{external_estimator, perfect_estimator};
use crate::node::Node;
use crate::simulator::SimulatorConfig;
use crate::stateful::Stateful;
use crate::utils::geometry::mod2pi;
use crate::{
    plugin_api::PluginAPI, utils::determinist_random_variable::DeterministRandomVariableFactory,
};

use std::sync::{Arc, RwLock};

/// List the possible configs, to be selected in the global config.
///
/// To select the [`StateEstimatorConfig::Perfect`], the yaml config should be:
/// ```YAML
/// state_estimator:
///     Perfect:
///         prediction_period: 0.01
/// ```
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(deny_unknown_fields)]
pub enum StateEstimatorConfig {
    Perfect(perfect_estimator::PerfectEstimatorConfig),
    External(external_estimator::ExternalEstimatorConfig),
}

/// List the possible records.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum StateEstimatorRecord {
    Perfect(perfect_estimator::PerfectEstimatorRecord),
    External(external_estimator::ExternalEstimatorRecord),
}

/**
 * Make the right [`StateEstimator`] from the configuration given.
 */
pub fn make_state_estimator_from_config(
    config: &StateEstimatorConfig,
    plugin_api: &Option<Box<&dyn PluginAPI>>,
    global_config: &SimulatorConfig,
    va_factory: &DeterministRandomVariableFactory,
) -> Box<dyn StateEstimator> {
    return match config {
        StateEstimatorConfig::Perfect(c) => {
            Box::new(perfect_estimator::PerfectEstimator::from_config(
                c,
                plugin_api,
                global_config,
                va_factory,
            )) as Box<dyn StateEstimator>
        }
        StateEstimatorConfig::External(c) => {
            Box::new(external_estimator::ExternalEstimator::from_config(
                c,
                plugin_api,
                global_config,
                va_factory,
            )) as Box<dyn StateEstimator>
        }
    };
}

use crate::sensors::sensor::{Observation, SensorObservation};

pub trait StateEstimator:
    std::fmt::Debug + std::marker::Send + std::marker::Sync + Stateful<StateEstimatorRecord>
{
    /// Prediction step of the state estimator.
    ///
    /// The prediction step should be able to compute the state of the robot at the given time.
    ///
    /// ## Arguments
    /// * `robot` -- mutable reference on the current [`Robot`] to be able to interact with
    /// other modules.
    /// * `time` -- Time to reach.
    fn prediction_step(&mut self, robot: &mut Node, time: f32);

    /// Correction step of the state estimator.
    ///
    /// The correction step processes the observations. The received observations can be of every
    /// types, the implementation should not assert a specific type.
    ///
    /// ## Arguments
    /// * `robot` -- mutable reference on the current [`Robot`] to be able to interact with
    /// other modules.
    /// * `observations` -- Observation vector.
    /// * `time` -- Current time.
    fn correction_step(&mut self, robot: &mut Node, observations: &Vec<Observation>, time: f32);

    /// Return the current estimated state.
    fn state(&self) -> State;

    /// Return the next prediction step time. The correction step
    /// is called for each observation.
    fn next_time_step(&self) -> f32;
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct CentralStateEstimatorRecord {
    pub name: String,
    pub estimator: StateEstimatorRecord,
}

#[derive(Debug, Serialize, Deserialize, Check, Clone)]
pub struct CentralStateEstimatorConfig {
    pub name: String,
    pub robots: Vec<String>,
    #[check]
    pub config: StateEstimatorConfig,
}

impl Default for CentralStateEstimatorConfig {
    fn default() -> Self {
        Self {
            name: "central_state_estimator".to_string(),
            robots: Vec::new(),
            config: StateEstimatorConfig::Perfect(PerfectEstimatorConfig::default()),
        }
    }
}

pub struct CentralStateEstimator {
    pub name: String,
    pub robots: Vec<String>,
    pub estimator: Box<dyn StateEstimator>,
}

impl CentralStateEstimator {
    pub fn from_config(
        config: &CentralStateEstimatorConfig,
        plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        Self {
            name: config.name.clone(),
            robots: config.robots.clone(),
            estimator: make_state_estimator_from_config(
                &config.config,
                plugin_api,
                global_config,
                va_factory,
            ),
        }
    }

    pub fn next_time_step(&self) -> f32 {
        self.estimator.next_time_step()
    }
}

impl Stateful<CentralStateEstimatorRecord> for CentralStateEstimator {
    fn from_record(&mut self, record: CentralStateEstimatorRecord) {
        self.estimator.from_record(record.estimator);
    }

    fn record(&self) -> CentralStateEstimatorRecord {
        CentralStateEstimatorRecord {
            name: self.name.clone(),
            estimator: self.estimator.record(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct BenchStateEstimatorConfig {
    pub name: String,
    #[check]
    pub config: StateEstimatorConfig,
}

impl Default for BenchStateEstimatorConfig {
    fn default() -> Self {
        Self {
            name: String::from("bench_state_estimator"),
            config: StateEstimatorConfig::Perfect(
                perfect_estimator::PerfectEstimatorConfig::default(),
            ),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BenchStateEstimatorRecord {
    pub name: String,
    pub record: StateEstimatorRecord,
}

#[derive(Debug)]
pub struct BenchStateEstimator {
    pub name: String,
    pub state_estimator: Arc<RwLock<Box<dyn StateEstimator>>>,
}
