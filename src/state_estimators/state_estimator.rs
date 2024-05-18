/*!
Module defining the [`StateEstimator`] trait, which need to be implemented
for the state estimation strategies.
*/

extern crate nalgebra as na;
use na::SVector;

extern crate confy;
use serde_derive::{Deserialize, Serialize};

/// Configuration for [`State`] in order to load a state from the configuration.
///
/// The pose should contain 3 elements.
/// TODO: Make a config validation scheme.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct StateConfig {
    /// Position and orientation of the robot
    pub pose: Vec<f32>,
    /// Linear velocity
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
        return state;
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

use super::{external_estimator, perfect_estimator};
use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorMetaConfig;
use crate::stateful::Stateful;
use crate::turtlebot::Turtlebot;

use std::sync::{Arc, RwLock};

/// List the possible configs, to be selected in the global config.
///
/// To select the [`StateEstimatorConfig::Perfect`], the yaml config should be:
/// ```YAML
/// state_estimator:
///     Perfect:
///         update_period: 0.01
/// ```
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum StateEstimatorConfig {
    Perfect(Box<perfect_estimator::PerfectEstimatorConfig>),
    External(Box<external_estimator::ExternalEstimatorConfig>),
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
    plugin_api: &Option<Box<dyn PluginAPI>>,
    meta_config: SimulatorMetaConfig,
) -> Arc<RwLock<Box<dyn StateEstimator>>> {
    return match config {
        StateEstimatorConfig::Perfect(c) => Arc::new(RwLock::new(Box::new(
            perfect_estimator::PerfectEstimator::from_config(c, plugin_api, meta_config.clone()),
        )
            as Box<dyn StateEstimator>)),
        StateEstimatorConfig::External(c) => Arc::new(RwLock::new(Box::new(
            external_estimator::ExternalEstimator::from_config(c, plugin_api, meta_config.clone()),
        )
            as Box<dyn StateEstimator>)),
    };
}

use crate::sensors::sensor::GenericObservation;

pub trait StateEstimator:
    std::fmt::Debug + std::marker::Send + std::marker::Sync + Stateful<StateEstimatorRecord>
{
    /// Prediction step of the state estimator.
    ///
    /// The prediction step should be able to compute the state of the robot at the given time.
    ///
    /// ## Arguments
    /// * `turtle` -- mutable reference on the current [`Turtlebot`] to be able to interact with
    /// other modules.
    /// * `time` -- Time to reach.
    fn prediction_step(&mut self, turtle: &mut Turtlebot, time: f32);

    /// Correction step of the state estimator.
    ///
    /// The correction step processes the observations. The received observations can be of every
    /// types, the implementation should not assert a specific type.
    ///
    /// ## Arguments
    /// * `turtle` -- mutable reference on the current [`Turtlebot`] to be able to interact with
    /// other modules.
    /// * `observations` -- Observation vector.
    /// * `time` -- Current time.
    fn correction_step(
        &mut self,
        turtle: &mut Turtlebot,
        observations: &Vec<Box<dyn GenericObservation>>,
        time: f32,
    );

    /// Return the current estimated state.
    fn state(&self) -> State;

    /// Return the next prediction step time. The correction step
    /// is called for each observation.
    fn next_time_step(&self) -> f32;
}
