use super::navigators::navigator::{Navigator, NavigatorConfig};
use super::navigators::trajectory_follower;

use crate::controllers::controller::{Controller, ControllerConfig};
use crate::controllers::pid;

use crate::physics::physic::{Physic, PhysicConfig};
use crate::physics::perfect_physic;

use crate::state_estimators::state_estimator::{StateEstimator, StateEstimatorConfig};
use crate::state_estimators::perfect_estimator;

// Configuration for Turtlebot
extern crate confy;
use serde_derive::{Serialize, Deserialize};

use std::borrow::BorrowMut;


#[derive(Serialize, Deserialize)]
#[serde(default)]
pub struct TurtlebotConfig {
    pub name: String,
    pub navigator: NavigatorConfig,
    pub controller: ControllerConfig,
    pub physic: PhysicConfig,
    pub state_estimator: StateEstimatorConfig
    
}

impl Default for TurtlebotConfig {
    fn default() -> Self {
        TurtlebotConfig {
            name: String::from("NoName"),
            navigator: NavigatorConfig::TrajectoryFollower(Box::new(trajectory_follower::TrajectoryFollowerConfig::default())),
            controller: ControllerConfig::PID(Box::new(pid::PIDConfig::default())),
            physic: PhysicConfig::Perfect(Box::new(perfect_physic::PerfectPhysicConfig::default())),
            state_estimator: StateEstimatorConfig::Perfect(Box::new(perfect_estimator::PerfectEstimatorConfig::default()))
        }
    }
}


// Turtlebot itself

#[derive(Debug)]
pub struct Turtlebot {
    name: String,
    navigator: Box<dyn Navigator>,
    controller: Box<dyn Controller>,
    physic: Box<dyn Physic>,
    state_estimator: Box<dyn StateEstimator>
}

impl Turtlebot {
    pub fn new(name:String) -> Self {
        Self { 
            name: name,
            navigator: Box::new(trajectory_follower::TrajectoryFollower::new()),
            controller: Box::new(pid::PID::new()),
            physic: Box::new(perfect_physic::PerfectPhysic::new()),
            state_estimator: Box::new(perfect_estimator::PerfectEstimator::new()),
        }
    }

    pub fn from_config(config:&TurtlebotConfig) -> Self {
        let mut turtle = Self {
            name: config.name.clone(),
            navigator: Box::new(
                match &config.navigator {
                    NavigatorConfig::TrajectoryFollower(c) => trajectory_follower::TrajectoryFollower::from_config(c)
                }
            ),
            controller: Box::new(
                match &config.controller {
                    ControllerConfig::PID(c) => pid::PID::from_config(c)
                }
            ),
            physic: Box::new(
                match &config.physic {
                    PhysicConfig::Perfect(c) => perfect_physic::PerfectPhysic::from_config(c)
                }
            ),
            state_estimator: Box::new(
                match &config.state_estimator {
                    StateEstimatorConfig::Perfect(c) => perfect_estimator::PerfectEstimator::from_config(c)
                }
            )
        };
        turtle
    }
    
}
