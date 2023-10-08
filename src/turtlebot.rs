use super::navigators::navigator::{Navigator, NavigatorConfig, NavigatorRecord};
use super::navigators::trajectory_follower;

use crate::controllers::controller::{Controller, ControllerConfig, ControllerRecord};
use crate::controllers::pid;

use crate::physics::physic::{Physic, PhysicConfig, PhysicRecord};
use crate::physics::perfect_physic;

use crate::state_estimators::state_estimator::{StateEstimator, StateEstimatorConfig, StateEstimatorRecord};
use crate::state_estimators::perfect_estimator;

// Configuration for Turtlebot
extern crate confy;
use serde_derive::{Serialize, Deserialize};


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


#[derive(Serialize, Deserialize)]
pub struct TurtlebotRecord {
    pub name: String,
    pub navigator: NavigatorRecord,
    pub controller: ControllerRecord,
    pub physic: PhysicRecord,
    pub state_estimator: StateEstimatorRecord
    
}


// Turtlebot itself

#[derive(Debug)]
pub struct Turtlebot {
    name: String,
    navigator: Box<dyn Navigator>,
    controller: Box<dyn Controller>,
    physic: Box<dyn Physic>,
    state_estimator: Box<dyn StateEstimator>,
    next_time_step: f32
}

impl Turtlebot {
    pub fn new(name:String) -> Self {
        Self { 
            name: name,
            navigator: Box::new(trajectory_follower::TrajectoryFollower::new()),
            controller: Box::new(pid::PID::new()),
            physic: Box::new(perfect_physic::PerfectPhysic::new()),
            state_estimator: Box::new(perfect_estimator::PerfectEstimator::new()),
            next_time_step: 0.
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
            ),
            next_time_step: 0.
        };
        turtle.next_time_step = turtle.state_estimator.next_time_step();
        turtle
    }
    
    pub fn run_next_time_step(&mut self, time: f32) -> f32 {
        if time < self.next_time_step {
            return self.next_time_step;
        }
        println!("Run time {}", time);
        self.physic.update_state(time);
        self.state_estimator.update_estimation(time, self.physic.as_ref());
        let state = self.state_estimator.state();
        println!("State: {:?}", state);
        let error = self.navigator.compute_error(state);
        println!("Error: {:?}", error);
        let command = self.controller.make_command(&error, time);
        println!("Command: {:?}", command);
        self.physic.apply_command(&command, time);

        println!("{}: {}", time, state);
        self.next_time_step = self.state_estimator.next_time_step();
        self.next_time_step
    }

    pub fn next_time_step(&self) -> f32 {
        self.next_time_step
    }

    pub fn record(&self) -> TurtlebotRecord {
        TurtlebotRecord {
            name: self.name.clone(),
            navigator: self.navigator.record(),
            controller: self.controller.record(),
            physic: self.physic.record(),
            state_estimator: self.state_estimator.record(),
        }
    }
}
