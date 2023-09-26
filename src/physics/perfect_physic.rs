// Configuration for PerfectPhysic
use serde_derive::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Debug)]
#[serde(default)]
pub struct PerfectPhysicConfig {
    wheel_distance: f32
}

impl Default for PerfectPhysicConfig {
    fn default() -> Self {
        Self {
            wheel_distance: 0.25
        }
    }
}


use crate::state_estimators::state_estimator::State;

#[derive(Debug)]
pub struct PerfectPhysic {
    wheel_distance: f32,
    state: State,
    last_time_update: f32
}

impl PerfectPhysic {
    pub fn new() -> Self {
        Self::from_config(&PerfectPhysicConfig::default())
    }

    pub fn from_config(config: &PerfectPhysicConfig) -> Self {
        PerfectPhysic {
            wheel_distance: config.wheel_distance,
            state: State::new(),
            last_time_update: 0.
        }
    }
}

use super::physic::Command;
use super::physic::Physic;

impl Physic for PerfectPhysic {
    fn apply_command(&mut self, command: &Command, time: f32) {
        let dt = time - self.last_time_update;
        assert!(dt > 0., "PID delta time should be positive: {} - {} = {} > 0", time, self.last_time_update, dt);
        
        let theta = self.state.pose.z;
        let x = self.state.pose.x;
        let y = self.state.pose.y;

        let displacement_wheel_left = command.left_wheel_speed * dt;
        let displacement_wheel_right = command.right_wheel_speed * dt;

        let translation = (displacement_wheel_left + displacement_wheel_right) / 2.;
        let rotation = (displacement_wheel_right - displacement_wheel_left) / self.wheel_distance;

        self.last_time_update = time;

        self.state.pose.x += translation * (theta + rotation / 2.).cos();
        self.state.pose.y += translation * (theta + rotation / 2.).sin();
        self.state.pose.z += rotation;
        
    }

    fn state(&self) -> &State {
        &self.state
    }
}