// Configuration for PerfectPhysic
use serde_derive::{Serialize, Deserialize};
use crate::state_estimators::state_estimator::{State, StateConfig, StateRecord};

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct PerfectPhysicConfig {
    wheel_distance: f32,
    initial_state: StateConfig
}

impl Default for PerfectPhysicConfig {
    fn default() -> Self {
        Self {
            wheel_distance: 0.25,
            initial_state: StateConfig::default()
        }
    }
}


#[derive(Serialize, Deserialize, Debug)]
pub struct PerfectPhysicRecord {
    state: StateRecord
}



#[derive(Debug)]
pub struct PerfectPhysic {
    wheel_distance: f32,
    state: State,
    last_time_update: f32,
    current_command: Command
}

impl PerfectPhysic {
    pub fn new() -> Self {
        Self::from_config(&PerfectPhysicConfig::default())
    }

    pub fn from_config(config: &PerfectPhysicConfig) -> Self {
        PerfectPhysic {
            wheel_distance: config.wheel_distance,
            state: State::from_config(&config.initial_state),
            last_time_update: 0.,
            current_command: Command {
                left_wheel_speed: 0.,
                right_wheel_speed: 0.
            }
        }
    }

    fn compute_state_until(&mut self, time: f32) {
        let dt = time - self.last_time_update;
        assert!(dt > 0., "PID delta time should be positive: {} - {} = {} > 0", time, self.last_time_update, dt);
        
        let theta = self.state.pose.z;

        let displacement_wheel_left = self.current_command.left_wheel_speed * dt;
        let displacement_wheel_right = self.current_command.right_wheel_speed * dt;

        let translation = (displacement_wheel_left + displacement_wheel_right) / 2.;
        let rotation = (displacement_wheel_right - displacement_wheel_left) / self.wheel_distance;

        self.last_time_update = time;

        self.state.pose.x += translation * (theta + rotation / 2.).cos();
        self.state.pose.y += translation * (theta + rotation / 2.).sin();
        self.state.pose.z += rotation;

        self.state.velocity = translation / dt;
    }
}

use super::physic::Command;
use super::physic::{Physic, PhysicRecord};

impl Physic for PerfectPhysic {
    fn apply_command(&mut self, command: &Command, _time: f32) {
        self.current_command = command.clone();
    }

    fn update_state(&mut self, time: f32) {
        self.compute_state_until(time);
    }

    fn state(&self, _time: f32) -> &State {
        &self.state
    }

    fn record(&self) -> PhysicRecord {
        PhysicRecord::Perfect(PerfectPhysicRecord {
            state: self.state.record()
        })
    }
}