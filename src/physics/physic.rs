#[derive(Debug)]
pub struct Command {
    pub left_wheel_speed: f32,
    pub right_wheel_speed: f32
}

extern crate confy;
use serde_derive::{Serialize, Deserialize};

use super::perfect_physic;

#[derive(Serialize, Deserialize, Debug)]
pub enum PhysicConfig {
    Perfect(Box<perfect_physic::PerfectPhysicConfig>)
}

use crate::state_estimators::state_estimator::State;

pub trait Physic : std::fmt::Debug {
    fn apply_command(&mut self, command: &Command, time: f32);
    fn state(&self) -> &State;
}