extern crate confy;
use serde_derive::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Command {
    pub left_wheel_speed: f32,
    pub right_wheel_speed: f32
}


use super::perfect_physic;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum PhysicConfig {
    Perfect(Box<perfect_physic::PerfectPhysicConfig>)
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum PhysicRecord {
    Perfect(perfect_physic::PerfectPhysicRecord)
}

use crate::{state_estimators::state_estimator::State, stateful::Stateful};

pub trait Physic : std::fmt::Debug + std::marker::Send + std::marker::Sync + Stateful<PhysicRecord> {
    fn apply_command(&mut self, command: &Command, time: f32);
    fn update_state(&mut self, time: f32);
    fn state(&self, time: f32) -> &State;
}