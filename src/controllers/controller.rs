use crate::physics::physic::Command;

extern crate confy;
use serde_derive::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ControllerError {
    pub lateral: f32,
    pub theta: f32,
    pub velocity: f32
}

use super::pid;


#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum ControllerConfig {
    PID(Box<pid::PIDConfig>)
}

#[derive(Serialize, Deserialize, Debug)]
pub enum ControllerRecord {
    PID(pid::PIDRecord)
}

use crate::turtlebot::Turtlebot;

pub trait Controller : std::fmt::Debug + std::marker::Send + std::marker::Sync {
    fn make_command(&mut self, turtle: &mut Turtlebot, error: &ControllerError, time: f32) -> Command;
    fn record(&self) -> ControllerRecord;
}