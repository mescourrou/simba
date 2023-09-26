extern crate nalgebra as na;
use na::SVector;
use crate::physics::physic::Command;

#[derive(Debug)]
pub struct ControllerError {
    pub lateral: f32,
    pub theta: f32,
    pub velocity: f32
}

use super::pid;

extern crate confy;
use serde_derive::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Debug)]
pub enum ControllerConfig {
    PID(Box<pid::PIDConfig>)
}


pub trait Controller : std::fmt::Debug {
    fn make_command(&mut self, error: &ControllerError, time: f32) -> Command;
}