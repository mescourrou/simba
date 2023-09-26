extern crate nalgebra as na;
use na::{SVector};

#[derive(Debug)]
pub struct State {
    pub pose: SVector<f32, 3>,
    pub velocity: f32
}

impl State {
    pub fn new() -> Self {
        Self {
            pose: SVector::<f32,3>::new(0., 0., 0.),
            velocity: 0.
        }
    }
}

extern crate confy;
use serde_derive::{Serialize, Deserialize};

use super::perfect_estimator;

#[derive(Serialize, Deserialize, Debug)]
pub enum StateEstimatorConfig {
    Perfect(Box<perfect_estimator::PerfectEstimatorConfig>)
}

pub trait StateEstimator : std::fmt::Debug {

}