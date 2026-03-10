//! TODO: Lots of code duplication between fault models, need to find a way to factorize

use std::{fmt::Debug, sync::Arc};

use crate::{environment::Environment, errors::SimbaResult, sensors::SensorObservation};

pub trait FaultModel: Debug + Sync + Send {
    fn post_init(&mut self, _node: &mut crate::node::Node, _initial_time: f32) -> SimbaResult<()> {
        Ok(())
    }

    fn add_faults(
        &mut self,
        time: f32,
        seed: f32,
        obs_list: &mut Vec<SensorObservation>,
        obs_type: SensorObservation,
        environment: &Arc<Environment>,
    );
}
