//! Defines the [`FaultModel`] trait for simulating sensor faults in the Simba framework.

use std::{fmt::Debug, sync::Arc};

use crate::{environment::Environment, errors::SimbaResult, sensors::SensorObservation};

/// Trait defining the interface for sensor fault models in Simba.
///
/// This trait is used for user defined fault models that can be applied to sensor observations to simulate faults or noise.
/// Otherwise, fault models are managed without trait using Enum variants in each sensors.
///
pub trait FaultModel: Debug + Sync + Send {
    /// Post-initialization method called after the fault model is created and before the simulation starts.
    #[allow(unused_variables)]
    fn post_init(&mut self, node: &mut crate::node::Node, initial_time: f32) -> SimbaResult<()> {
        Ok(())
    }

    /// Add faults to the given sensor observations based on the current time, a seed for randomness, and the environment context
    ///
    /// The method modifies the `obs_list` in place by adding new observations and modifying or removing existing ones to simulate faults.
    ///
    /// # Arguments
    /// * `time` - Current simulation time used for time-based fault behavior.
    /// * `seed` - A float seed for any randomness in the fault model, ensuring reproducibility. The seed is based on time but can vary if multiple faults are applied at the same time step.
    /// * `obs_list` - A mutable reference to the list of sensor observations to which faults should be applied. The method can modify existing observations or add new ones to simulate faults.
    /// * `obs_type` - The type of sensor observation being processed, which can be used to apply different faults based on the sensor type.
    /// * `environment` - An Arc to the [`Environment`] providing context about the simulation environment (map, other nodes), which can be used to create context-aware faults.
    fn add_faults(
        &mut self,
        time: f32,
        seed: f32,
        obs_list: &mut Vec<SensorObservation>,
        obs_type: SensorObservation,
        environment: &Arc<Environment>,
    );
}
