/*!
Module providing different strategies for the state estimation.

To use an external state estimator (in Rust), use [`external_estimator`]
and implement a specification for [`PluginAPI`](crate::plugin_api::PluginAPI).

## How to create a new (internal) state estimation strategy
To create a new state estimation strategy, here are the required steps.

1) **Make a configuration**
Your new strategy should define a Config struct, such as
[`PerfectEstimator`](crate::state_estimators::perfect_estimator::PerfectEstimator),
with Serialize, Deserialize, Debug, Clone and Default traits.  Add this new Config to
[`StateEstimatorConfig`](state_estimator::StateEstimatorConfig) enumeration, and to the match
patterns in [`make_state_estimator_from_config`](state_estimator::make_state_estimator_from_config)
so that the right strategy is created.

2) **Make a new Record**
Your new strategy should define a Record struct, such as
[`PerfectEstimator`](crate::state_estimators::perfect_estimator::PerfectEstimator),
with Serialize, Deserialize, Debug and Clone traits.

3) **Implement the Trait**
Implement the trait methods.

4) **Implement the Stateful trait**
Don't forget to implement the [`Stateful`](crate::stateful::Stateful) trait, with the newly created
Record struct as generic type.
*/

pub mod external_estimator;
pub mod perfect_estimator;
pub mod state_estimator;
pub mod pybinds;
