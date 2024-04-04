use crate::simulator::SimulatorMetaConfig;
use crate::state_estimators::state_estimator::StateEstimator;

use serde_json::Value;

pub trait PluginAPI {
    fn get_state_estimator(
        &self,
        config: &Value,
        meta_config: SimulatorMetaConfig,
    ) -> Box<dyn StateEstimator>;
}
