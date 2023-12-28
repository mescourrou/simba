use crate::state_estimators::state_estimator::StateEstimator;

use serde_json::Value;
use std::collections::BTreeMap as Map;

pub type ConfigMap = Map<String, Value>;

pub trait PluginAPI {
    fn get_state_estimator(&self, config: &ConfigMap) -> Box<dyn StateEstimator>;
}