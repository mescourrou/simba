use crate::state_estimators::state_estimator::StateEstimator;

pub trait PluginAPI {
    fn get_state_estimator(&self) -> Box<dyn StateEstimator>;
}