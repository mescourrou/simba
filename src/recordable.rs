/*!
The Recordable module defines the [`Recordable`] trait which provides method
to save the state of a struct to be analysed afterward (or during with GUI).
*/

/// Trait providing save state method.
///
/// The generic `RecordType` is the Record which is produced.
/// See structs like [`RobotRecord`](crate::node_factory::RobotRecord),
/// [`StateEstimatorRecord`](crate::state_estimators::state_estimator::StateEstimatorRecord) or
/// [`PIDRecord`](crate::controllers::pid::PIDRecord).
///
/// The Record should contain the elements worth to be used in the result
/// analysis.
pub trait Recordable<RecordType> {
    /// Generate the current state Record.
    fn record(&self) -> RecordType;
}
