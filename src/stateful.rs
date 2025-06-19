/*!
The Stateful module defines the [`Stateful`] trait which provides method
to save the state of a struct, and to set the struct in a given state.
*/

/// Trait providing save and resume state method.
///
/// The generic `RecordType` is the Record which is produced and taken.
/// See structs like [`RobotRecord`](crate::node_factory::RobotRecord),
/// [`StateEstimatorRecord`](crate::state_estimators::state_estimator::StateEstimatorRecord) or
/// [`PIDRecord`](crate::controllers::pid::PIDRecord).
///
/// The Record should contain the dynamic elements of the struct, to set
/// the attributes to a previous record. The static elements are not meant to
/// be in the Record struct.
///
/// Furthermore, the Record is also used to analyse the results. Save inside
/// the values you want to see in the result log.
pub trait Stateful<RecordType> {
    /// Generate the current state Record.
    fn record(&self) -> RecordType;
    /// Set the struct in the given `record` state.
    fn from_record(&mut self, record: RecordType);
}
