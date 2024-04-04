pub trait Stateful<RecordType> {
    fn record(&self) -> RecordType;
    fn from_record(&mut self, record: RecordType);
}
