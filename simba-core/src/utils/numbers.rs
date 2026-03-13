//! Utilities for working with numbers in Simba, including ordered floating-point types and periodicity configurations.

/// Wrapper around `f32` that implements `Ord` and `Eq` by treating NaN values as equal and ordering them in a consistent way.
/// Useful for using floating-point numbers in ordered collections like `BTreeSet` or as keys in `BTreeMap`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct OrderedF32(pub f32);

impl Eq for OrderedF32 {}

impl PartialOrd for OrderedF32 {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for OrderedF32 {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0
            .partial_cmp(&other.0)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}
