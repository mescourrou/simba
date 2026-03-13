//! Read-only lock trait and implementation for thread-safe access to shared data.
use std::{
    fmt::Debug,
    sync::{LockResult, RwLock, RwLockReadGuard},
};

/// Read-only lock trait for thread-safe access to shared data.
/// 
/// Wraps a `RwLock` to provide read-only access to the underlying data, ensuring thread safety while allowing multiple readers.
pub trait RoLock<T: ?Sized + Send + Sync + Debug>: Send + Sync + Debug {
    /// Acquire a read lock on the underlying data, returning a guard that allows access to the data.
    /// 
    /// Returns a `LockResult` containing a `RwLockReadGuard` if the lock was successfully acquired, or an error if the lock is poisoned.
    fn read(&self) -> LockResult<RwLockReadGuard<'_, T>>;
}

impl<T: ?Sized + Send + Sync + Debug> RoLock<T> for RwLock<T> {
    fn read(&self) -> LockResult<RwLockReadGuard<'_, T>> {
        self.read()
    }
}
