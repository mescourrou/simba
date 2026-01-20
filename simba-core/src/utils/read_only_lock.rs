use std::{fmt::Debug, sync::{LockResult, RwLock, RwLockReadGuard}};

pub trait ReadOnlyLock<T: ?Sized + Send + Sync + Debug> : Send + Sync + Debug {
    fn read(&self) -> LockResult<RwLockReadGuard<'_, T>>;
}

impl<T: ?Sized + Send + Sync + Debug> ReadOnlyLock<T> for RwLock<T> {
    fn read(&self) -> LockResult<RwLockReadGuard<'_, T>> {
        self.read()
    }
}