//! Minimal in-process remote-function-call primitives based on channels.
//!
//! This module provides a pair of endpoint types:
//! - [`RemoteFunctionCall`], used by the caller side to send parameters and receive results;
//! - [`RemoteFunctionCallHost`], used by the host side to receive parameters, execute work, and
//!   send results back.
//!
//! Use [`make_pair`] to create a connected caller/host pair.

use std::sync::{Arc, Mutex, RwLock, mpsc};

#[derive(Debug, Clone)]
/// Caller endpoint for a typed remote function call.
///
/// Each call sends a parameter of type `ParamType` and receives a result of type `ReturnType`.
pub struct RemoteFunctionCall<ParamType, ReturnType> {
    sender: mpsc::Sender<ParamType>,
    receiver: Arc<Mutex<mpsc::Receiver<ReturnType>>>,
}

impl<ParamType, ReturnType> RemoteFunctionCall<ParamType, ReturnType> {
    /// Sends a request and waits for a result.
    ///
    /// Returns `None` if sending fails or if the result channel is closed.
    pub fn call(&self, param: ParamType) -> Option<ReturnType> {
        if self.sender.send(param).is_err() {
            return None;
        }
        self.receiver.lock().unwrap().recv().ok()
    }

    /// Sends a request without waiting for the result.
    pub fn async_call(&self, param: ParamType) {
        let _ = self.sender.send(param);
    }

    /// Blocks until a result is received.
    ///
    /// Returns `None` if the result channel is closed.
    pub fn wait_result(&self) -> Option<ReturnType> {
        self.receiver.lock().unwrap().recv().ok()
    }

    /// Tries to retrieve a result without blocking.
    pub fn try_get_result(&self) -> Option<ReturnType> {
        self.receiver.lock().unwrap().try_recv().ok()
    }
}

#[derive(Debug, Clone)]
/// Host endpoint for a typed remote function call.
///
/// The host receives parameters, executes user-provided logic, and sends computed results back.
pub struct RemoteFunctionCallHost<ParamType, ReturnType> {
    receiver: Arc<Mutex<mpsc::Receiver<ParamType>>>,
    stop_sender: mpsc::Sender<ParamType>,
    sender: mpsc::Sender<ReturnType>,
    stopping: Arc<RwLock<bool>>,
}

impl<ParamType, ReturnType> RemoteFunctionCallHost<ParamType, ReturnType> {
    /// Tries to receive one request and executes a function pointer if available.
    pub fn try_recv_fn(&self, exe: fn(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().try_recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            let _ = self.sender.send(result);
        }
    }

    /// Tries to receive one request and executes a non-mutable closure if available.
    pub fn try_recv_closure(&self, exe: impl Fn(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().try_recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            let _ = self.sender.send(result);
        }
    }

    /// Tries to receive one request and executes a mutable closure if available.
    pub fn try_recv_closure_mut(&self, mut exe: impl FnMut(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().try_recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            let _ = self.sender.send(result);
        }
    }

    /// Blocks until one request is received, then executes a function pointer.
    pub fn recv_fn(&self, exe: fn(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            let _ = self.sender.send(result);
        }
    }

    /// Blocks until one request is received, then executes a non-mutable closure.
    pub fn recv_closure(&self, exe: impl Fn(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            let _ = self.sender.send(result);
        }
    }

    /// Blocks until one request is received, then executes a mutable closure.
    pub fn recv_closure_mut(&self, mut exe: impl FnMut(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            let _ = self.sender.send(result);
        }
    }
}

impl<ParamType: Default, ReturnType> RemoteFunctionCallHost<ParamType, ReturnType> {
    /// Requests the host loop to stop by setting the stop flag and sending a wake-up value.
    pub fn stop_recv(&self) {
        *self.stopping.write().unwrap() = true;
        self.stop_sender.send(ParamType::default()).unwrap();
    }
}

/// Creates a connected caller/host pair.
///
/// The returned tuple contains `(caller, host)` endpoints sharing request and response channels.
pub fn make_pair<ParamType, ReturnType>() -> (
    RemoteFunctionCall<ParamType, ReturnType>,
    RemoteFunctionCallHost<ParamType, ReturnType>,
) {
    let call_pipe = mpsc::channel();
    let return_pipe = mpsc::channel();

    (
        RemoteFunctionCall {
            sender: call_pipe.0.clone(),
            receiver: Arc::new(Mutex::new(return_pipe.1)),
        },
        RemoteFunctionCallHost {
            sender: return_pipe.0,
            stop_sender: call_pipe.0,
            receiver: Arc::new(Mutex::new(call_pipe.1)),
            stopping: Arc::new(RwLock::new(false)),
        },
    )
}
