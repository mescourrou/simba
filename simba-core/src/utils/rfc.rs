use std::sync::{Arc, Mutex, RwLock, mpsc};

use crate::utils::{SharedMutex, SharedRwLock};

#[derive(Debug, Clone)]
pub struct RemoteFunctionCall<ParamType, ReturnType> {
    sender: mpsc::Sender<ParamType>,
    receiver: SharedMutex<mpsc::Receiver<ReturnType>>,
}

impl<ParamType, ReturnType> RemoteFunctionCall<ParamType, ReturnType> {
    pub fn call(&self, param: ParamType) -> Option<ReturnType> {
        if self.sender.send(param).is_err() {
            return None;
        }
        self.receiver.lock().unwrap().recv().ok()
    }

    pub fn async_call(&self, param: ParamType) {
        let _ = self.sender.send(param);
    }

    /// Blocking
    pub fn wait_result(&self) -> Option<ReturnType> {
        self.receiver.lock().unwrap().recv().ok()
    }

    /// Non Blocking
    pub fn try_get_result(&self) -> Option<ReturnType> {
        self.receiver.lock().unwrap().try_recv().ok()
    }
}

#[derive(Debug, Clone)]
pub struct RemoteFunctionCallHost<ParamType, ReturnType> {
    receiver: SharedMutex<mpsc::Receiver<ParamType>>,
    stop_sender: mpsc::Sender<ParamType>,
    sender: mpsc::Sender<ReturnType>,
    stopping: SharedRwLock<bool>,
}

impl<ParamType, ReturnType> RemoteFunctionCallHost<ParamType, ReturnType> {
    pub fn try_recv_fn(&self, exe: fn(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().try_recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            let _ = self.sender.send(result);
        }
    }

    pub fn try_recv_closure(&self, exe: impl Fn(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().try_recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            let _ = self.sender.send(result);
        }
    }

    pub fn try_recv_closure_mut(&self, mut exe: impl FnMut(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().try_recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            let _ = self.sender.send(result);
        }
    }

    pub fn recv_fn(&self, exe: fn(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            let _ = self.sender.send(result);
        }
    }

    pub fn recv_closure(&self, exe: impl Fn(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            let _ = self.sender.send(result);
        }
    }

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
    pub fn stop_recv(&self) {
        *self.stopping.write().unwrap() = true;
        self.stop_sender.send(ParamType::default()).unwrap();
    }
}

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
