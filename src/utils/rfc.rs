use std::sync::{mpsc, Arc, Mutex, RwLock};

use crate::errors::SimbaResult;

#[derive(Clone)]
pub struct RemoteFunctionCall<ParamType: Clone, ReturnType: Clone> {
    sender: mpsc::Sender<ParamType>,
    receiver: Arc<Mutex<mpsc::Receiver<ReturnType>>>,
}

impl<ParamType: Clone, ReturnType: Clone> RemoteFunctionCall<ParamType, ReturnType> {
    pub fn call(&self, param: ParamType) -> SimbaResult<ReturnType> {
        self.sender.send(param).unwrap();
        let result = self.receiver.lock().unwrap().recv().unwrap();
        Ok(result)
    }

    pub fn async_call(&self, param: ParamType) {
        self.sender.send(param).unwrap();
    }

    /// Blocking
    pub fn wait_result(&self) -> SimbaResult<ReturnType> {
        let result = self.receiver.lock().unwrap().recv().unwrap();
        Ok(result)
    }

    /// Non Blocking
    pub fn try_get_result(&self) -> Option<SimbaResult<ReturnType>> {
        if let Ok(result) = self.receiver.lock().unwrap().try_recv() {
            Some(Ok(result))
        } else {
            None
        }
    }
}

#[derive(Clone)]
pub struct RemoteFunctionCallHost<ParamType: Clone, ReturnType: Clone> {
    receiver: Arc<Mutex<mpsc::Receiver<ParamType>>>,
    stop_sender: mpsc::Sender<ParamType>,
    sender: mpsc::Sender<ReturnType>,
    stopping: Arc<RwLock<bool>>,
}

impl<ParamType: Clone, ReturnType: Clone> RemoteFunctionCallHost<ParamType, ReturnType> {
    pub fn try_recv_fn(&self, exe: fn(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().try_recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            self.sender.send(result).unwrap();
        }
    }

    pub fn try_recv_closure(&self, exe: impl Fn(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().try_recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            self.sender.send(result).unwrap();
        }
    }

    pub fn try_recv_closure_mut(&self, mut exe: impl FnMut(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().try_recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            self.sender.send(result).unwrap();
        }
    }

    pub fn recv_fn(&self, exe: fn(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            self.sender.send(result).unwrap();
        }
    }

    pub fn recv_closure(&self, exe: impl Fn(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            self.sender.send(result).unwrap();
        }
    }

    pub fn recv_closure_mut(&self, mut exe: impl FnMut(ParamType) -> ReturnType) {
        if let Ok(param) = self.receiver.lock().unwrap().recv() {
            if *self.stopping.read().unwrap() {
                return;
            }
            let result = exe(param);
            self.sender.send(result).unwrap();
        }
    }
}

impl<ParamType: Clone + Default, ReturnType: Clone> RemoteFunctionCallHost<ParamType, ReturnType> {
    pub fn stop_recv(&self) {
        *self.stopping.write().unwrap() = true;
        self.stop_sender.send(ParamType::default()).unwrap();
    }
}

pub fn make_pair<ParamType: Clone, ReturnType: Clone>() -> (
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
