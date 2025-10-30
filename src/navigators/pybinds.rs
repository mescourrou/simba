use std::{
    str::FromStr,
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc, Mutex,
    },
};

use log::debug;
use pyo3::prelude::*;
use serde_json::Value;

use crate::{
    controllers::controller::ControllerError,
    logger::is_enabled,
    navigators::external_navigator::ExternalNavigatorRecord,
    networking::message_handler::MessageHandler,
    node::Node,
    pywrappers::{ControllerErrorWrapper, NodeWrapper, WorldStateWrapper},
    recordable::Recordable,
    state_estimators::state_estimator::WorldState,
};

use super::navigator::{Navigator, NavigatorRecord};

#[derive(Debug, Clone)]
pub struct PythonNavigatorAsyncClient {
    pub compute_error_request: mpsc::Sender<(NodeWrapper, WorldState)>,
    pub compute_error_response: Arc<Mutex<mpsc::Receiver<ControllerError>>>,
    pub record_request: mpsc::Sender<()>,
    pub record_response: Arc<Mutex<mpsc::Receiver<NavigatorRecord>>>,
    pub pre_loop_hook_request: mpsc::Sender<(NodeWrapper, f32)>,
    pub pre_loop_hook_response: Arc<Mutex<mpsc::Receiver<()>>>,
    received_msgs: Vec<(String, String, f32)>,
    letter_box_receiver: Arc<Mutex<Receiver<(String, Value, f32)>>>,
    letter_box_sender: Sender<(String, Value, f32)>,
}

impl PythonNavigatorAsyncClient {
    fn update_messages(&mut self) {
        while let Ok((from, msg, time)) = self.letter_box_receiver.lock().unwrap().try_recv() {
            let msg = serde_json::to_string(&msg).unwrap();
            self.received_msgs.push((from, msg, time));
        }
    }
}

impl Navigator for PythonNavigatorAsyncClient {
    fn compute_error(&mut self, node: &mut Node, world_state: WorldState) -> ControllerError {
        self.update_messages();
        let node_py = NodeWrapper::from_rust(&node, self.received_msgs.clone());
        self.compute_error_request
            .send((node_py, world_state.clone()))
            .unwrap();
        self.compute_error_response.lock().unwrap().recv().unwrap()
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        self.received_msgs.clear();
        self.update_messages();
        let node_py = NodeWrapper::from_rust(&node, self.received_msgs.clone());
        self.pre_loop_hook_request.send((node_py, time)).unwrap();
        self.pre_loop_hook_response.lock().unwrap().recv().unwrap()
    }
}

impl Recordable<NavigatorRecord> for PythonNavigatorAsyncClient {
    fn record(&self) -> NavigatorRecord {
        self.record_request.send(()).unwrap();
        self.record_response
            .lock()
            .unwrap()
            .recv()
            .expect("Error during call of record")
    }
}

impl MessageHandler for PythonNavigatorAsyncClient {
    fn get_letter_box(&self) -> Option<Sender<(String, Value, f32)>> {
        Some(self.letter_box_sender.clone())
    }
}

#[derive(Debug)]
#[pyclass(subclass)]
#[pyo3(name = "Navigator")]
pub struct PythonNavigator {
    model: Py<PyAny>,
    client: PythonNavigatorAsyncClient,
    compute_error_request: Arc<Mutex<mpsc::Receiver<(NodeWrapper, WorldState)>>>,
    compute_error_response: mpsc::Sender<ControllerError>,
    record_request: Arc<Mutex<mpsc::Receiver<()>>>,
    record_response: mpsc::Sender<NavigatorRecord>,
    pre_loop_hook_request: Arc<Mutex<Receiver<(NodeWrapper, f32)>>>,
    pre_loop_hook_response: Sender<()>,
}

#[pymethods]
impl PythonNavigator {
    #[new]
    pub fn new(py_model: Py<PyAny>) -> PythonNavigator {
        if is_enabled(crate::logger::InternalLog::API) {
            Python::with_gil(|py| {
                debug!("Model got: {}", py_model.bind(py).dir().unwrap());
            });
        }
        let (compute_error_request_tx, compute_error_request_rx) = mpsc::channel();
        let (compute_error_response_tx, compute_error_response_rx) = mpsc::channel();
        let (record_request_tx, record_request_rx) = mpsc::channel();
        let (record_response_tx, record_response_rx) = mpsc::channel();
        let (pre_loop_hook_request_tx, pre_loop_hook_request_rx) = mpsc::channel();
        let (pre_loop_hook_response_tx, pre_loop_hook_response_rx) = mpsc::channel();
        let (letter_box_sender, letter_box_receiver) = mpsc::channel();

        PythonNavigator {
            model: py_model,
            client: PythonNavigatorAsyncClient {
                compute_error_request: compute_error_request_tx,
                compute_error_response: Arc::new(Mutex::new(compute_error_response_rx)),
                record_request: record_request_tx,
                record_response: Arc::new(Mutex::new(record_response_rx)),
                pre_loop_hook_request: pre_loop_hook_request_tx,
                pre_loop_hook_response: Arc::new(Mutex::new(pre_loop_hook_response_rx)),
                letter_box_receiver: Arc::new(Mutex::new(letter_box_receiver)),
                letter_box_sender: letter_box_sender,
                received_msgs: Vec::new(),
            },
            compute_error_request: Arc::new(Mutex::new(compute_error_request_rx)),
            compute_error_response: compute_error_response_tx,
            record_request: Arc::new(Mutex::new(record_request_rx)),
            record_response: record_response_tx,
            pre_loop_hook_request: Arc::new(Mutex::new(pre_loop_hook_request_rx)),
            pre_loop_hook_response: pre_loop_hook_response_tx,
        }
    }
}

impl PythonNavigator {
    pub fn get_client(&self) -> PythonNavigatorAsyncClient {
        self.client.clone()
    }

    pub fn check_requests(&mut self) {
        if let Ok((node, state)) = self
            .compute_error_request
            .clone()
            .lock()
            .unwrap()
            .try_recv()
        {
            let error = self.compute_error(node, &state);
            self.compute_error_response.send(error).unwrap();
        }
        if let Ok(()) = self.record_request.clone().lock().unwrap().try_recv() {
            self.record_response.send(self.record()).unwrap();
        }
        if let Ok((node, time)) = self
            .pre_loop_hook_request
            .clone()
            .lock()
            .unwrap()
            .try_recv()
        {
            self.pre_loop_hook(node, time);
            self.pre_loop_hook_response.send(()).unwrap();
        }
    }

    fn compute_error(&mut self, node: NodeWrapper, state: &WorldState) -> ControllerError {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of compute_error");
        }
        // let node_record = node.record();
        let result = Python::with_gil(|py| -> ControllerErrorWrapper {
            match self.model.bind(py).call_method(
                "compute_error",
                (node, WorldStateWrapper::from_rust(state)),
                None,
            ) {
                Err(e) => {
                    e.display(py);
                    panic!("Error while calling 'compute_error' method of PythonNavigator.");
                }
                Ok(r) => r
                    .extract()
                    .expect("Error during the call of Python implementation of 'compute_error'"),
            }
        });
        result.to_rust()
    }

    fn record(&self) -> NavigatorRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = Python::with_gil(|py| {
            match self.model
                .bind(py)
                .call_method("record", (), None) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error while calling 'record' method of PythonNavigator.");
                    }
                    Ok(r) => {
                        r.extract()
                        .expect("The 'record' method of PythonNavigator does not return a valid EstimatorRecord type")
                    }
                }
        });
        let record = ExternalNavigatorRecord {
            record: Value::from_str(record_str.as_str()).expect(
                "Impossible to get serde_json::Value from the input serialized python structure",
            ),
        };
        // record.clone()
        // StateEstimatorRecord::External(PythonNavigator::record(&self))
        NavigatorRecord::External(record)
    }

    fn pre_loop_hook(&mut self, node: NodeWrapper, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of pre_loop_hook");
        }
        Python::with_gil(|py: Python<'_>| {
            match self
                .model
                .bind(py)
                .call_method("pre_loop_hook", (node, time), None)
            {
                Err(e) => {
                    e.display(py);
                    panic!("Error while calling 'next_time_step' method of PythonNavigator.");
                }
                Ok(_) => {}
            }
        });
    }
}
