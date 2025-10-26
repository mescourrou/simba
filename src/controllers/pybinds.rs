use std::{
    str::FromStr,
    sync::{mpsc::{self, Receiver, Sender}, Arc, Mutex},
};

use log::debug;
use pyo3::prelude::*;
use serde_json::Value;

use crate::{
    controllers::external_controller::ExternalControllerRecord, logger::is_enabled, networking::message_handler::MessageHandler, node::Node, physics::physics::Command, pywrappers::{CommandWrapper, ControllerErrorWrapper, NodeWrapper}, recordable::Recordable
};

use super::controller::{Controller, ControllerError, ControllerRecord};

#[derive(Debug, Clone)]
pub struct PythonControllerAsyncClient {
    pub make_command_request: mpsc::Sender<(NodeWrapper, ControllerError, f32)>,
    pub make_command_response: Arc<Mutex<mpsc::Receiver<Command>>>,
    pub record_request: mpsc::Sender<()>,
    pub record_response: Arc<Mutex<mpsc::Receiver<ControllerRecord>>>,
    pub pre_loop_hook_request: mpsc::Sender<(NodeWrapper, f32)>,
    pub pre_loop_hook_response: Arc<Mutex<mpsc::Receiver<()>>>,
    received_msgs: Vec<(String, String, f32)>,
    letter_box_receiver: Arc<Mutex<Receiver<(String, Value, f32)>>>,
    letter_box_sender: Sender<(String, Value, f32)>,
}

impl PythonControllerAsyncClient {
    fn update_messages(&mut self) {
        while let  Ok((from, msg, time)) = self.letter_box_receiver.lock().unwrap().try_recv() {
            let msg = serde_json::to_string(&msg).unwrap();
            self.received_msgs.push((from, msg, time));
        }
    }
}

impl Controller for PythonControllerAsyncClient {
    fn make_command(&mut self, node: &mut Node, error: &ControllerError, time: f32) -> Command {
        self.update_messages();
        let node_py = NodeWrapper::from_rust(&node, self.received_msgs.clone());
        self.make_command_request
            .send((node_py, error.clone(), time))
            .unwrap();
        self.make_command_response.lock().unwrap().recv().unwrap()
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        self.received_msgs.clear();
        self.update_messages();
        let node_py = NodeWrapper::from_rust(&node, self.received_msgs.clone());
        self.pre_loop_hook_request
            .send((node_py, time))
            .unwrap();
        self.pre_loop_hook_response.lock().unwrap().recv().unwrap()
    }
}

impl Recordable<ControllerRecord> for PythonControllerAsyncClient {
    fn record(&self) -> ControllerRecord {
        self.record_request.send(()).unwrap();
        self.record_response
            .lock()
            .unwrap()
            .recv()
            .expect("Error during call of record")
    }
}

impl MessageHandler for PythonControllerAsyncClient {
    fn get_letter_box(&self) -> Option<Sender<(String, Value, f32)>> {
        Some(self.letter_box_sender.clone())
    }
}

#[derive(Debug)]
#[pyclass(subclass)]
#[pyo3(name = "Controller")]
pub struct PythonController {
    model: Py<PyAny>,
    client: PythonControllerAsyncClient,
    make_command_request: Arc<Mutex<mpsc::Receiver<(NodeWrapper, ControllerError, f32)>>>,
    make_command_response: mpsc::Sender<Command>,
    record_request: Arc<Mutex<mpsc::Receiver<()>>>,
    record_response: mpsc::Sender<ControllerRecord>,
    pre_loop_hook_request: Arc<Mutex<Receiver<(NodeWrapper, f32)>>>,
    pre_loop_hook_response: Sender<()>,
}

#[pymethods]
impl PythonController {
    #[new]
    pub fn new(py_model: Py<PyAny>) -> PythonController {
        if is_enabled(crate::logger::InternalLog::API) {
            Python::with_gil(|py| {
                debug!("Model got: {}", py_model.bind(py).dir().unwrap());
            });
        }
        let (make_command_request_tx, make_command_request_rx) = mpsc::channel();
        let (make_command_response_tx, make_command_response_rx) = mpsc::channel();
        let (record_request_tx, record_request_rx) = mpsc::channel();
        let (record_response_tx, record_response_rx) = mpsc::channel();
        let (pre_loop_hook_request_tx, pre_loop_hook_request_rx) = mpsc::channel();
        let (pre_loop_hook_response_tx, pre_loop_hook_response_rx) = mpsc::channel();
        let (letter_box_sender, letter_box_receiver) = mpsc::channel();

        PythonController {
            model: py_model,
            client: PythonControllerAsyncClient {
                make_command_request: make_command_request_tx,
                make_command_response: Arc::new(Mutex::new(make_command_response_rx)),
                record_request: record_request_tx,
                record_response: Arc::new(Mutex::new(record_response_rx)),
                pre_loop_hook_request: pre_loop_hook_request_tx,
                pre_loop_hook_response: Arc::new(Mutex::new(pre_loop_hook_response_rx)),
                letter_box_receiver: Arc::new(Mutex::new(letter_box_receiver)),
                letter_box_sender,
                received_msgs: Vec::new(),
            },
            make_command_request: Arc::new(Mutex::new(make_command_request_rx)),
            make_command_response: make_command_response_tx,
            record_request: Arc::new(Mutex::new(record_request_rx)),
            record_response: record_response_tx,
            pre_loop_hook_request: Arc::new(Mutex::new(pre_loop_hook_request_rx)),
            pre_loop_hook_response: pre_loop_hook_response_tx,
        }
    }
}

impl PythonController {
    pub fn get_client(&self) -> PythonControllerAsyncClient {
        self.client.clone()
    }

    pub fn check_requests(&mut self) {
        if let Ok((node, error, time)) = self.make_command_request.clone().lock().unwrap().try_recv() {
            let command = self.make_command(node, &error, time);
            self.make_command_response.send(command).unwrap();
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

    fn make_command(&mut self, node: NodeWrapper, error: &ControllerError, time: f32) -> Command {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of make_command");
        }
        // let node_record = node.record();
        let result = Python::with_gil(|py| -> CommandWrapper {
            match self.model.bind(py).call_method(
                "make_command",
                (node, ControllerErrorWrapper::from_rust(error), time),
                None,
            ) {
                Err(e) => {
                    e.display(py);
                    panic!("Error while calling 'make_command' method of PythonController.");
                }
                Ok(r) => r
                    .extract()
                    .expect("Error during the call of Python implementation of 'make_command'"),
            }
        });
        result.to_rust()
    }

    fn record(&self) -> ControllerRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = Python::with_gil(|py| {
            match self.model
                .bind(py)
                .call_method("record", (), None) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error while calling 'record' method of PythonController.");
                    }
                    Ok(r) => {
                        r.extract()
                        .expect("The 'record' method of PythonController does not return a valid EstimatorRecord type")
                    }
                }
        });
        let record = ExternalControllerRecord {
            record: Value::from_str(record_str.as_str()).expect(
                "Impossible to get serde_json::Value from the input serialized python structure",
            ),
        };
        // record.clone()
        // StateEstimatorRecord::External(PythonController::record(&self))
        ControllerRecord::External(record)
    }

    fn pre_loop_hook(&mut self, node: NodeWrapper, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of pre_loop_hook");
        }
        Python::with_gil(|py: Python<'_>| {
            match self.model
                .bind(py)
                .call_method("pre_loop_hook", (node, time), None) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error while calling 'next_time_step' method of PythonController.");
                    }
                    Ok(_) => {}
                }
        });
    }
}
