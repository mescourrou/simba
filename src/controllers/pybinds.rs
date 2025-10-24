use std::{
    str::FromStr,
    sync::{mpsc, Arc, Mutex},
};

use log::debug;
use pyo3::prelude::*;
use serde_json::Value;

use crate::{
    controllers::external_controller::ExternalControllerRecord,
    logger::is_enabled,
    node::Node,
    physics::physics::Command,
    pywrappers::{CommandWrapper, ControllerErrorWrapper, NodeWrapper},
    recordable::Recordable,
};

use super::controller::{Controller, ControllerError, ControllerRecord};

#[derive(Debug, Clone)]
pub struct PythonControllerAsyncClient {
    pub make_command_request: mpsc::Sender<(NodeWrapper, ControllerError, f32)>,
    pub make_command_response: Arc<Mutex<mpsc::Receiver<Command>>>,
    pub record_request: mpsc::Sender<()>,
    pub record_response: Arc<Mutex<mpsc::Receiver<ControllerRecord>>>,
}

impl Controller for PythonControllerAsyncClient {
    fn make_command(&mut self, node: &mut Node, error: &ControllerError, time: f32) -> Command {
        let node_py = NodeWrapper::from_rust(&node);
        self.make_command_request
            .send((node_py, error.clone(), time))
            .unwrap();
        self.make_command_response.lock().unwrap().recv().unwrap()
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

        PythonController {
            model: py_model,
            client: PythonControllerAsyncClient {
                make_command_request: make_command_request_tx,
                make_command_response: Arc::new(Mutex::new(make_command_response_rx)),
                record_request: record_request_tx,
                record_response: Arc::new(Mutex::new(record_response_rx)),
            },
            make_command_request: Arc::new(Mutex::new(make_command_request_rx)),
            make_command_response: make_command_response_tx,
            record_request: Arc::new(Mutex::new(record_request_rx)),
            record_response: record_response_tx,
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
}
