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
    node_factory::NodeRecord,
    physics::physic::Command,
    pywrappers::{CommandWrapper, ControllerErrorWrapper},
    stateful::Stateful,
};

use super::controller::{Controller, ControllerError, ControllerRecord};

#[derive(Debug, Clone)]
pub struct PythonControllerAsyncClient {
    pub make_command_request: mpsc::Sender<(ControllerError, f32)>,
    pub make_command_response: Arc<Mutex<mpsc::Receiver<Command>>>,
    pub record_request: mpsc::Sender<()>,
    pub record_response: Arc<Mutex<mpsc::Receiver<ControllerRecord>>>,
    pub from_record_request: mpsc::Sender<ControllerRecord>,
    pub from_record_response: Arc<Mutex<mpsc::Receiver<()>>>,
}

impl Controller for PythonControllerAsyncClient {
    fn make_command(&mut self, _node: &mut Node, error: &ControllerError, time: f32) -> Command {
        self.make_command_request
            .send((error.clone(), time))
            .unwrap();
        self.make_command_response.lock().unwrap().recv().unwrap()
    }
}

impl Stateful<ControllerRecord> for PythonControllerAsyncClient {
    fn from_record(&mut self, record: ControllerRecord) {
        self.from_record_request.send(record).unwrap();
        self.from_record_response.lock().unwrap().recv().unwrap();
    }

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
    make_command_request: Arc<Mutex<mpsc::Receiver<(ControllerError, f32)>>>,
    make_command_response: mpsc::Sender<Command>,
    record_request: Arc<Mutex<mpsc::Receiver<()>>>,
    record_response: mpsc::Sender<ControllerRecord>,
    from_record_request: Arc<Mutex<mpsc::Receiver<ControllerRecord>>>,
    from_record_response: mpsc::Sender<()>,
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
        let (from_record_request_tx, from_record_request_rx) = mpsc::channel();
        let (from_record_response_tx, from_record_response_rx) = mpsc::channel();

        PythonController {
            model: py_model,
            client: PythonControllerAsyncClient {
                make_command_request: make_command_request_tx,
                make_command_response: Arc::new(Mutex::new(make_command_response_rx)),
                record_request: record_request_tx,
                record_response: Arc::new(Mutex::new(record_response_rx)),
                from_record_request: from_record_request_tx,
                from_record_response: Arc::new(Mutex::new(from_record_response_rx)),
            },
            make_command_request: Arc::new(Mutex::new(make_command_request_rx)),
            make_command_response: make_command_response_tx,
            record_request: Arc::new(Mutex::new(record_request_rx)),
            record_response: record_response_tx,
            from_record_request: Arc::new(Mutex::new(from_record_request_rx)),
            from_record_response: from_record_response_tx,
        }
    }
}

impl PythonController {
    pub fn get_client(&self) -> PythonControllerAsyncClient {
        self.client.clone()
    }

    pub fn check_requests(&mut self) {
        if let Ok((error, time)) =
            self.make_command_request.clone().lock().unwrap().try_recv()
        {
            let command = self.make_command( &error, time);
            self.make_command_response.send(command).unwrap();
        }
        if let Ok(()) = self.record_request.clone().lock().unwrap().try_recv() {
            self.record_response.send(self.record()).unwrap();
        }
        if let Ok(record) = self.from_record_request.clone().lock().unwrap().try_recv() {
            self.from_record(record);
            self.from_record_response.send(()).unwrap();
        }
    }

    fn make_command(&mut self, error: &ControllerError, time: f32) -> Command {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of make_command");
        }
        // let node_record = node.record();
        let result = Python::with_gil(|py| -> CommandWrapper {
            self.model
                .bind(py)
                .call_method(
                    "make_command",
                    (ControllerErrorWrapper::from_ros(error), time),
                    None,
                )
                .expect("PythonController does not have a correct 'make_command' method")
                .extract()
                .expect("Error during the call of Python implementation of 'make_command'")
        });
        result.to_ros()
    }

    fn record(&self) -> ControllerRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = Python::with_gil(|py| {
            self.model
                .bind(py)
                .call_method("record", (), None)
                .expect("Python implementation of PythonController does not have a correct 'record' method")
                .extract()
                .expect("The 'record' method of PythonController does not return a valid EstimatorRecord type")
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

    fn from_record(&mut self, record: ControllerRecord) {
        if let ControllerRecord::External(record) = record {
            if is_enabled(crate::logger::InternalLog::API) {
                debug!("Calling python implementation of from_record");
            }
            Python::with_gil(|py| {
                self.model
                    .bind(py)
                    .call_method("from_record", (serde_json::to_string(&record).unwrap(),), None)
                    .expect("Python implementation of PythonController does not have a correct 'from_record' method");
            });
        }
    }
}
