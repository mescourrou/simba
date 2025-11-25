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
    controllers::external_controller::ExternalControllerRecord,
    logger::is_enabled,
    networking::{message_handler::MessageHandler, network::Envelope},
    node::Node,
    physics::robot_models::Command,
    pywrappers::{CommandWrapper, ControllerErrorWrapper, NodeWrapper},
    recordable::Recordable,
    utils::rfc::{self, RemoteFunctionCall, RemoteFunctionCallHost},
};

use super::controller::{Controller, ControllerError, ControllerRecord};

#[derive(Debug, Clone)]
pub struct PythonControllerAsyncClient {
    pub make_command: RemoteFunctionCall<(NodeWrapper, ControllerError, f32), Command>,
    pub record: RemoteFunctionCall<(), ControllerRecord>,
    pub pre_loop_hook: RemoteFunctionCall<(NodeWrapper, f32), ()>,
    letter_box_receiver: Arc<Mutex<Receiver<Envelope>>>,
    letter_box_sender: Sender<Envelope>,
}

impl Controller for PythonControllerAsyncClient {
    fn make_command(&mut self, node: &mut Node, error: &ControllerError, time: f32) -> Command {
        let node_py = NodeWrapper::from_rust(node, self.letter_box_receiver.clone());
        self.make_command
            .call((node_py, error.clone(), time))
            .unwrap()
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        let node_py = NodeWrapper::from_rust(node, self.letter_box_receiver.clone());
        self.pre_loop_hook.call((node_py, time)).unwrap()
    }
}

impl Recordable<ControllerRecord> for PythonControllerAsyncClient {
    fn record(&self) -> ControllerRecord {
        self.record.call(()).unwrap()
    }
}

impl MessageHandler for PythonControllerAsyncClient {
    fn get_letter_box(&self) -> Option<Sender<Envelope>> {
        Some(self.letter_box_sender.clone())
    }
}

#[derive(Debug)]
#[pyclass(subclass)]
#[pyo3(name = "Controller")]
pub struct PythonController {
    model: Py<PyAny>,
    client: PythonControllerAsyncClient,
    make_command: Arc<RemoteFunctionCallHost<(NodeWrapper, ControllerError, f32), Command>>,
    record: Arc<RemoteFunctionCallHost<(), ControllerRecord>>,
    pre_loop_hook: Arc<RemoteFunctionCallHost<(NodeWrapper, f32), ()>>,
}

#[pymethods]
impl PythonController {
    #[new]
    pub fn new(py_model: Py<PyAny>) -> PythonController {
        if is_enabled(crate::logger::InternalLog::API) {
            Python::attach(|py| {
                debug!("Model got: {}", py_model.bind(py).dir().unwrap());
            });
        }

        let (letter_box_sender, letter_box_receiver) = mpsc::channel();

        let (make_command_client, make_command_host) = rfc::make_pair();
        let (record_client, record_host) = rfc::make_pair();
        let (pre_loop_hook_client, pre_loop_hook_host) = rfc::make_pair();

        PythonController {
            model: py_model,
            client: PythonControllerAsyncClient {
                make_command: make_command_client,
                record: record_client,
                pre_loop_hook: pre_loop_hook_client,
                letter_box_receiver: Arc::new(Mutex::new(letter_box_receiver)),
                letter_box_sender,
            },
            make_command: Arc::new(make_command_host),
            record: Arc::new(record_host),
            pre_loop_hook: Arc::new(pre_loop_hook_host),
        }
    }
}

impl PythonController {
    pub fn get_client(&self) -> PythonControllerAsyncClient {
        self.client.clone()
    }

    pub fn check_requests(&mut self) {
        self.make_command
            .clone()
            .try_recv_closure_mut(|(node, error, time)| self.make_command(node, &error, time));
        self.record.try_recv_closure(|()| self.record());
        self.pre_loop_hook
            .clone()
            .try_recv_closure_mut(|(node, time)| self.pre_loop_hook(node, time));
    }

    fn make_command(&mut self, node: NodeWrapper, error: &ControllerError, time: f32) -> Command {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of make_command");
        }
        // let node_record = node.record();
        let result = Python::attach(|py| -> CommandWrapper {
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
        let record_str: String = Python::attach(|py| {
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
        Python::attach(|py: Python<'_>| {
            if let Err(e) = self
                .model
                .bind(py)
                .call_method("pre_loop_hook", (node, time), None)
            {
                e.display(py);
                panic!("Error while calling 'pre_loop_hook' method of PythonController.");
            }
        });
    }
}
