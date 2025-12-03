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
    controllers::ControllerError,
    logger::is_enabled,
    navigators::external_navigator::ExternalNavigatorRecord,
    networking::{message_handler::MessageHandler, network::Envelope},
    node::Node,
    pywrappers::{ControllerErrorWrapper, NodeWrapper, WorldStateWrapper},
    recordable::Recordable,
    state_estimators::WorldState,
    utils::{
        python::{call_py_method, call_py_method_void}, rfc::{self, RemoteFunctionCall, RemoteFunctionCallHost}
    },
};

use super::{Navigator, NavigatorRecord};

#[derive(Debug, Clone)]
pub struct PythonNavigatorAsyncClient {
    pub compute_error: RemoteFunctionCall<(NodeWrapper, WorldState), ControllerError>,
    pub record: RemoteFunctionCall<(), NavigatorRecord>,
    pub pre_loop_hook: RemoteFunctionCall<(NodeWrapper, f32), ()>,
    letter_box_receiver: Arc<Mutex<Receiver<Envelope>>>,
    letter_box_sender: Sender<Envelope>,
}

impl Navigator for PythonNavigatorAsyncClient {
    fn compute_error(&mut self, node: &mut Node, world_state: WorldState) -> ControllerError {
        let node_py = NodeWrapper::from_rust(node, self.letter_box_receiver.clone());
        self.compute_error.call((node_py, world_state)).unwrap()
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        let node_py = NodeWrapper::from_rust(node, self.letter_box_receiver.clone());
        self.pre_loop_hook.call((node_py, time)).unwrap()
    }
}

impl Recordable<NavigatorRecord> for PythonNavigatorAsyncClient {
    fn record(&self) -> NavigatorRecord {
        self.record.call(()).unwrap()
    }
}

impl MessageHandler for PythonNavigatorAsyncClient {
    fn get_letter_box(&self) -> Option<Sender<Envelope>> {
        Some(self.letter_box_sender.clone())
    }
}

#[derive(Debug)]
#[pyclass(subclass)]
#[pyo3(name = "Navigator")]
pub struct PythonNavigator {
    model: Py<PyAny>,
    client: PythonNavigatorAsyncClient,
    compute_error: Arc<RemoteFunctionCallHost<(NodeWrapper, WorldState), ControllerError>>,
    record: Arc<RemoteFunctionCallHost<(), NavigatorRecord>>,
    pre_loop_hook: Arc<RemoteFunctionCallHost<(NodeWrapper, f32), ()>>,
}

#[pymethods]
impl PythonNavigator {
    #[new]
    pub fn new(py_model: Py<PyAny>) -> PythonNavigator {
        if is_enabled(crate::logger::InternalLog::API) {
            Python::attach(|py| {
                debug!("Model got: {}", py_model.bind(py).dir().unwrap());
            });
        }
        let (letter_box_sender, letter_box_receiver) = mpsc::channel();

        let (compute_error_client, compute_error_host) = rfc::make_pair();
        let (record_client, record_host) = rfc::make_pair();
        let (pre_loop_hook_client, pre_loop_hook_host) = rfc::make_pair();

        PythonNavigator {
            model: py_model,
            client: PythonNavigatorAsyncClient {
                compute_error: compute_error_client,
                record: record_client,
                pre_loop_hook: pre_loop_hook_client,
                letter_box_receiver: Arc::new(Mutex::new(letter_box_receiver)),
                letter_box_sender,
            },
            compute_error: Arc::new(compute_error_host),
            record: Arc::new(record_host),
            pre_loop_hook: Arc::new(pre_loop_hook_host),
        }
    }
}

impl PythonNavigator {
    pub fn get_client(&self) -> PythonNavigatorAsyncClient {
        self.client.clone()
    }

    pub fn check_requests(&mut self) {
        self.compute_error
            .clone()
            .try_recv_closure_mut(|(node, state)| self.compute_error(node, &state));
        self.record.try_recv_closure(|()| self.record());
        self.pre_loop_hook
            .clone()
            .try_recv_closure_mut(|(node, time)| self.pre_loop_hook(node, time));
    }

    fn compute_error(&mut self, node: NodeWrapper, state: &WorldState) -> ControllerError {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of compute_error");
        }
        let result = call_py_method!(
            self.model,
            "compute_error",
            ControllerErrorWrapper,
            (node, WorldStateWrapper::from_rust(state))
        );
        result.to_rust()
    }

    fn record(&self) -> NavigatorRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = call_py_method!(self.model, "record", String,);
        NavigatorRecord::External(ExternalNavigatorRecord {
            record: Value::from_str(&record_str).expect(
                "Impossible to get serde_json::Value from the input serialized python structure",
            ),
        })
    }

    fn pre_loop_hook(&mut self, node: NodeWrapper, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of pre_loop_hook");
        }
        call_py_method_void!(self.model, "pre_loop_hook", node, time);
    }
}
