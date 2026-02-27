use std::{str::FromStr, sync::Arc};

use log::debug;
use pyo3::{prelude::*, types::PyDict};
use serde_json::Value;

use simba_com::rfc::{self, RemoteFunctionCall, RemoteFunctionCallHost};

use crate::{
    controllers::ControllerError,
    errors::SimbaResult,
    logger::is_enabled,
    navigators::external_navigator::ExternalNavigatorRecord,
    node::Node,
    pywrappers::{ControllerErrorWrapper, NodeWrapper, WorldStateWrapper},
    recordable::Recordable,
    state_estimators::WorldState,
    utils::python::{call_py_method, call_py_method_void},
};

use super::{Navigator, NavigatorRecord};

#[derive(Debug, Clone)]
pub struct PythonNavigatorAsyncClient {
    pub post_init: RemoteFunctionCall<NodeWrapper, SimbaResult<()>>,
    pub compute_error: RemoteFunctionCall<(NodeWrapper, WorldState), ControllerError>,
    pub record: RemoteFunctionCall<(), NavigatorRecord>,
    pub pre_loop_hook: RemoteFunctionCall<(NodeWrapper, f32), ()>,
    pub next_time_step: RemoteFunctionCall<(), Option<f32>>,
}

impl Navigator for PythonNavigatorAsyncClient {
    fn post_init(&mut self, node: &mut Node) -> SimbaResult<()> {
        let node_py = NodeWrapper::from_rust(node);
        self.post_init.call(node_py).unwrap()
    }

    fn compute_error(&mut self, node: &mut Node, world_state: WorldState) -> ControllerError {
        let node_py = NodeWrapper::from_rust(node);
        self.compute_error.call((node_py, world_state)).unwrap()
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        let node_py = NodeWrapper::from_rust(node);
        self.pre_loop_hook.call((node_py, time)).unwrap()
    }

    fn next_time_step(&self) -> Option<f32> {
        self.next_time_step.call(()).unwrap()
    }
}

impl Recordable<NavigatorRecord> for PythonNavigatorAsyncClient {
    fn record(&self) -> NavigatorRecord {
        self.record.call(()).unwrap()
    }
}

#[derive(Debug)]
pub struct PythonNavigator {
    model: Py<PyAny>,
    client: PythonNavigatorAsyncClient,
    post_init: Arc<RemoteFunctionCallHost<NodeWrapper, SimbaResult<()>>>,
    compute_error: Arc<RemoteFunctionCallHost<(NodeWrapper, WorldState), ControllerError>>,
    record: Arc<RemoteFunctionCallHost<(), NavigatorRecord>>,
    pre_loop_hook: Arc<RemoteFunctionCallHost<(NodeWrapper, f32), ()>>,
    next_time_step: Arc<RemoteFunctionCallHost<(), Option<f32>>>,
}

impl PythonNavigator {
    pub fn new(py_model: Py<PyAny>) -> PythonNavigator {
        if is_enabled(crate::logger::InternalLog::API) {
            Python::attach(|py| {
                debug!("Model got: {}", py_model.bind(py).dir().unwrap());
            });
        }
        let (post_init_client, post_init_host) = rfc::make_pair();
        let (compute_error_client, compute_error_host) = rfc::make_pair();
        let (record_client, record_host) = rfc::make_pair();
        let (pre_loop_hook_client, pre_loop_hook_host) = rfc::make_pair();
        let (next_time_step_client, next_time_step_host) = rfc::make_pair();

        PythonNavigator {
            model: py_model,
            client: PythonNavigatorAsyncClient {
                post_init: post_init_client,
                compute_error: compute_error_client,
                record: record_client,
                pre_loop_hook: pre_loop_hook_client,
                next_time_step: next_time_step_client,
            },
            post_init: Arc::new(post_init_host),
            compute_error: Arc::new(compute_error_host),
            record: Arc::new(record_host),
            pre_loop_hook: Arc::new(pre_loop_hook_host),
            next_time_step: Arc::new(next_time_step_host),
        }
    }
}

impl PythonNavigator {
    pub fn get_client(&self) -> PythonNavigatorAsyncClient {
        self.client.clone()
    }

    pub fn check_requests(&mut self) {
        self.post_init
            .clone()
            .try_recv_closure_mut(|node| self.post_init(node));
        self.compute_error
            .clone()
            .try_recv_closure_mut(|(node, state)| self.compute_error(node, &state));
        self.record.try_recv_closure(|()| self.record());
        self.pre_loop_hook
            .clone()
            .try_recv_closure_mut(|(node, time)| self.pre_loop_hook(node, time));
        self.next_time_step
            .clone()
            .try_recv_closure(|()| self.next_time_step());
    }

    fn post_init(&mut self, node: NodeWrapper) -> SimbaResult<()> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of post_init");
        }
        call_py_method_void!(self.model, "post_init", (node,));
        Ok(())
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

    fn next_time_step(&self) -> Option<f32> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of next_time_step");
        }
        call_py_method!(self.model, "next_time_step", Option<f32>,)
    }
}

#[pyclass(subclass)]
#[pyo3(name = "Navigator")]
pub struct NavigatorWrapper {}

#[pymethods]
impl NavigatorWrapper {
    #[new]
    pub fn new(_config: Py<PyAny>, _initial_time: f32) -> NavigatorWrapper {
        Self {}
    }

    fn post_init(&mut self, _node: NodeWrapper) {}

    fn compute_error(
        &mut self,
        _node: NodeWrapper,
        _state: WorldStateWrapper,
    ) -> ControllerErrorWrapper {
        unimplemented!()
    }

    fn record(&self) -> Py<PyDict> {
        unimplemented!()
    }

    fn pre_loop_hook(&mut self, _node: NodeWrapper, _time: f32) {
        unimplemented!()
    }

    fn next_time_step(&self) -> Option<f32> {
        None
    }
}
