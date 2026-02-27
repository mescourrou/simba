use std::{str::FromStr, sync::Arc};

use log::debug;
use pyo3::{prelude::*, types::PyDict};

use simba_com::rfc::{self, RemoteFunctionCall, RemoteFunctionCallHost};

use crate::{
    constants::TIME_ROUND,
    errors::SimbaResult,
    logger::is_enabled,
    node::Node,
    physics::robot_models::Command,
    pywrappers::{CommandWrapper, NodeWrapper, ObservationWrapper, WorldStateWrapper},
    recordable::Recordable,
    sensors::Observation,
    utils::{
        maths::round_precision,
        python::{call_py_method, call_py_method_void},
    },
};

use super::{
    external_estimator::ExternalEstimatorRecord,
    {StateEstimator, StateEstimatorRecord, WorldState},
};

#[derive(Debug, Clone)]
pub struct PythonStateEstimatorAsyncClient {
    post_init: RemoteFunctionCall<NodeWrapper, SimbaResult<()>>,
    prediction_step: RemoteFunctionCall<PythonStateEstimatorPredictionStepRequest, ()>,
    correction_step: RemoteFunctionCall<PythonStateEstimatorCorrectionStepRequest, ()>,
    state: RemoteFunctionCall<(), WorldState>,
    next_time_step: RemoteFunctionCall<(), f32>,
    record: RemoteFunctionCall<(), StateEstimatorRecord>,
    pre_loop_hook: RemoteFunctionCall<PythonStateEstimatorPreLoopHookRequest, ()>,
}

impl StateEstimator for PythonStateEstimatorAsyncClient {
    fn post_init(&mut self, node: &mut Node) -> SimbaResult<()> {
        let node_py = NodeWrapper::from_rust(node);
        self.post_init.call(node_py).unwrap()
    }

    fn prediction_step(&mut self, node: &mut Node, command: Option<Command>, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Start prediction step from async client");
        }
        let node_py = NodeWrapper::from_rust(node);
        let command = command.map(|c| CommandWrapper::from_rust(&c));
        self.prediction_step
            .call(PythonStateEstimatorPredictionStepRequest {
                node: node_py,
                command,
                time,
            })
            .unwrap()
    }

    fn correction_step(&mut self, node: &mut Node, observations: &[Observation], time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Start correction step from async client");
        }
        let node_py = NodeWrapper::from_rust(node);
        self.correction_step
            .call(PythonStateEstimatorCorrectionStepRequest {
                node: node_py,
                observations: observations.to_vec(),
                time,
            })
            .unwrap()
    }

    fn next_time_step(&self) -> f32 {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Get next time step from async client");
        }
        self.next_time_step.call(()).unwrap()
    }

    fn world_state(&self) -> WorldState {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Get state from async client");
        }
        self.state.call(()).unwrap()
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Start pre loop hook from async client");
        }
        let node_py = NodeWrapper::from_rust(node);
        self.pre_loop_hook
            .call(PythonStateEstimatorPreLoopHookRequest {
                node: node_py,
                time,
            })
            .unwrap()
    }
}

impl Recordable<StateEstimatorRecord> for PythonStateEstimatorAsyncClient {
    fn record(&self) -> StateEstimatorRecord {
        self.record.call(()).unwrap()
    }
}

#[derive(Debug)]
// #[pyclass(subclass)]
// #[pyo3(name = "StateEstimator")]
pub struct PythonStateEstimator {
    model: Py<PyAny>,
    client: PythonStateEstimatorAsyncClient,
    post_init: Arc<RemoteFunctionCallHost<NodeWrapper, SimbaResult<()>>>,
    prediction_step: Arc<RemoteFunctionCallHost<PythonStateEstimatorPredictionStepRequest, ()>>,
    correction_step: Arc<RemoteFunctionCallHost<PythonStateEstimatorCorrectionStepRequest, ()>>,
    state: Arc<RemoteFunctionCallHost<(), WorldState>>,
    next_time_step: Arc<RemoteFunctionCallHost<(), f32>>,
    record: Arc<RemoteFunctionCallHost<(), StateEstimatorRecord>>,
    pre_loop_hook: Arc<RemoteFunctionCallHost<PythonStateEstimatorPreLoopHookRequest, ()>>,
}

#[derive(Debug, Clone)]
pub struct PythonStateEstimatorPredictionStepRequest {
    pub node: NodeWrapper,
    pub command: Option<CommandWrapper>,
    pub time: f32,
}

#[derive(Debug, Clone)]
pub struct PythonStateEstimatorCorrectionStepRequest {
    pub node: NodeWrapper,
    pub observations: Vec<Observation>,
    pub time: f32,
}

#[derive(Debug, Clone)]
pub struct PythonStateEstimatorPreLoopHookRequest {
    pub node: NodeWrapper,
    pub time: f32,
}

// #[pymethods]
impl PythonStateEstimator {
    // #[new]
    pub fn new(py_model: Py<PyAny>) -> PythonStateEstimator {
        if is_enabled(crate::logger::InternalLog::API) {
            Python::attach(|py| {
                debug!("Model got: {}", py_model.bind(py).dir().unwrap());
            });
        }
        let (post_init_client, post_init_host) = rfc::make_pair();
        let (prediction_step_client, prediction_step_host) = rfc::make_pair();
        let (correction_step_client, correction_step_host) = rfc::make_pair();
        let (state_client, state_host) = rfc::make_pair();
        let (next_time_step_client, next_time_step_host) = rfc::make_pair();
        let (record_client, record_host) = rfc::make_pair();
        let (pre_loop_hook_client, pre_loop_hook_host) = rfc::make_pair();

        PythonStateEstimator {
            model: py_model,
            client: PythonStateEstimatorAsyncClient {
                post_init: post_init_client,
                prediction_step: prediction_step_client,
                correction_step: correction_step_client,
                state: state_client,
                next_time_step: next_time_step_client,
                record: record_client,
                pre_loop_hook: pre_loop_hook_client,
            },
            post_init: Arc::new(post_init_host),
            prediction_step: Arc::new(prediction_step_host),
            correction_step: Arc::new(correction_step_host),
            state: Arc::new(state_host),
            next_time_step: Arc::new(next_time_step_host),
            record: Arc::new(record_host),
            pre_loop_hook: Arc::new(pre_loop_hook_host),
        }
    }
}

impl PythonStateEstimator {
    pub fn get_client(&self) -> PythonStateEstimatorAsyncClient {
        self.client.clone()
    }

    pub fn check_requests(&mut self) {
        self.post_init
            .clone()
            .try_recv_closure_mut(|node| self.post_init(node));
        self.prediction_step
            .clone()
            .try_recv_closure_mut(|request| {
                self.prediction_step(request.node, request.command, request.time)
            });
        self.correction_step
            .clone()
            .try_recv_closure_mut(|request| {
                self.correction_step(request.node, &request.observations, request.time)
            });
        self.state
            .clone()
            .try_recv_closure_mut(|()| self.world_state());
        self.next_time_step
            .clone()
            .try_recv_closure_mut(|()| self.next_time_step());
        self.record.clone().try_recv_closure_mut(|()| self.record());
        self.pre_loop_hook
            .clone()
            .try_recv_closure_mut(|request| self.pre_loop_hook(request.node, request.time));
    }

    fn post_init(&mut self, node: NodeWrapper) -> SimbaResult<()> {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of post_init");
        }
        call_py_method_void!(self.model, "post_init", (node,));
        Ok(())
    }

    fn prediction_step(&mut self, node: NodeWrapper, command: Option<CommandWrapper>, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of prediction_step");
        }
        call_py_method_void!(self.model, "prediction_step", node, command, time);
    }

    fn correction_step(&mut self, node: NodeWrapper, observations: &Vec<Observation>, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of correction_step");
        }
        let mut observation_py = Vec::new();
        for obs in observations {
            observation_py.push(ObservationWrapper::from_rust(obs));
        }
        call_py_method_void!(self.model, "correction_step", node, observation_py, time);
    }

    fn world_state(&self) -> WorldState {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of state");
        }
        let state = call_py_method!(self.model, "state", WorldStateWrapper,);
        state.to_rust()
    }

    fn next_time_step(&self) -> f32 {
        // PythonStateEstimator::next_time_step(self)
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of next_time_step");
        }
        let time = call_py_method!(self.model, "next_time_step", f32,);
        round_precision(time, TIME_ROUND).unwrap()
    }

    fn record(&self) -> StateEstimatorRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = call_py_method!(self.model, "record", String,);
        let record = ExternalEstimatorRecord {
            record: serde_json::Value::from_str(record_str.as_str()).expect(
                "Impossible to get serde_json::Value from the input serialized python structure",
            ),
        };
        // record.clone()
        // StateEstimatorRecord::External(PythonStateEstimator::record(&self))
        StateEstimatorRecord::External(record)
    }

    fn pre_loop_hook(&mut self, node: NodeWrapper, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of pre_loop_hook");
        }
        call_py_method_void!(self.model, "pre_loop_hook", node, time);
    }
}

#[pyclass(subclass)]
#[pyo3(name = "StateEstimator")]
pub struct StateEstimatorWrapper {
    #[pyo3(get, set)]
    pub name: String,
}

#[pymethods]
impl StateEstimatorWrapper {
    #[new]
    pub fn new(_config: Py<PyAny>, _initial_time: f32) -> StateEstimatorWrapper {
        StateEstimatorWrapper {
            name: String::from("anonyme"),
        }
    }

    fn post_init(&mut self, _node: NodeWrapper) {}

    fn prediction_step(&mut self, _node: NodeWrapper, _time: f32) {
        unimplemented!()
    }

    fn correction_step(
        &mut self,
        _node: NodeWrapper,
        _observations: Vec<ObservationWrapper>,
        _time: f32,
    ) {
        unimplemented!()
    }

    fn world_state(&self) -> WorldStateWrapper {
        unimplemented!()
    }

    fn next_time_step(&self) -> f32 {
        unimplemented!()
    }

    fn record(&self) -> Py<PyDict> {
        unimplemented!()
    }

    fn pre_loop_hook(&mut self, _node: NodeWrapper, _time: f32) {
        unimplemented!()
    }
}
