use std::{str::FromStr, sync::Arc};

use log::debug;
use pyo3::{prelude::*, types::PyDict};
use serde_json::Value;
use simba_com::rfc::{self, RemoteFunctionCall, RemoteFunctionCallHost};

use crate::{
    logger::is_enabled,
    networking::service::HasService,
    physics::{external_physics::ExternalPhysicsRecord, robot_models::Command},
    pywrappers::{CommandWrapper, StateWrapper},
    recordable::Recordable,
    state_estimators::State,
    utils::{
        python::{call_py_method, call_py_method_void},
    },
};

use super::{GetRealStateReq, GetRealStateResp, Physics, PhysicsRecord};

#[derive(Debug, Clone)]
pub struct PythonPhysicAsyncClient {
    apply_command: RemoteFunctionCall<(Command, f32), ()>,
    state: RemoteFunctionCall<f32, State>,
    update_state: RemoteFunctionCall<f32, ()>,
    record: RemoteFunctionCall<(), PhysicsRecord>,
    last_state: State,
}

impl Physics for PythonPhysicAsyncClient {
    fn apply_command(&mut self, command: &Command, time: f32) {
        self.apply_command.call((command.clone(), time)).unwrap()
    }

    fn state(&self, _time: f32) -> State {
        self.last_state.clone()
    }

    fn update_state(&mut self, time: f32) {
        self.update_state.call(time).unwrap();
        self.last_state = self.state.call(time).unwrap();
    }
}

impl Recordable<PhysicsRecord> for PythonPhysicAsyncClient {
    fn record(&self) -> PhysicsRecord {
        self.record.call(()).unwrap()
    }
}

impl HasService<GetRealStateReq, GetRealStateResp> for PythonPhysicAsyncClient {
    fn handle_service_requests(
        &mut self,
        _req: GetRealStateReq,
        time: f32,
    ) -> Result<GetRealStateResp, String> {
        Ok(GetRealStateResp {
            state: self.state(time).clone(),
        })
    }
}

#[derive(Debug)]
pub struct PythonPhysics {
    model: Py<PyAny>,
    client: PythonPhysicAsyncClient,
    apply_command: Arc<RemoteFunctionCallHost<(Command, f32), ()>>,
    state: Arc<RemoteFunctionCallHost<f32, State>>,
    update_state: Arc<RemoteFunctionCallHost<f32, ()>>,
    record: Arc<RemoteFunctionCallHost<(), PhysicsRecord>>,
}

impl PythonPhysics {
    pub fn new(py_model: Py<PyAny>) -> PythonPhysics {
        if is_enabled(crate::logger::InternalLog::API) {
            Python::attach(|py| {
                debug!("Model got: {}", py_model.bind(py).dir().unwrap());
            });
        }

        let (apply_command_client, apply_command_host) = rfc::make_pair();
        let (state_client, state_host) = rfc::make_pair();
        let (update_state_client, update_state_host) = rfc::make_pair();
        let (record_client, record_host) = rfc::make_pair();

        PythonPhysics {
            model: py_model,
            client: PythonPhysicAsyncClient {
                apply_command: apply_command_client,
                state: state_client,
                update_state: update_state_client,
                record: record_client,
                last_state: State::new(),
            },
            apply_command: Arc::new(apply_command_host),
            state: Arc::new(state_host),
            update_state: Arc::new(update_state_host),
            record: Arc::new(record_host),
        }
    }
}

impl PythonPhysics {
    pub fn get_client(&self) -> PythonPhysicAsyncClient {
        self.client.clone()
    }

    pub fn check_requests(&mut self) {
        self.apply_command
            .clone()
            .try_recv_closure_mut(|(command, time)| self.apply_command(&command, time));
        self.state
            .clone()
            .try_recv_closure_mut(|time| self.state(time));
        self.update_state
            .clone()
            .try_recv_closure_mut(|time| self.update_state(time));
        self.record.try_recv_closure(|()| self.record());
    }

    fn apply_command(&mut self, command: &Command, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of apply_command");
        }
        call_py_method_void!(
            self.model,
            "apply_command",
            CommandWrapper::from_rust(command),
            time
        );
    }

    fn update_state(&mut self, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of update_state");
        }
        call_py_method_void!(self.model, "update_state", (time,));
    }

    fn state(&mut self, time: f32) -> State {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of state");
        }
        // let robot_record = robot.record();
        let state = call_py_method!(self.model, "state", StateWrapper, (time,));
        state.to_rust()
    }

    fn record(&self) -> PhysicsRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = call_py_method!(self.model, "record", String,);
        let record = ExternalPhysicsRecord {
            record: Value::from_str(&record_str).expect(
                "Impossible to get serde_json::Value from the input serialized python structure",
            ),
        };
        // record.clone()
        // StateEstimatorRecord::External(PythonPhysics::record(&self))
        PhysicsRecord::External(record)
    }
}

#[pyclass(subclass)]
#[pyo3(name = "Physics")]
pub struct PhysicsWrapper {}

#[pymethods]
impl PhysicsWrapper {
    #[new]
    pub fn new(_config: Py<PyAny>, _initial_time: f32) -> PhysicsWrapper {
        Self {}
    }

    fn apply_command(&mut self, _command: CommandWrapper, _time: f32) {
        unimplemented!()
    }

    fn update_state(&mut self, _time: f32) {
        unimplemented!()
    }

    fn state(&mut self, _time: f32) -> StateWrapper {
        unimplemented!()
    }

    fn record(&self) -> Py<PyDict> {
        unimplemented!()
    }
}
