use std::{
    str::FromStr,
    sync::{mpsc, Arc, Mutex},
};

use log::debug;
use pyo3::prelude::*;
use serde_json::Value;

use crate::{
    networking::service::HasService,
    physics::external_physic::ExternalPhysicRecord,
    pywrappers::{CommandWrapper, StateWrapper},
    state_estimators::state_estimator::{State, StateRecord},
    stateful::Stateful,
};

use super::physic::{Command, GetRealStateReq, GetRealStateResp, Physic, PhysicRecord};

#[derive(Debug, Clone)]
pub struct PythonPhysicAsyncClient {
    apply_command_request: mpsc::Sender<(Command, f32)>,
    apply_command_response: Arc<Mutex<mpsc::Receiver<()>>>,
    state_request: mpsc::Sender<f32>,
    state_response: Arc<Mutex<mpsc::Receiver<State>>>,
    update_state_request: mpsc::Sender<f32>,
    update_state_response: Arc<Mutex<mpsc::Receiver<()>>>,
    record_request: mpsc::Sender<()>,
    record_response: Arc<Mutex<mpsc::Receiver<PhysicRecord>>>,
    from_record_request: mpsc::Sender<PhysicRecord>,
    from_record_response: Arc<Mutex<mpsc::Receiver<()>>>,
    last_state: State,
}

impl Physic for PythonPhysicAsyncClient {
    fn apply_command(&mut self, command: &Command, time: f32) {
        self.apply_command_request
            .send((command.clone(), time))
            .unwrap();
        self.apply_command_response.lock().unwrap().recv().unwrap()
    }

    fn state(&self, time: f32) -> &State {
        &self.last_state
    }

    fn update_state(&mut self, time: f32) {
        self.update_state_request.send(time).unwrap();
        self.update_state_response.lock().unwrap().recv().unwrap();
        self.state_request.send(time).unwrap();
        self.last_state = self.state_response.lock().unwrap().recv().unwrap().clone()
    }
}

impl Stateful<PhysicRecord> for PythonPhysicAsyncClient {
    fn from_record(&mut self, record: PhysicRecord) {
        self.from_record_request.send(record).unwrap();
        self.from_record_response.lock().unwrap().recv().unwrap();
    }

    fn record(&self) -> PhysicRecord {
        self.record_request.send(()).unwrap();
        self.record_response
            .lock()
            .unwrap()
            .recv()
            .expect("Error during call of record")
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
#[pyclass(subclass)]
#[pyo3(name = "Physics")]
pub struct PythonPhysic {
    model: Py<PyAny>,
    client: PythonPhysicAsyncClient,
    apply_command_request: Arc<Mutex<mpsc::Receiver<(Command, f32)>>>,
    apply_command_response: mpsc::Sender<()>,
    state_request: Arc<Mutex<mpsc::Receiver<f32>>>,
    state_response: mpsc::Sender<State>,
    update_state_request: Arc<Mutex<mpsc::Receiver<f32>>>,
    update_state_response: mpsc::Sender<()>,
    record_request: Arc<Mutex<mpsc::Receiver<()>>>,
    record_response: mpsc::Sender<PhysicRecord>,
    from_record_request: Arc<Mutex<mpsc::Receiver<PhysicRecord>>>,
    from_record_response: mpsc::Sender<()>,
}

#[pymethods]
impl PythonPhysic {
    #[new]
    pub fn new(py_model: Py<PyAny>) -> PythonPhysic {
        Python::with_gil(|py| {
            debug!("Model got: {}", py_model.bind(py).dir().unwrap());
        });
        let (apply_command_request_tx, apply_command_request_rx) = mpsc::channel();
        let (apply_command_response_tx, apply_command_response_rx) = mpsc::channel();
        let (state_request_tx, state_request_rx) = mpsc::channel();
        let (state_response_tx, state_response_rx) = mpsc::channel();
        let (update_state_request_tx, update_state_request_rx) = mpsc::channel();
        let (update_state_response_tx, update_state_response_rx) = mpsc::channel();
        let (record_request_tx, record_request_rx) = mpsc::channel();
        let (record_response_tx, record_response_rx) = mpsc::channel();
        let (from_record_request_tx, from_record_request_rx) = mpsc::channel();
        let (from_record_response_tx, from_record_response_rx) = mpsc::channel();

        PythonPhysic {
            model: py_model,
            client: PythonPhysicAsyncClient {
                apply_command_request: apply_command_request_tx,
                apply_command_response: Arc::new(Mutex::new(apply_command_response_rx)),
                state_request: state_request_tx,
                state_response: Arc::new(Mutex::new(state_response_rx)),
                update_state_request: update_state_request_tx,
                update_state_response: Arc::new(Mutex::new(update_state_response_rx)),
                record_request: record_request_tx,
                record_response: Arc::new(Mutex::new(record_response_rx)),
                from_record_request: from_record_request_tx,
                from_record_response: Arc::new(Mutex::new(from_record_response_rx)),
                last_state: State::new(),
            },
            apply_command_request: Arc::new(Mutex::new(apply_command_request_rx)),
            apply_command_response: apply_command_response_tx,
            state_request: Arc::new(Mutex::new(state_request_rx)),
            state_response: state_response_tx,
            update_state_request: Arc::new(Mutex::new(update_state_request_rx)),
            update_state_response: update_state_response_tx,
            record_request: Arc::new(Mutex::new(record_request_rx)),
            record_response: record_response_tx,
            from_record_request: Arc::new(Mutex::new(from_record_request_rx)),
            from_record_response: from_record_response_tx,
        }
    }
}

impl PythonPhysic {
    pub fn get_client(&self) -> PythonPhysicAsyncClient {
        self.client.clone()
    }

    pub fn check_requests(&mut self) {
        if let Ok((command, time)) = self
            .apply_command_request
            .clone()
            .lock()
            .unwrap()
            .try_recv()
        {
            self.apply_command(&command, time);
            self.apply_command_response.send(()).unwrap();
        }
        if let Ok(time) = self.state_request.clone().lock().unwrap().try_recv() {
            let state = self.state(time);
            self.state_response.send(state).unwrap();
        }
        if let Ok(time) = self.update_state_request.clone().lock().unwrap().try_recv() {
            self.update_state(time);
            self.update_state_response.send(()).unwrap();
        }
        if let Ok(()) = self.record_request.clone().lock().unwrap().try_recv() {
            self.record_response.send(self.record()).unwrap();
        }
        if let Ok(record) = self.from_record_request.clone().lock().unwrap().try_recv() {
            self.from_record(record);
            self.from_record_response.send(()).unwrap();
        }
    }

    fn apply_command(&mut self, command: &Command, time: f32) {
        debug!("Calling python implementation of apply_command");
        // let robot_record = robot.record();
        Python::with_gil(|py| {
            self.model
                .bind(py)
                .call_method(
                    "apply_command",
                    (CommandWrapper::from_ros(command), time),
                    None,
                )
                .expect("PythonPhysic does not have a correct 'apply_command' method");
        });
    }

    fn update_state(&mut self, time: f32) {
        debug!("Calling python implementation of update_state");
        // let robot_record = robot.record();
        Python::with_gil(|py| {
            self.model
                .bind(py)
                .call_method("update_state", (time,), None)
                .expect("PythonPhysic does not have a correct 'update_state' method");
        });
    }

    fn state(&mut self, time: f32) -> State {
        debug!("Calling python implementation of state");
        // let robot_record = robot.record();
        let state = Python::with_gil(|py| -> StateWrapper {
            self.model
                .bind(py)
                .call_method("state", (time,), None)
                .expect("PythonPhysic does not have a correct 'state' method")
                .extract()
                .expect("The 'state' method of PythonPhysic does not return a correct state vector")
        });
        state.to_ros()
    }

    fn record(&self) -> PhysicRecord {
        debug!("Calling python implementation of record");
        let record_str: String = Python::with_gil(|py| {
            self.model
                .bind(py)
                .call_method("record", (), None)
                .expect("Python implementation of PythonPhysic does not have a correct 'record' method")
                .extract()
                .expect("The 'record' method of PythonPhysic does not return a valid EstimatorRecord type")
        });
        let record = ExternalPhysicRecord {
            record: Value::from_str(record_str.as_str()).expect(
                "Impossible to get serde_json::Value from the input serialized python structure",
            ),
        };
        // record.clone()
        // StateEstimatorRecord::External(PythonPhysic::record(&self))
        PhysicRecord::External(record)
    }

    fn from_record(&mut self, record: PhysicRecord) {
        if let PhysicRecord::External(record) = record {
            debug!("Calling python implementation of from_record");
            Python::with_gil(|py| {
                self.model
                    .bind(py)
                    .call_method("from_record", (serde_json::to_string(&record).unwrap(),), None)
                    .expect("Python implementation of PythonPhysic does not have a correct 'from_record' method");
            });
        }
    }
}
