use std::{
    str::FromStr,
    sync::{mpsc::{self, Receiver, Sender}, Arc, Mutex},
};

use log::debug;
use pyo3::prelude::*;
use serde_json::Value;

use crate::{
    constants::TIME_ROUND, logger::is_enabled, networking::message_handler::MessageHandler, node::Node, pywrappers::{NodeWrapper, ObservationWrapper, WorldStateWrapper}, recordable::Recordable, sensors::sensor::Observation, utils::maths::round_precision
};

use super::{
    external_estimator::ExternalEstimatorRecord,
    state_estimator::{StateEstimator, StateEstimatorRecord, WorldState},
};

#[derive(Debug, Clone)]
pub struct PythonStateEstimatorAsyncClient {
    pub prediction_step_request: mpsc::Sender<(NodeWrapper, f32)>,
    pub prediction_step_response: Arc<Mutex<mpsc::Receiver<()>>>,
    pub correction_step_request: mpsc::Sender<(NodeWrapper, Vec<Observation>, f32)>,
    pub correction_step_response: Arc<Mutex<mpsc::Receiver<()>>>,
    pub state_request: mpsc::Sender<()>,
    pub state_response: Arc<Mutex<mpsc::Receiver<WorldState>>>,
    pub next_time_step_request: mpsc::Sender<()>,
    pub next_time_step_response: Arc<Mutex<mpsc::Receiver<f32>>>,
    pub record_request: mpsc::Sender<()>,
    pub record_response: Arc<Mutex<mpsc::Receiver<StateEstimatorRecord>>>,
    pub pre_loop_hook_request: mpsc::Sender<(NodeWrapper, f32)>,
    pub pre_loop_hook_response: Arc<Mutex<mpsc::Receiver<()>>>,
    received_msgs: Vec<(String, String, f32)>,
    letter_box_receiver: Arc<Mutex<Receiver<(String, Value, f32)>>>,
    letter_box_sender: Sender<(String, Value, f32)>,
}

impl PythonStateEstimatorAsyncClient {
    fn update_messages(&mut self) {
        while let  Ok((from, msg, time)) = self.letter_box_receiver.lock().unwrap().try_recv() {
            let msg = serde_json::to_string(&msg).unwrap();
            self.received_msgs.push((from, msg, time));
        }
    }
}

impl StateEstimator for PythonStateEstimatorAsyncClient {
    fn prediction_step(&mut self, node: &mut Node, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Start prediction step from async client");
        }
        self.update_messages();
        let node_py = NodeWrapper::from_rust(&node, self.received_msgs.clone());
        self.prediction_step_request.send((node_py, time)).unwrap();
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Start prediction step from async client: Request sent");
        }
        self.prediction_step_response
            .lock()
            .unwrap()
            .recv()
            .unwrap();
    }

    fn correction_step(&mut self, node: &mut Node, observations: &Vec<Observation>, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Start correction step from async client");
        }
        self.update_messages();
        let node_py = NodeWrapper::from_rust(&node, self.received_msgs.clone());
        self.correction_step_request
            .send((node_py, observations.clone(), time))
            .unwrap();
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Start correction step from async client: Request sent");
        }
        self.correction_step_response
            .lock()
            .unwrap()
            .recv()
            .unwrap();
    }

    fn next_time_step(&self) -> f32 {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Get next time step from async client");
        }
        self.next_time_step_request.send(()).unwrap();
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Get next time step from async client: Request sent");
        }
        self.next_time_step_response
            .lock()
            .unwrap()
            .recv()
            .expect("Error during call of next_time_step")
    }

    fn world_state(&self) -> WorldState {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Get state from async client");
        }
        self.state_request.send(()).unwrap();
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Get state from async client: Request sent");
        }
        self.state_response
            .lock()
            .unwrap()
            .recv()
            .expect("Error during call of state")
    }

    fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Start pre loop hook from async client");
        }
        self.received_msgs.clear();
        self.update_messages();
        let node_py = NodeWrapper::from_rust(&node, self.received_msgs.clone());
        self.pre_loop_hook_request.send((node_py, time)).unwrap();
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Start prediction step from async client: Request sent");
        }
        self.pre_loop_hook_response
            .lock()
            .unwrap()
            .recv()
            .unwrap();
    }
}

impl Recordable<StateEstimatorRecord> for PythonStateEstimatorAsyncClient {
    fn record(&self) -> StateEstimatorRecord {
        self.record_request.send(()).unwrap();
        self.record_response
            .lock()
            .unwrap()
            .recv()
            .expect("Error during call of record")
    }
}

impl MessageHandler for PythonStateEstimatorAsyncClient {
    fn get_letter_box(&self) -> Option<Sender<(String, Value, f32)>> {
        Some(self.letter_box_sender.clone())
    }
}

#[derive(Debug)]
#[pyclass(subclass)]
#[pyo3(name = "StateEstimator")]
pub struct PythonStateEstimator {
    model: Py<PyAny>,
    client: PythonStateEstimatorAsyncClient,
    prediction_step_request: Arc<Mutex<mpsc::Receiver<(NodeWrapper, f32)>>>,
    prediction_step_response: mpsc::Sender<()>,
    correction_step_request: Arc<Mutex<mpsc::Receiver<(NodeWrapper, Vec<Observation>, f32)>>>,
    correction_step_response: mpsc::Sender<()>,
    state_request: Arc<Mutex<mpsc::Receiver<()>>>,
    state_response: mpsc::Sender<WorldState>,
    next_time_step_request: Arc<Mutex<mpsc::Receiver<()>>>,
    next_time_step_response: mpsc::Sender<f32>,
    record_request: Arc<Mutex<mpsc::Receiver<()>>>,
    record_response: mpsc::Sender<StateEstimatorRecord>,
    pre_loop_hook_request: Arc<Mutex<Receiver<(NodeWrapper, f32)>>>,
    pre_loop_hook_response: Sender<()>,
}

#[pymethods]
impl PythonStateEstimator {
    #[new]
    pub fn new(py_model: Py<PyAny>) -> PythonStateEstimator {
        if is_enabled(crate::logger::InternalLog::API) {
            Python::with_gil(|py| {
                debug!("Model got: {}", py_model.bind(py).dir().unwrap());
            });
        }
        let (prediction_request_tx, prediction_request_rx) = mpsc::channel();
        let (prediction_response_tx, prediction_response_rx) = mpsc::channel();
        let (correction_request_tx, correction_request_rx) = mpsc::channel();
        let (correction_response_tx, correction_response_rx) = mpsc::channel();
        let (state_request_tx, state_request_rx) = mpsc::channel();
        let (state_response_tx, state_response_rx) = mpsc::channel();
        let (next_time_step_request_tx, next_time_step_request_rx) = mpsc::channel();
        let (next_time_step_response_tx, next_time_step_response_rx) = mpsc::channel();
        let (record_request_tx, record_request_rx) = mpsc::channel();
        let (record_response_tx, record_response_rx) = mpsc::channel();
        let (pre_loop_hook_request_tx, pre_loop_hook_request_rx) = mpsc::channel();
        let (pre_loop_hook_response_tx, pre_loop_hook_response_rx) = mpsc::channel();
        let (letter_box_tx, letter_box_rx) = mpsc::channel();

        PythonStateEstimator {
            model: py_model,
            client: PythonStateEstimatorAsyncClient {
                prediction_step_request: prediction_request_tx,
                prediction_step_response: Arc::new(Mutex::new(prediction_response_rx)),
                correction_step_request: correction_request_tx,
                correction_step_response: Arc::new(Mutex::new(correction_response_rx)),
                state_request: state_request_tx,
                state_response: Arc::new(Mutex::new(state_response_rx)),
                next_time_step_request: next_time_step_request_tx,
                next_time_step_response: Arc::new(Mutex::new(next_time_step_response_rx)),
                record_request: record_request_tx,
                record_response: Arc::new(Mutex::new(record_response_rx)),
                pre_loop_hook_request: pre_loop_hook_request_tx,
                pre_loop_hook_response: Arc::new(Mutex::new(pre_loop_hook_response_rx)),
                received_msgs: Vec::new(),
                letter_box_receiver: Arc::new(Mutex::new(letter_box_rx)),
                letter_box_sender: letter_box_tx,
            },
            prediction_step_request: Arc::new(Mutex::new(prediction_request_rx)),
            prediction_step_response: prediction_response_tx,
            correction_step_request: Arc::new(Mutex::new(correction_request_rx)),
            correction_step_response: correction_response_tx,
            state_request: Arc::new(Mutex::new(state_request_rx)),
            state_response: state_response_tx,
            next_time_step_request: Arc::new(Mutex::new(next_time_step_request_rx)),
            next_time_step_response: next_time_step_response_tx,
            record_request: Arc::new(Mutex::new(record_request_rx)),
            record_response: record_response_tx,
            pre_loop_hook_request: Arc::new(Mutex::new(pre_loop_hook_request_rx)),
            pre_loop_hook_response: pre_loop_hook_response_tx,
        }
    }
}

impl PythonStateEstimator {
    pub fn get_client(&self) -> PythonStateEstimatorAsyncClient {
        self.client.clone()
    }

    pub fn check_requests(&mut self) {
        if let Ok((node, time)) = self
            .prediction_step_request
            .clone()
            .lock()
            .unwrap()
            .try_recv()
        {
            if is_enabled(crate::logger::InternalLog::API) {
                debug!("Request for prediction step received");
            }
            self.prediction_step(node, time);
            self.prediction_step_response.send(()).unwrap();
            if is_enabled(crate::logger::InternalLog::API) {
                debug!("Response for prediction step sent");
            }
        }
        if let Ok((node, obs, time)) = self
            .correction_step_request
            .clone()
            .lock()
            .unwrap()
            .try_recv()
        {
            if is_enabled(crate::logger::InternalLog::API) {
                debug!("Request for correction step received");
            }
            self.correction_step(node, &obs, time);
            self.correction_step_response.send(()).unwrap();
            if is_enabled(crate::logger::InternalLog::API) {
                debug!("Response for correction step sent");
            }
        }
        if let Ok(()) = self.state_request.clone().lock().unwrap().try_recv() {
            if is_enabled(crate::logger::InternalLog::API) {
                debug!("Request for state received");
            }
            self.state_response.send(self.world_state()).unwrap();
            if is_enabled(crate::logger::InternalLog::API) {
                debug!("Response for state sent");
            }
        }
        if let Ok(()) = self
            .next_time_step_request
            .clone()
            .lock()
            .unwrap()
            .try_recv()
        {
            if is_enabled(crate::logger::InternalLog::API) {
                debug!("Request for next time step received");
            }
            self.next_time_step_response
                .send(self.next_time_step())
                .unwrap();

            if is_enabled(crate::logger::InternalLog::API) {
                debug!("Response for next time step sent");
            }
        }
        if let Ok(()) = self.record_request.clone().lock().unwrap().try_recv() {
            self.record_response.send(self.record()).unwrap();
        }
        if let Ok((node, time)) = self.pre_loop_hook_request.clone().lock().unwrap().try_recv() {
            self.pre_loop_hook(node, time);
            self.pre_loop_hook_response.send(()).unwrap();
        }
    }

    fn prediction_step(&mut self, node: NodeWrapper, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of prediction_step");
        }
        // let node_record = node.record();
        Python::with_gil(|py| {
            if let Err(e) = self
                .model
                .bind(py)
                .call_method("prediction_step", (node, time), None)
            {
                e.display(py);
                panic!("Error while calling 'prediction_step' method of PythonStateEstimator.");
            }
        });
    }

    fn correction_step(&mut self, node: NodeWrapper, observations: &Vec<Observation>, time: f32) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of correction_step");
        }
        let mut observation_py = Vec::new();
        for obs in observations {
            observation_py.push(ObservationWrapper::from_rust(obs));
        }
        Python::with_gil(|py| {
            if let Err(e) =
                self.model
                    .bind(py)
                    .call_method("correction_step", (node, observation_py, time), None)
            {
                e.display(py);
                panic!("Error while calling 'correction_step' method of PythonStateEstimator.");
            }
        });
    }

    fn world_state(&self) -> WorldState {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of state");
        }
        let state = Python::with_gil(|py| -> WorldStateWrapper {
            match self.model
                .bind(py)
                .call_method("state", (), None) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error while calling 'state' method of PythonStateEstimator.");
                    }
                    Ok(r) => {
                        r.extract()
                        .expect("The 'state' method of PythonStateEstimator does not return a correct state vector")
                    }
                }
        });
        state.to_rust()
    }

    fn next_time_step(&self) -> f32 {
        // PythonStateEstimator::next_time_step(self)
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of next_time_step");
        }
        let time = Python::with_gil(|py| {
            match self.model
                .bind(py)
                .call_method("next_time_step", (), None) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error while calling 'next_time_step' method of PythonStateEstimator.");
                    }
                    Ok(r) => {
                        r.extract()
                        .expect("The 'next_time_step' method of PythonStateEstimator does not return a correct time for next step")
                    }
                }
        });
        round_precision(time, TIME_ROUND).unwrap()
    }

    fn record(&self) -> StateEstimatorRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = Python::with_gil(|py| {
            match self.model
                .bind(py)
                .call_method("record", (), None) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error while calling 'record' method of PythonStateEstimator.");
                    }
                    Ok(r) => {
                        r.extract()
                        .expect("The 'record' method of PythonStateEstimator does not return a valid EstimatorRecord type")
                    }
                }
        });
        let record = ExternalEstimatorRecord {
            record: Value::from_str(record_str.as_str()).expect(
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
        Python::with_gil(|py: Python<'_>| {
            match self.model
                .bind(py)
                .call_method("pre_loop_hook", (node, time), None) {
                    Err(e) => {
                        e.display(py);
                        panic!("Error while calling 'next_time_step' method of PythonEstimator.");
                    }
                    Ok(_) => {}
                }
        });
    }
}
