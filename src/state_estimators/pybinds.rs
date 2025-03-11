use std::{
    str::FromStr,
    sync::{mpsc, Arc, Mutex},
};

use log::debug;
use pyo3::{prelude::*, types::PyTuple};
use serde_json::Value;

use crate::{
    robot::Robot,
    sensors::{
        gnss_sensor::GNSSObservationRecord,
        odometry_sensor::OdometryObservationRecord,
        oriented_landmark_sensor::OrientedLandmarkObservationRecord,
        robot_sensor::OrientedRobotObservationRecord,
        sensor::{Observation, ObservationRecord},
    },
    stateful::Stateful,
};

use super::{
    external_estimator::ExternalEstimatorRecord,
    state_estimator::{State, StateEstimator, StateEstimatorRecord},
};

pub fn make_state_estimator_module(m: &Bound<'_, PyModule>) -> PyResult<()> {
    let module = PyModule::new_bound(m.py(), "state_estimators")?;

    module.add_class::<PythonStateEstimator>()?;
    module.add_class::<ObservationRecord>()?;
    module.add_class::<GNSSObservationRecord>()?;
    module.add_class::<OdometryObservationRecord>()?;
    module.add_class::<OrientedLandmarkObservationRecord>()?;
    module.add_class::<OrientedRobotObservationRecord>()?;
    m.add_submodule(&module)
}

#[derive(Debug, Clone)]
pub struct PythonStateEstimatorAsyncClient {
    pub prediction_step_request: mpsc::Sender<(Robot, f32)>,
    pub prediction_step_response: Arc<Mutex<mpsc::Receiver<()>>>,
    pub correction_step_request: mpsc::Sender<(Robot, Vec<Observation>, f32)>,
    pub correction_step_response: Arc<Mutex<mpsc::Receiver<()>>>,
    pub state_request: mpsc::Sender<()>,
    pub state_response: Arc<Mutex<mpsc::Receiver<State>>>,
    pub next_time_step_request: mpsc::Sender<()>,
    pub next_time_step_response: Arc<Mutex<mpsc::Receiver<f32>>>,
    pub record_request: mpsc::Sender<()>,
    pub record_response: Arc<Mutex<mpsc::Receiver<StateEstimatorRecord>>>,
    pub from_record_request: mpsc::Sender<StateEstimatorRecord>,
    pub from_record_response: Arc<Mutex<mpsc::Receiver<()>>>,
}

impl StateEstimator for PythonStateEstimatorAsyncClient {
    fn prediction_step(&mut self, robot: &mut Robot, time: f32) {
        let _ = self.prediction_step_request.send((robot.clone(), time));
        let _ = self.prediction_step_response.lock().unwrap().recv();
    }

    fn correction_step(&mut self, robot: &mut Robot, observations: &Vec<Observation>, time: f32) {
        let _ = self
            .correction_step_request
            .send((robot.clone(), observations.clone(), time));
        let _ = self.correction_step_response.lock().unwrap().recv();
    }

    fn next_time_step(&self) -> f32 {
        let _ = self.next_time_step_request.send(());
        self.next_time_step_response
            .lock()
            .unwrap()
            .recv()
            .expect("Error during call of next_time_step")
    }

    fn state(&self) -> State {
        let _ = self.state_request.send(());
        self.state_response
            .lock()
            .unwrap()
            .recv()
            .expect("Error during call of state")
    }
}

impl Stateful<StateEstimatorRecord> for PythonStateEstimatorAsyncClient {
    fn from_record(&mut self, record: StateEstimatorRecord) {
        let _ = self.from_record_request.send(record);
        let _ = self.from_record_response.lock().unwrap().recv();
    }

    fn record(&self) -> StateEstimatorRecord {
        let _ = self.record_request.send(());
        self.record_response
            .lock()
            .unwrap()
            .recv()
            .expect("Error during call of record")
    }
}

#[derive(Debug)]
#[pyclass]
pub struct PythonStateEstimator {
    model: Py<PyAny>,
    client: PythonStateEstimatorAsyncClient,
    prediction_step_request: Arc<Mutex<mpsc::Receiver<(Robot, f32)>>>,
    prediction_step_response: mpsc::Sender<()>,
    correction_step_request: Arc<Mutex<mpsc::Receiver<(Robot, Vec<Observation>, f32)>>>,
    correction_step_response: mpsc::Sender<()>,
    state_request: Arc<Mutex<mpsc::Receiver<()>>>,
    state_response: mpsc::Sender<State>,
    next_time_step_request: Arc<Mutex<mpsc::Receiver<()>>>,
    next_time_step_response: mpsc::Sender<f32>,
    record_request: Arc<Mutex<mpsc::Receiver<()>>>,
    record_response: mpsc::Sender<StateEstimatorRecord>,
    from_record_request: Arc<Mutex<mpsc::Receiver<StateEstimatorRecord>>>,
    from_record_response: mpsc::Sender<()>,
}

#[pymethods]
impl PythonStateEstimator {
    #[new]
    pub fn new(py_model: Py<PyAny>) -> PythonStateEstimator {
        Python::with_gil(|py| {
            debug!("Model got: {}", py_model.bind(py).dir().unwrap());
        });
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
        let (from_record_request_tx, from_record_request_rx) = mpsc::channel();
        let (from_record_response_tx, from_record_response_rx) = mpsc::channel();

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
                from_record_request: from_record_request_tx,
                from_record_response: Arc::new(Mutex::new(from_record_response_rx)),
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
            from_record_request: Arc::new(Mutex::new(from_record_request_rx)),
            from_record_response: from_record_response_tx,
        }
    }
}

impl PythonStateEstimator {
    pub fn get_client(&self) -> PythonStateEstimatorAsyncClient {
        self.client.clone()
    }

    pub fn check_requests(&mut self) {
        if let Ok((robot, time)) = self
            .prediction_step_request
            .clone()
            .lock()
            .unwrap()
            .try_recv()
        {
            self.prediction_step(&robot, time);
            let _ = self.prediction_step_response.send(());
        }
        if let Ok((robot, obs, time)) = self
            .correction_step_request
            .clone()
            .lock()
            .unwrap()
            .try_recv()
        {
            self.correction_step(&robot, &obs, time);
            let _ = self.correction_step_response.send(());
        }
        if let Ok(()) = self.state_request.clone().lock().unwrap().try_recv() {
            let _ = self.state_response.send(self.state());
        }
        if let Ok(()) = self
            .next_time_step_request
            .clone()
            .lock()
            .unwrap()
            .try_recv()
        {
            let _ = self.next_time_step_response.send(self.next_time_step());
        }
        if let Ok(()) = self.record_request.clone().lock().unwrap().try_recv() {
            let _ = self.record_response.send(self.record());
        }
        if let Ok(record) = self.from_record_request.clone().lock().unwrap().try_recv() {
            self.from_record(record);
            let _ = self.from_record_response.send(());
        }
    }

    fn prediction_step(&mut self, robot: &crate::robot::Robot, time: f32) {
        debug!("Calling python implementation of prediction_step");
        // let robot_record = robot.record();
        Python::with_gil(|py| {
            self.model
                .bind(py)
                .call_method("prediction_step", (time,), None)
                .expect("Python implementation of PythonStateEstimator does not have a correct 'prediction_step' method");
        });
    }

    fn correction_step(
        &mut self,
        robot: &crate::robot::Robot,
        observations: &Vec<crate::sensors::sensor::Observation>,
        time: f32,
    ) {
        debug!("Calling python implementation of correction_step");
        let mut observation_records = Vec::new();
        for obs in observations {
            observation_records.push(obs.record());
        }
        Python::with_gil(|py| {
            self.model
                .bind(py)
                .call_method("correction_step", (observation_records, time), None)
                .expect("Python implementation of PythonStateEstimator does not have a correct 'correction_step' method");
        });
    }

    fn state(&self) -> State {
        debug!("Calling python implementation of state");
        State::from_vector(Python::with_gil(|py| {
            self.model
                .bind(py)
                .call_method("state", (), None)
                .expect("Python implementation of PythonStateEstimator does not have a correct 'state' method")
                .extract()
                .expect("The 'state' method of PythonStateEstimator does not return a correct state vector")
        }))
    }

    fn next_time_step(&self) -> f32 {
        // PythonStateEstimator::next_time_step(self)
        debug!("Calling python implementation of next_time_step");
        let time = Python::with_gil(|py| {
            self.model
                .bind(py)
                .call_method("next_time_step", (), None)
                .expect("Python implementation of PythonStateEstimator does not have a correct 'next_time_step' method")
                .extract()
                .expect("The 'next_time_step' method of PythonStateEstimator does not return a correct time for next step")
        });
        time
    }

    fn record(&self) -> StateEstimatorRecord {
        debug!("Calling python implementation of record");
        let record_str: String = Python::with_gil(|py| {
            self.model
                .bind(py)
                .call_method("record", (), None)
                .expect("Python implementation of PythonStateEstimator does not have a correct 'record' method")
                .extract()
                .expect("The 'record' method of PythonStateEstimator does not return a valid EstimatorRecord type")
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

    fn from_record(&mut self, record: StateEstimatorRecord) {
        if let StateEstimatorRecord::External(record) = record {
            debug!("Calling python implementation of from_record");
            Python::with_gil(|py| {
                self.model
                    .bind(py)
                    .call_method("from_record", (serde_json::to_string(&record).unwrap(),), None)
                    .expect("Python implementation of PythonStateEstimator does not have a correct 'from_record' method");
            });
        }
    }
}
