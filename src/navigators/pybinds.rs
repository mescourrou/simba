use std::{
    str::FromStr,
    sync::{mpsc, Arc, Mutex},
};

use log::debug;
use pyo3::prelude::*;
use serde_json::Value;

use crate::{
    controllers::controller::ControllerError,
    logger::is_enabled,
    navigators::external_navigator::ExternalNavigatorRecord,
    node::Node,
    pywrappers::{ControllerErrorWrapper, WorldStateWrapper},
    state_estimators::state_estimator::WorldState,
    stateful::Stateful,
};

use super::navigator::{Navigator, NavigatorRecord};

#[derive(Debug, Clone)]
pub struct PythonNavigatorAsyncClient {
    pub compute_error_request: mpsc::Sender<WorldState>,
    pub compute_error_response: Arc<Mutex<mpsc::Receiver<ControllerError>>>,
    pub record_request: mpsc::Sender<()>,
    pub record_response: Arc<Mutex<mpsc::Receiver<NavigatorRecord>>>,
    pub from_record_request: mpsc::Sender<NavigatorRecord>,
    pub from_record_response: Arc<Mutex<mpsc::Receiver<()>>>,
}

impl Navigator for PythonNavigatorAsyncClient {
    fn compute_error(&mut self, _node: &mut Node, world_state: WorldState) -> ControllerError {
        self.compute_error_request
            .send(world_state.clone())
            .unwrap();
        self.compute_error_response.lock().unwrap().recv().unwrap()
    }
}

impl Stateful<NavigatorRecord> for PythonNavigatorAsyncClient {
    fn from_record(&mut self, record: NavigatorRecord) {
        self.from_record_request.send(record).unwrap();
        self.from_record_response.lock().unwrap().recv().unwrap();
    }

    fn record(&self) -> NavigatorRecord {
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
#[pyo3(name = "Navigator")]
pub struct PythonNavigator {
    model: Py<PyAny>,
    client: PythonNavigatorAsyncClient,
    compute_error_request: Arc<Mutex<mpsc::Receiver<WorldState>>>,
    compute_error_response: mpsc::Sender<ControllerError>,
    record_request: Arc<Mutex<mpsc::Receiver<()>>>,
    record_response: mpsc::Sender<NavigatorRecord>,
    from_record_request: Arc<Mutex<mpsc::Receiver<NavigatorRecord>>>,
    from_record_response: mpsc::Sender<()>,
}

#[pymethods]
impl PythonNavigator {
    #[new]
    pub fn new(py_model: Py<PyAny>) -> PythonNavigator {
        if is_enabled(crate::logger::InternalLog::API) {
            Python::with_gil(|py| {
                debug!("Model got: {}", py_model.bind(py).dir().unwrap());
            });
        }
        let (compute_error_request_tx, compute_error_request_rx) = mpsc::channel();
        let (compute_error_response_tx, compute_error_response_rx) = mpsc::channel();
        let (record_request_tx, record_request_rx) = mpsc::channel();
        let (record_response_tx, record_response_rx) = mpsc::channel();
        let (from_record_request_tx, from_record_request_rx) = mpsc::channel();
        let (from_record_response_tx, from_record_response_rx) = mpsc::channel();

        PythonNavigator {
            model: py_model,
            client: PythonNavigatorAsyncClient {
                compute_error_request: compute_error_request_tx,
                compute_error_response: Arc::new(Mutex::new(compute_error_response_rx)),
                record_request: record_request_tx,
                record_response: Arc::new(Mutex::new(record_response_rx)),
                from_record_request: from_record_request_tx,
                from_record_response: Arc::new(Mutex::new(from_record_response_rx)),
            },
            compute_error_request: Arc::new(Mutex::new(compute_error_request_rx)),
            compute_error_response: compute_error_response_tx,
            record_request: Arc::new(Mutex::new(record_request_rx)),
            record_response: record_response_tx,
            from_record_request: Arc::new(Mutex::new(from_record_request_rx)),
            from_record_response: from_record_response_tx,
        }
    }
}

impl PythonNavigator {
    pub fn get_client(&self) -> PythonNavigatorAsyncClient {
        self.client.clone()
    }

    pub fn check_requests(&mut self) {
        if let Ok(state) = self
            .compute_error_request
            .clone()
            .lock()
            .unwrap()
            .try_recv()
        {
            let error = self.compute_error(&state);
            self.compute_error_response.send(error).unwrap();
        }
        if let Ok(()) = self.record_request.clone().lock().unwrap().try_recv() {
            self.record_response.send(self.record()).unwrap();
        }
        if let Ok(record) = self.from_record_request.clone().lock().unwrap().try_recv() {
            self.from_record(record);
            self.from_record_response.send(()).unwrap();
        }
    }

    fn compute_error(&mut self, state: &WorldState) -> ControllerError {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of compute_error");
        }
        // let node_record = node.record();
        let result = Python::with_gil(|py| -> ControllerErrorWrapper {
            self.model
                .bind(py)
                .call_method(
                    "compute_error",
                    (WorldStateWrapper::from_rust(state),),
                    None,
                )
                .expect("PythonNavigator does not have a correct 'compute_error' method")
                .extract()
                .expect("Error during the call of Python implementation of 'compute_error'")
        });
        result.to_rust()
    }

    fn record(&self) -> NavigatorRecord {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Calling python implementation of record");
        }
        let record_str: String = Python::with_gil(|py| {
            self.model
                .bind(py)
                .call_method("record", (), None)
                .expect("Python implementation of PythonNavigator does not have a correct 'record' method")
                .extract()
                .expect("The 'record' method of PythonNavigator does not return a valid EstimatorRecord type")
        });
        let record = ExternalNavigatorRecord {
            record: Value::from_str(record_str.as_str()).expect(
                "Impossible to get serde_json::Value from the input serialized python structure",
            ),
        };
        // record.clone()
        // StateEstimatorRecord::External(PythonNavigator::record(&self))
        NavigatorRecord::External(record)
    }

    fn from_record(&mut self, record: NavigatorRecord) {
        if let NavigatorRecord::External(record) = record {
            if is_enabled(crate::logger::InternalLog::API) {
                debug!("Calling python implementation of from_record");
            }
            Python::with_gil(|py| {
                self.model
                    .bind(py)
                    .call_method("from_record", (serde_json::to_string(&record).unwrap(),), None)
                    .expect("Python implementation of PythonNavigator does not have a correct 'from_record' method");
            });
        }
    }
}
