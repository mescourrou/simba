use std::sync::{Arc, Mutex, RwLock, mpsc};

use log::debug;
use pyo3::Python;

use crate::{
    api::async_api::{
        AsyncApi, AsyncApiLoadConfigRequest, AsyncApiRunRequest, AsyncApiRunner, PluginAsyncAPI,
    },
    errors::SimbaResult,
    logger::is_enabled,
    plugin_api::PluginAPI,
    simulator::{Record, Simulator},
};

pub struct AsyncSimulator {
    server: Arc<Mutex<AsyncApiRunner>>,
    api: AsyncApi,
    async_plugin_api: Option<Arc<PluginAsyncAPI>>,
    // python_api: Option<PythonAPI>,
}

impl AsyncSimulator {
    pub fn from_config(
        config_path: String,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
    ) -> SimbaResult<AsyncSimulator> {
        Simulator::init_environment();

        let server = Arc::new(Mutex::new(AsyncApiRunner::new()));
        let api = server.lock().unwrap().get_api();
        let sim = Self {
            server,
            api,
            async_plugin_api: plugin_api.as_ref().map(|_| Arc::new(PluginAsyncAPI::new())),
        };

        sim.server.lock().unwrap().run(
            sim.async_plugin_api
                .clone()
                .map(|api| api as Arc<dyn PluginAPI>),
        );
        sim.api.load_config.async_call(AsyncApiLoadConfigRequest {
            config_path,
            force_send_results: false,
        });

        if let Some(unwrapped_async_api) = &sim.async_plugin_api {
            let api_client = &unwrapped_async_api.get_client();
            let plugin_api_unwrapped = plugin_api.as_ref().unwrap();
            let mut res = sim.api.load_config.try_get_result();
            while res.is_none() {
                api_client.get_state_estimator.try_recv_closure(|request| {
                    plugin_api_unwrapped.get_state_estimator(
                        &request.config,
                        &request.global_config,
                        &request.va_factory,
                        0.,
                    )
                });
                api_client.get_controller.try_recv_closure(|request| {
                    plugin_api_unwrapped.get_controller(
                        &request.config,
                        &request.global_config,
                        &request.va_factory,
                        0.,
                    )
                });
                api_client.get_navigator.try_recv_closure(|request| {
                    plugin_api_unwrapped.get_navigator(
                        &request.config,
                        &request.global_config,
                        &request.va_factory,
                        0.,
                    )
                });
                api_client.get_physics.try_recv_closure(|request| {
                    plugin_api_unwrapped.get_physics(
                        &request.config,
                        &request.global_config,
                        &request.va_factory,
                        0.,
                    )
                });
                plugin_api_unwrapped.check_requests();
                res = sim.api.load_config.try_get_result();
            }
            res.unwrap()?;
        } else {
            sim.api.load_config.wait_result().unwrap()?;
        }

        Ok(sim)
    }

    pub fn run(
        &mut self,
        plugin_api: &Option<Arc<dyn PluginAPI>>,
        max_time: Option<f32>,
        reset: bool,
    ) {
        self.api
            .run
            .async_call(AsyncApiRunRequest { max_time, reset });
        if let Some(plugin_api) = plugin_api {
            while self.api.run.try_get_result().is_none() {
                plugin_api.check_requests();
                if Python::attach(|py| py.check_signals()).is_err() {
                    break;
                }
            }
        } else {
            while self.api.run.try_get_result().is_none() {
                if Python::attach(|py| py.check_signals()).is_err() {
                    break;
                }
            }
        }
    }

    pub fn get_records(&self, sorted: bool) -> SimbaResult<Vec<Record>> {
        self.api.get_records.call(sorted).unwrap()
    }

    pub fn compute_results(&self) {
        // Calling directly the simulator to keep python in one thread
        self.server
            .lock()
            .unwrap()
            .get_simulator()
            .lock()
            .unwrap()
            .compute_results()
            .unwrap();
    }

    pub fn stop(&self) {
        if is_enabled(crate::logger::InternalLog::API) {
            debug!("Stop server");
        }
        // Stop server thread
        self.server.lock().unwrap().stop();
    }

    pub fn show(&self) {
        self.server
            .lock()
            .unwrap()
            .get_simulator()
            .lock()
            .unwrap()
            .show();
    }
}

pub struct SimulatorAsyncApi {
    pub current_time: Arc<RwLock<f32>>,
    pub records: Arc<Mutex<mpsc::Receiver<Record>>>,
}

#[derive(Clone)]
pub(super) struct SimulatorAsyncApiServer {
    current_time: Arc<RwLock<f32>>,
    records: Vec<mpsc::Sender<Record>>,
}

impl SimulatorAsyncApiServer {
    pub fn new(time: f32) -> Self {
        Self {
            current_time: Arc::new(RwLock::new(time)),
            records: Vec::new(),
        }
    }

    pub fn new_client(&mut self) -> SimulatorAsyncApi {
        let (tx, rx) = mpsc::channel();
        self.records.push(tx);
        SimulatorAsyncApi {
            current_time: self.current_time.clone(),
            records: Arc::new(Mutex::new(rx)),
        }
    }

    pub fn update_time(&self, new_time: f32) {
        *self.current_time.write().unwrap() = new_time;
    }

    pub fn send_record(&self, record: &Record) {
        for tx in &self.records {
            tx.send(record.clone()).unwrap();
        }
    }
}
