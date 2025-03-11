use std::{
    os::unix::{raw::pthread_t, thread::JoinHandleExt},
    path::Path,
    sync::{mpsc, Arc, Mutex},
    thread::{self, sleep, JoinHandle},
    time::Duration,
};

use serde_json::Value;

use crate::{
    plugin_api::{self, PluginAPI},
    simulator::{self, Simulator, SimulatorAsyncApi, SimulatorConfig},
    state_estimators::state_estimator::StateEstimator,
};

// Run by client
#[derive(Clone)]
pub struct AsyncApi {
    pub simulator_api: Arc<SimulatorAsyncApi>,
    // Channels
    pub load_config: mpsc::Sender<String>,
    pub run: mpsc::Sender<Option<f32>>,
    pub ended: Arc<Mutex<mpsc::Receiver<()>>>,
}

// Run by the simulator
#[derive(Clone)]
pub struct AsyncApiServer {
    pub load_config: Arc<Mutex<mpsc::Receiver<String>>>,
    pub run: Arc<Mutex<mpsc::Receiver<Option<f32>>>>,
    pub ended: mpsc::Sender<()>,
}

// #[derive(Clone)]
pub struct AsyncApiRunner {
    public_api: AsyncApi,
    private_api: AsyncApiServer,
    simulator: Arc<Mutex<Simulator>>,
    keep_alive_tx: mpsc::Sender<()>,
    keep_alive_rx: Arc<Mutex<mpsc::Receiver<()>>>,
    thread_handle: Option<JoinHandle<()>>,
}

impl AsyncApiRunner {
    pub fn new() -> Self {
        let simulator = Arc::new(Mutex::new(Simulator::new()));
        AsyncApiRunner::new_with_simulator(simulator)
    }

    pub fn new_with_simulator(simulator: Arc<Mutex<Simulator>>) -> Self {
        let (load_config_tx, load_config_rx) = mpsc::channel();
        let (run_tx, run_rx) = mpsc::channel();
        let (keep_alive_tx, keep_alive_rx) = mpsc::channel();
        let (ended_tx, ended_rx) = mpsc::channel();
        let simulator_api = simulator.lock().unwrap().get_async_api();
        Self {
            public_api: AsyncApi {
                simulator_api,
                load_config: load_config_tx,
                run: run_tx,
                ended: Arc::new(Mutex::new(ended_rx)),
            },
            private_api: AsyncApiServer {
                load_config: Arc::new(Mutex::new(load_config_rx)),
                run: Arc::new(Mutex::new(run_rx)),
                ended: ended_tx,
            },
            simulator,
            keep_alive_rx: Arc::new(Mutex::new(keep_alive_rx)),
            keep_alive_tx,
            thread_handle: None,
        }
    }

    pub fn get_api(&self) -> AsyncApi {
        self.public_api.clone()
    }

    pub fn stop(&mut self) {
        self.keep_alive_tx.send(()).unwrap();
        log::info!("Stop requested...");
        if let Some(handle) = self.thread_handle.take() {
            sleep(Duration::new(0, 200000000));
            if !handle.is_finished() {
                unsafe {
                    libc::pthread_cancel(handle.as_pthread_t());
                }
            }
            handle
                .join()
                .expect("Error while waiting for server thread to join");
        }
    }

    pub fn run(&mut self, plugin_api: Option<Box<&'static dyn PluginAPI>>) {
        let private_api = self.private_api.clone();
        let keep_alive_rx = self.keep_alive_rx.clone();
        let simulator_arc = self.simulator.clone();
        let plugin_api = plugin_api.clone();
        self.thread_handle = Some(thread::spawn(move || {
            let plugin_api = plugin_api.clone();
            let mut simulator = simulator_arc.lock().unwrap();
            let mut need_reset = false;
            loop {
                // Should not be blocking
                if let Ok(_) = keep_alive_rx.lock().unwrap().try_recv() {
                    break;
                }
                if let Ok(config_path) = private_api.load_config.lock().unwrap().try_recv() {
                    println!("Loading config: {}", config_path);
                    let path = Path::new(&config_path);
                    simulator.load_config_path(path, &plugin_api);
                    println!("End loading");
                    private_api.ended.send(()).unwrap();
                }
                if let Ok(max_time) = private_api.run.lock().unwrap().try_recv() {
                    if need_reset {
                        simulator.reset(&plugin_api);
                    }
                    if let Some(max_time) = max_time {
                        simulator.set_max_time(max_time);
                    }
                    simulator.run();
                    need_reset = true;
                    private_api
                        .ended
                        .send(())
                        .expect("Error during sending 'end' information");
                }
            }
            log::info!("AsyncApiRunner thread exited");
        }));
    }
}

#[derive(Clone)]
pub struct PluginAsyncAPI {
    pub client: PluginAsyncAPIClient,
    pub get_state_estimator_request: mpsc::Sender<(Value, SimulatorConfig)>,
    pub get_state_estimator_response: Arc<Mutex<mpsc::Receiver<Box<dyn StateEstimator>>>>,
}

impl PluginAsyncAPI {
    pub fn new() -> PluginAsyncAPI {
        let (get_state_estimator_request_tx, get_state_estimator_request_rx) = mpsc::channel();
        let (get_state_estimator_respose_tx, get_state_estimator_respose_rx) = mpsc::channel();

        PluginAsyncAPI {
            client: PluginAsyncAPIClient {
                get_state_estimator_request: Arc::new(Mutex::new(get_state_estimator_request_rx)),
                get_state_estimator_response: get_state_estimator_respose_tx,
            },
            get_state_estimator_request: get_state_estimator_request_tx,
            get_state_estimator_response: Arc::new(Mutex::new(get_state_estimator_respose_rx)),
        }
    }
}

impl PluginAPI for PluginAsyncAPI {
    fn get_state_estimator(
        &self,
        config: &Value,
        global_config: &SimulatorConfig,
    ) -> Box<dyn StateEstimator> {
        self.get_state_estimator_request
            .send((config.clone(), global_config.clone()))
            .unwrap();

        self.get_state_estimator_response
            .lock()
            .unwrap()
            .recv()
            .unwrap()
    }
}

#[derive(Clone)]
pub struct PluginAsyncAPIClient {
    pub get_state_estimator_request: Arc<Mutex<mpsc::Receiver<(Value, SimulatorConfig)>>>,
    pub get_state_estimator_response: mpsc::Sender<Box<dyn StateEstimator>>,
}
