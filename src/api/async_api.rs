use std::{
    os::unix::thread::JoinHandleExt,
    path::Path,
    sync::{mpsc, Arc, Mutex},
    thread::{self, sleep, JoinHandle},
    time::Duration,
};

use serde_json::Value;

use crate::{
    controllers::controller::Controller,
    navigators::navigator::Navigator,
    physics::physic::Physic,
    plugin_api::PluginAPI,
    simulator::{Simulator, SimulatorAsyncApi, SimulatorConfig},
    state_estimators::state_estimator::StateEstimator, utils::rfc,
};

// Run by client
#[derive(Clone)]
pub struct AsyncApi {
    pub simulator_api: Arc<SimulatorAsyncApi>,
    // Channels
    pub load_config: rfc::RemoteFunctionCall<String, ()>,
    pub run: rfc::RemoteFunctionCall<Option<f32>, ()>,
    pub compute_results: rfc::RemoteFunctionCall<(), ()>,
}

// Run by the simulator
#[derive(Clone)]
pub struct AsyncApiServer {
    pub load_config: rfc::RemoteFunctionCallHost<String, ()>,
    pub run: rfc::RemoteFunctionCallHost<Option<f32>, ()>,
    pub compute_results: rfc::RemoteFunctionCallHost<(), ()>,
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
        let (load_config_call, load_config_host) = rfc::make_pair();
        let (run_call, run_host) = rfc::make_pair();
        let (results_call, results_host) = rfc::make_pair();
        let (keep_alive_tx, keep_alive_rx) = mpsc::channel();
        let simulator_api = simulator.lock().unwrap().get_async_api();
        Self {
            public_api: AsyncApi {
                simulator_api,
                load_config: load_config_call,
                run: run_call,
                compute_results: results_call,
            },
            private_api: AsyncApiServer {
                load_config: load_config_host,
                run: run_host,
                compute_results: results_host,
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

    pub fn get_simulator(&self) -> Arc<Mutex<Simulator>> {
        self.simulator.clone()
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
                private_api.load_config.try_recv_closure_mut(|config_path| {
                    println!("Loading config: {}", config_path);
                    let path = Path::new(&config_path);
                    simulator.load_config_path(path, &plugin_api);
                    println!("End loading");
                });
                
                private_api.run.try_recv_closure_mut(|max_time| {
                    if need_reset {
                        simulator.reset(&plugin_api);
                    }
                    if let Some(max_time) = max_time {
                        simulator.set_max_time(max_time);
                    }
                    simulator.run();
                    need_reset = true;
                });
                private_api.compute_results.try_recv_closure_mut(|_| {
                    simulator.compute_results();
                    need_reset = true;
                });
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
    pub get_controller_request: mpsc::Sender<(Value, SimulatorConfig)>,
    pub get_controller_response: Arc<Mutex<mpsc::Receiver<Box<dyn Controller>>>>,
    pub get_navigator_request: mpsc::Sender<(Value, SimulatorConfig)>,
    pub get_navigator_response: Arc<Mutex<mpsc::Receiver<Box<dyn Navigator>>>>,
    pub get_physic_request: mpsc::Sender<(Value, SimulatorConfig)>,
    pub get_physic_response: Arc<Mutex<mpsc::Receiver<Box<dyn Physic>>>>,
}

impl PluginAsyncAPI {
    pub fn new() -> PluginAsyncAPI {
        let (get_state_estimator_request_tx, get_state_estimator_request_rx) = mpsc::channel();
        let (get_state_estimator_response_tx, get_state_estimator_response_rx) = mpsc::channel();
        let (get_controller_request_tx, get_controller_request_rx) = mpsc::channel();
        let (get_controller_response_tx, get_controller_response_rx) = mpsc::channel();
        let (get_navigator_request_tx, get_navigator_request_rx) = mpsc::channel();
        let (get_navigator_response_tx, get_navigator_response_rx) = mpsc::channel();
        let (get_physic_request_tx, get_physic_request_rx) = mpsc::channel();
        let (get_physic_response_tx, get_physic_response_rx) = mpsc::channel();

        PluginAsyncAPI {
            client: PluginAsyncAPIClient {
                get_state_estimator_request: Arc::new(Mutex::new(get_state_estimator_request_rx)),
                get_state_estimator_response: get_state_estimator_response_tx,
                get_controller_request: Arc::new(Mutex::new(get_controller_request_rx)),
                get_controller_response: get_controller_response_tx,
                get_navigator_request: Arc::new(Mutex::new(get_navigator_request_rx)),
                get_navigator_response: get_navigator_response_tx,
                get_physic_request: Arc::new(Mutex::new(get_physic_request_rx)),
                get_physic_response: get_physic_response_tx,
            },
            get_state_estimator_request: get_state_estimator_request_tx,
            get_state_estimator_response: Arc::new(Mutex::new(get_state_estimator_response_rx)),
            get_controller_request: get_controller_request_tx,
            get_controller_response: Arc::new(Mutex::new(get_controller_response_rx)),
            get_navigator_request: get_navigator_request_tx,
            get_navigator_response: Arc::new(Mutex::new(get_navigator_response_rx)),
            get_physic_request: get_physic_request_tx,
            get_physic_response: Arc::new(Mutex::new(get_physic_response_rx)),
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

    fn get_controller(
        &self,
        config: &Value,
        global_config: &SimulatorConfig,
    ) -> Box<dyn Controller> {
        self.get_controller_request
            .send((config.clone(), global_config.clone()))
            .unwrap();

        self.get_controller_response.lock().unwrap().recv().unwrap()
    }

    fn get_navigator(&self, config: &Value, global_config: &SimulatorConfig) -> Box<dyn Navigator> {
        self.get_navigator_request
            .send((config.clone(), global_config.clone()))
            .unwrap();

        self.get_navigator_response.lock().unwrap().recv().unwrap()
    }

    fn get_physic(&self, config: &Value, global_config: &SimulatorConfig) -> Box<dyn Physic> {
        self.get_physic_request
            .send((config.clone(), global_config.clone()))
            .unwrap();

        self.get_physic_response.lock().unwrap().recv().unwrap()
    }
}

#[derive(Clone)]
pub struct PluginAsyncAPIClient {
    pub get_state_estimator_request: Arc<Mutex<mpsc::Receiver<(Value, SimulatorConfig)>>>,
    pub get_state_estimator_response: mpsc::Sender<Box<dyn StateEstimator>>,
    pub get_controller_request: Arc<Mutex<mpsc::Receiver<(Value, SimulatorConfig)>>>,
    pub get_controller_response: mpsc::Sender<Box<dyn Controller>>,
    pub get_navigator_request: Arc<Mutex<mpsc::Receiver<(Value, SimulatorConfig)>>>,
    pub get_navigator_response: mpsc::Sender<Box<dyn Navigator>>,
    pub get_physic_request: Arc<Mutex<mpsc::Receiver<(Value, SimulatorConfig)>>>,
    pub get_physic_response: mpsc::Sender<Box<dyn Physic>>,
}
