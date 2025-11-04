use std::{
    os::unix::thread::JoinHandleExt,
    path::Path,
    sync::{mpsc, Arc, Mutex, RwLock},
    thread::{self, sleep, JoinHandle},
    time::Duration,
};

use serde_json::Value;

use crate::{
    controllers::controller::Controller,
    errors::SimbaResult,
    navigators::navigator::Navigator,
    physics::physics::Physics,
    plugin_api::PluginAPI,
    simulator::{Simulator, SimulatorAsyncApi, SimulatorConfig},
    state_estimators::state_estimator::StateEstimator,
    utils::rfc,
};

// Run by client
#[derive(Clone)]
pub struct AsyncApi {
    pub simulator_api: Arc<SimulatorAsyncApi>,
    // Channels
    pub load_config: rfc::RemoteFunctionCall<String, SimbaResult<SimulatorConfig>>,
    pub load_results: rfc::RemoteFunctionCall<(), SimbaResult<f32>>,
    pub run: rfc::RemoteFunctionCall<Option<f32>, SimbaResult<()>>,
    pub compute_results: rfc::RemoteFunctionCall<(), SimbaResult<()>>,
}

// Run by the simulator
#[derive(Clone)]
pub struct AsyncApiServer {
    pub load_config: Arc<rfc::RemoteFunctionCallHost<String, SimbaResult<SimulatorConfig>>>,
    pub load_results: Arc<rfc::RemoteFunctionCallHost<(), SimbaResult<f32>>>,
    pub run: Arc<rfc::RemoteFunctionCallHost<Option<f32>, SimbaResult<()>>>,
    pub compute_results: Arc<rfc::RemoteFunctionCallHost<(), SimbaResult<()>>>,
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
        let (load_results_call, load_results_host) = rfc::make_pair();
        let (keep_alive_tx, keep_alive_rx) = mpsc::channel();
        let simulator_api = simulator.lock().unwrap().get_async_api();
        Self {
            public_api: AsyncApi {
                simulator_api,
                load_config: load_config_call,
                load_results: load_results_call,
                run: run_call,
                compute_results: results_call,
            },
            private_api: AsyncApiServer {
                load_config: Arc::new(load_config_host),
                load_results: Arc::new(load_results_host),
                run: Arc::new(run_host),
                compute_results: Arc::new(results_host),
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
        let simulator_cloned = self.simulator.clone();
        let plugin_api = plugin_api.clone();
        self.thread_handle = Some(thread::spawn(move || {
            println!("AsyncApiRunner thread started");
            let plugin_api = plugin_api.clone();
            let mut need_reset = false;

            let load_config = private_api.load_config.clone();
            let simulator_arc = simulator_cloned.clone();
            let plugin_api_threaded = plugin_api.clone();

            let stopping_root = Arc::new(RwLock::new(false));
            let stopping = stopping_root.clone();
            thread::spawn(move || {
                while !*stopping.read().unwrap() {
                    load_config.recv_closure_mut(|config_path| {
                        let mut simulator = simulator_arc.lock().unwrap();
                        println!("Loading config: {}", config_path);
                        let path = Path::new(&config_path);
                        simulator.load_config_path(path, &plugin_api_threaded)?;
                        println!("End loading");
                        Ok(simulator.config())
                    });
                }
            });

            let stopping = stopping_root.clone();
            let load_results = private_api.load_results.clone();
            let simulator_arc = simulator_cloned.clone();
            thread::spawn(move || {
                while !*stopping.read().unwrap() {
                    load_results.recv_closure(|()| {
                        let mut simulator = simulator_arc.lock().unwrap();
                        simulator.load_results()
                    });
                }
            });

            let stopping = stopping_root.clone();
            let run = private_api.run.clone();
            let simulator_arc = simulator_cloned.clone();
            let plugin_api_threaded = plugin_api.clone();
            thread::spawn(move || {
                while !*stopping.read().unwrap() {
                    run.recv_closure_mut(|max_time| {
                        let mut simulator = simulator_arc.lock().unwrap();
                        if need_reset {
                            simulator.reset(&plugin_api_threaded)?;
                        }
                        if let Some(max_time) = max_time {
                            simulator.set_max_time(max_time);
                        }
                        log::info!("Run...");
                        need_reset = true;
                        simulator.run()
                    });
                }
            });

            let compute_results = private_api.compute_results.clone();
            let simulator_arc = simulator_cloned.clone();
            let stopping = stopping_root.clone();
            thread::spawn(move || {
                while !*stopping.read().unwrap() {
                    compute_results.recv_closure_mut(|_| {
                        let simulator = simulator_arc.lock().unwrap();
                        need_reset = true;
                        simulator.compute_results()
                    });
                }
            });

            // Wait for end
            let _ = keep_alive_rx.lock().unwrap().recv();

            // Ending threads
            *stopping_root.write().unwrap() = true;
            private_api.load_config.stop_recv();
            private_api.run.stop_recv();
            private_api.compute_results.stop_recv();

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
    pub get_physic_response: Arc<Mutex<mpsc::Receiver<Box<dyn Physics>>>>,
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
                get_physics_request: Arc::new(Mutex::new(get_physic_request_rx)),
                get_physics_response: get_physic_response_tx,
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

    fn get_physics(&self, config: &Value, global_config: &SimulatorConfig) -> Box<dyn Physics> {
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
    pub get_physics_request: Arc<Mutex<mpsc::Receiver<(Value, SimulatorConfig)>>>,
    pub get_physics_response: mpsc::Sender<Box<dyn Physics>>,
}
