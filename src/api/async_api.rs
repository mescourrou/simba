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
    physics::Physics,
    plugin_api::PluginAPI,
    simulator::{Record, Simulator, SimulatorAsyncApi, SimulatorConfig},
    state_estimators::state_estimator::StateEstimator,
    utils::{
        determinist_random_variable::DeterministRandomVariableFactory,
        rfc::{self, RemoteFunctionCall, RemoteFunctionCallHost},
    },
};

// Run by client
#[derive(Clone)]
pub struct AsyncApi {
    pub simulator_api: Arc<SimulatorAsyncApi>,
    // Channels
    pub load_config:
        rfc::RemoteFunctionCall<AsyncApiLoadConfigRequest, SimbaResult<SimulatorConfig>>,
    pub load_results: rfc::RemoteFunctionCall<(), SimbaResult<f32>>,
    pub run: rfc::RemoteFunctionCall<AsyncApiRunRequest, SimbaResult<()>>,
    pub get_records: rfc::RemoteFunctionCall<bool, SimbaResult<Vec<Record>>>,
    pub compute_results: rfc::RemoteFunctionCall<(), SimbaResult<()>>,
}

// Run by the simulator
#[derive(Clone)]
pub struct AsyncApiServer {
    pub load_config:
        Arc<rfc::RemoteFunctionCallHost<AsyncApiLoadConfigRequest, SimbaResult<SimulatorConfig>>>,
    pub load_results: Arc<rfc::RemoteFunctionCallHost<(), SimbaResult<f32>>>,
    pub run: Arc<rfc::RemoteFunctionCallHost<AsyncApiRunRequest, SimbaResult<()>>>,
    pub compute_results: Arc<rfc::RemoteFunctionCallHost<(), SimbaResult<()>>>,
    pub get_records: Arc<rfc::RemoteFunctionCallHost<bool, SimbaResult<Vec<Record>>>>,
}

// #[derive(Clone)]
pub struct AsyncApiRunner {
    public_api: AsyncApi,
    private_api: AsyncApiServer,
    simulator: Arc<Mutex<Simulator>>,
    keep_alive_tx: mpsc::Sender<()>,
    keep_alive_rx: Arc<Mutex<mpsc::Receiver<()>>>,
    thread_handle: Option<JoinHandle<()>>,
    running: bool,
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
        let (get_records_call, get_records_host) = rfc::make_pair();
        let (keep_alive_tx, keep_alive_rx) = mpsc::channel();
        let simulator_api = simulator.lock().unwrap().get_async_api();
        Self {
            public_api: AsyncApi {
                simulator_api,
                load_config: load_config_call,
                load_results: load_results_call,
                run: run_call,
                compute_results: results_call,
                get_records: get_records_call,
            },
            private_api: AsyncApiServer {
                load_config: Arc::new(load_config_host),
                load_results: Arc::new(load_results_host),
                run: Arc::new(run_host),
                compute_results: Arc::new(results_host),
                get_records: Arc::new(get_records_host),
            },
            simulator,
            keep_alive_rx: Arc::new(Mutex::new(keep_alive_rx)),
            keep_alive_tx,
            thread_handle: None,
            running: true,
        }
    }

    pub fn get_api(&self) -> AsyncApi {
        self.public_api.clone()
    }

    pub fn get_simulator(&self) -> Arc<Mutex<Simulator>> {
        self.simulator.clone()
    }

    pub fn stop(&mut self) {
        if !self.running {
            return;
        }
        self.keep_alive_tx.send(()).unwrap();
        log::info!("Stop requested...");
        if let Some(handle) = self.thread_handle.take() {
            sleep(Duration::new(0, 200000000));
            if !handle.is_finished() {
                // Need to find a generic way to stop threads
                log::warn!("Force stopping AsyncApiRunner thread...");
                unsafe {
                    libc::pthread_cancel(handle.as_pthread_t());
                }
            }
            handle
                .join()
                .expect("Error while waiting for server thread to join");
        }
        self.running = false;
    }

    pub fn run(&mut self, plugin_api: Option<Arc<dyn PluginAPI>>) {
        let private_api = self.private_api.clone();
        let keep_alive_rx = self.keep_alive_rx.clone();
        let simulator_cloned = self.simulator.clone();
        let plugin_api = Arc::new(plugin_api);
        self.running = true;
        self.thread_handle = Some(thread::spawn(move || {
            println!("AsyncApiRunner thread started");
            let mut need_reset = false;

            let load_config = private_api.load_config.clone();
            let simulator_arc = simulator_cloned.clone();
            let plugin_api_threaded = plugin_api.clone();

            let stopping_root = Arc::new(RwLock::new(false));
            let stopping = stopping_root.clone();
            thread::spawn(move || {
                while !*stopping.read().unwrap() {
                    load_config.recv_closure_mut(|request| {
                        let mut simulator = simulator_arc.lock().unwrap();
                        println!("Loading config: {}", request.config_path);
                        let path = Path::new(&request.config_path);
                        simulator.load_config_path_full(
                            path,
                            &plugin_api_threaded,
                            request.force_send_results,
                        )?;
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
                    run.recv_closure_mut(|request| {
                        let mut simulator = simulator_arc.lock().unwrap();
                        if request.reset {
                            simulator.reset(&plugin_api_threaded)?;
                        }
                        if let Some(max_time) = request.max_time {
                            simulator.set_max_time(max_time);
                        }
                        log::info!("Run...");
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

            let get_records = private_api.get_records.clone();
            let simulator_arc = simulator_cloned.clone();
            let stopping = stopping_root.clone();
            thread::spawn(move || {
                while !*stopping.read().unwrap() {
                    get_records.recv_closure_mut(|sort| {
                        let simulator = simulator_arc.lock().unwrap();
                        Ok(simulator.get_records(sort))
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

impl Default for AsyncApiRunner {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for AsyncApiRunner {
    fn drop(&mut self) {
        self.stop();
    }
}

#[derive(Clone)]
pub struct PluginAsyncAPI {
    client: PluginAsyncAPIClient,
    get_state_estimator:
        Arc<RemoteFunctionCall<PluginAsyncAPIGetStateEstimatorRequest, Box<dyn StateEstimator>>>,
    get_controller:
        Arc<RemoteFunctionCall<PluginAsyncAPIGetControllerRequest, Box<dyn Controller>>>,
    get_navigator: Arc<RemoteFunctionCall<PluginAsyncAPIGetNavigatorRequest, Box<dyn Navigator>>>,
    get_physics: Arc<RemoteFunctionCall<PluginAsyncAPIGetPhysicsRequest, Box<dyn Physics>>>,
}

impl PluginAsyncAPI {
    pub fn new() -> PluginAsyncAPI {
        let (get_state_estimator_client, get_state_estimator_host) = rfc::make_pair();
        let (get_controller_client, get_controller_host) = rfc::make_pair();
        let (get_navigator_client, get_navigator_host) = rfc::make_pair();
        let (get_physics_client, get_physics_host) = rfc::make_pair();

        PluginAsyncAPI {
            client: PluginAsyncAPIClient {
                get_state_estimator: Arc::new(get_state_estimator_host),
                get_controller: Arc::new(get_controller_host),
                get_navigator: Arc::new(get_navigator_host),
                get_physics: Arc::new(get_physics_host),
            },
            get_state_estimator: Arc::new(get_state_estimator_client),
            get_controller: Arc::new(get_controller_client),
            get_navigator: Arc::new(get_navigator_client),
            get_physics: Arc::new(get_physics_client),
        }
    }

    pub fn get_client(&self) -> PluginAsyncAPIClient {
        self.client.clone()
    }
}

impl Default for PluginAsyncAPI {
    fn default() -> Self {
        Self::new()
    }
}

impl PluginAPI for PluginAsyncAPI {
    fn get_state_estimator(
        &self,
        config: &Value,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn StateEstimator> {
        self.get_state_estimator
            .call(PluginAsyncAPIGetStateEstimatorRequest {
                config: config.clone(),
                global_config: global_config.clone(),
                va_factory: va_factory.clone(),
            })
            .unwrap()
    }

    fn get_controller(
        &self,
        config: &Value,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn Controller> {
        self.get_controller
            .call(PluginAsyncAPIGetControllerRequest {
                config: config.clone(),
                global_config: global_config.clone(),
                va_factory: va_factory.clone(),
            })
            .unwrap()
    }

    fn get_navigator(
        &self,
        config: &Value,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn Navigator> {
        self.get_navigator
            .call(PluginAsyncAPIGetNavigatorRequest {
                config: config.clone(),
                global_config: global_config.clone(),
                va_factory: va_factory.clone(),
            })
            .unwrap()
    }

    fn get_physics(
        &self,
        config: &Value,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
    ) -> Box<dyn Physics> {
        self.get_physics
            .call(PluginAsyncAPIGetPhysicsRequest {
                config: config.clone(),
                global_config: global_config.clone(),
                va_factory: va_factory.clone(),
            })
            .unwrap()
    }
}

#[derive(Clone)]
pub struct PluginAsyncAPIClient {
    pub get_state_estimator: Arc<
        RemoteFunctionCallHost<PluginAsyncAPIGetStateEstimatorRequest, Box<dyn StateEstimator>>,
    >,
    pub get_controller:
        Arc<RemoteFunctionCallHost<PluginAsyncAPIGetControllerRequest, Box<dyn Controller>>>,
    pub get_navigator:
        Arc<RemoteFunctionCallHost<PluginAsyncAPIGetNavigatorRequest, Box<dyn Navigator>>>,
    pub get_physics: Arc<RemoteFunctionCallHost<PluginAsyncAPIGetPhysicsRequest, Box<dyn Physics>>>,
}

pub struct PluginAsyncAPIGetStateEstimatorRequest {
    pub config: Value,
    pub global_config: SimulatorConfig,
    pub va_factory: Arc<DeterministRandomVariableFactory>,
}

pub type PluginAsyncAPIGetControllerRequest = PluginAsyncAPIGetStateEstimatorRequest;
pub type PluginAsyncAPIGetNavigatorRequest = PluginAsyncAPIGetStateEstimatorRequest;
pub type PluginAsyncAPIGetPhysicsRequest = PluginAsyncAPIGetStateEstimatorRequest;

#[derive(Clone, Debug, Default)]
pub struct AsyncApiLoadConfigRequest {
    pub config_path: String,
    pub force_send_results: bool,
}

#[derive(Clone, Debug, Default)]
pub struct AsyncApiRunRequest {
    pub max_time: Option<f32>,
    pub reset: bool,
}
