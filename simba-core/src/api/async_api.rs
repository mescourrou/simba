//! Asynchronous API for the simulator, to allow running the simulator in a separate thread and communicate with it through channels.
//! This is used by the GUI and the Python API to communicate with the simulator without blocking the main thread.

use std::{
    os::unix::thread::JoinHandleExt,
    sync::{Arc, Mutex, RwLock, mpsc},
    thread::{self, JoinHandle, sleep},
    time::Duration,
};

use simba_com::rfc::{self, RemoteFunctionCall, RemoteFunctionCallHost};

use crate::{
    controllers::Controller,
    errors::SimbaResult,
    navigators::Navigator,
    networking::network::Network,
    physics::Physics,
    plugin_api::PluginAPI,
    simulator::{Record, Simulator, SimulatorAsyncApi, SimulatorConfig},
    state_estimators::StateEstimator,
    utils::{
        SharedMutex, SharedRwLock, determinist_random_variable::DeterministRandomVariableFactory,
    },
};

/// Asynchronous API for [`Simulator`] in order to allow running the simulator in a separate thread, and communicate with it through channels. This is a client of [`AsyncApiServer`].
///
/// It is used by the GUI, or when using a Python API as the simulator should not block the main thread.
#[derive(Clone)]
pub struct AsyncApi {
    /// Very limited API to the simulator to get the current time and records.
    pub simulator_api: Arc<SimulatorAsyncApi>,
    // Channels
    /// Trigger the loading of a configuration in the simulator, with the given configuration and a flag to force sending results to [`Self::simulator_api`] (function [`Simulator::load_config`]).
    pub load_config:
        rfc::RemoteFunctionCall<AsyncApiLoadConfigRequest, SimbaResult<SimulatorConfig>>,
    /// Trigger the loading of results in the simulator, with the given path to the results file (function [`Simulator::load_results`]). The path is optional, if not given, it will load the results from the last simulation run. It returns the last result time.
    pub load_results: rfc::RemoteFunctionCall<Option<String>, SimbaResult<f32>>,
    /// Trigger the running of the simulator, with the given parameters to reset the simulator and set a maximum time (combination of [`Simulator::set_max_time`], [`Simulator::reset`] and [`Simulator::run`]). If reset is true, the simulator will be reset before running. If max_time is given, the simulator will run until the simulated time reaches max_time. Otherwise, it will run until the end of the simulation.
    pub run: rfc::RemoteFunctionCall<AsyncApiRunRequest, SimbaResult<()>>,
    /// Ask for the simulator records (function [`Simulator::get_records`]). The call argument is for sorting the records by time, if true, or not sorted if false. Sorting requires more time, so it can be set to false if the order of the records is not important.
    pub get_records: rfc::RemoteFunctionCall<bool, SimbaResult<Vec<Record>>>,
    /// Trigger the computation of results in the simulator (function [`Simulator::compute_results`]). It will call the python script if it is defined in the configuration file.
    pub compute_results: rfc::RemoteFunctionCall<(), SimbaResult<()>>,
}

// Run by the simulator
/// Asynchronous API server for the simulator, to receive requests from the [`AsyncApi`] and execute them on the simulator. It is owned by the simulator and runs in a separate thread. See [`AsyncApi`] for details.
#[derive(Clone)]
#[allow(missing_docs)]
pub struct AsyncApiServer {
    pub load_config:
        Arc<rfc::RemoteFunctionCallHost<AsyncApiLoadConfigRequest, SimbaResult<SimulatorConfig>>>,
    pub load_results: Arc<rfc::RemoteFunctionCallHost<Option<String>, SimbaResult<f32>>>,
    pub run: Arc<rfc::RemoteFunctionCallHost<AsyncApiRunRequest, SimbaResult<()>>>,
    pub compute_results: Arc<rfc::RemoteFunctionCallHost<(), SimbaResult<()>>>,
    pub get_records: Arc<rfc::RemoteFunctionCallHost<bool, SimbaResult<Vec<Record>>>>,
}

// #[derive(Clone)]
/// Runner for the asynchronous API, to run the simulator in a separate thread and listen to API calls.
/// It owns the simulator and the API server, and runs the server in a separate thread.
/// It also provides a public API to send requests to the simulator, and a method to stop the runner and the threads.
///
/// # Example:
/// ```rust
/// // Create a new runner with a new simulator
/// let runner = Arc::new(Mutex::new(AsyncApiRunner::new()));
/// // Get the public API to send requests to the simulator
/// let api = Arc::new(Mutex::new(runner.lock().unwrap().get_api()));
/// // Start the runner (not the simulator)
/// runner.lock().unwrap().run(None);
///
/// api.lock().unwrap().load_config.async_call(AsyncApiLoadConfigRequest {
///     config: SimulatorConfig::default(),
///     force_send_results: false,
/// });
///
/// if let Ok(config) = api.lock().unwrap().load_config.wait_result() {
///     println!("Config loaded: {:?}", config);
///     // Run the simulator for 10 seconds, without resetting it (it will run from the current state)
///     api.lock().unwrap().run.call(AsyncApiRunRequest {
///         max_time: None, // Use the max_time defined in the simulator config
///         reset: false,  // Do not reset the simulator, run from the current state
///     }).unwrap();
///
///    api.lock().unwrap().compute_results.wait_result(()).unwrap();
/// }
///
/// // Stop the runner
/// runner.lock().unwrap().stop();
/// ```
pub struct AsyncApiRunner {
    public_api: AsyncApi,
    private_api: AsyncApiServer,
    simulator: SharedMutex<Simulator>,
    keep_alive_tx: mpsc::Sender<()>,
    keep_alive_rx: SharedMutex<mpsc::Receiver<()>>,
    thread_handle: Option<JoinHandle<()>>,
    running: bool,
}

impl AsyncApiRunner {
    /// Create a new [`AsyncApiRunner`] with a new simulator default simulator.
    /// The configuration of the simulator can be changed by calling the `load_config` function of the public API.
    /// The runner is not started by default, call the `run` function to start it and listen to API calls.
    pub fn new() -> Self {
        let simulator = Arc::new(Mutex::new(Simulator::new()));
        AsyncApiRunner::new_with_simulator(simulator)
    }

    /// Create a new [`AsyncApiRunner`] with the given simulator.
    /// The configuration of the simulator can be changed by calling the `load_config` function of the public API.
    /// The runner is not started by default, call the `run` function to start it and listen to API calls.
    pub fn new_with_simulator(simulator: SharedMutex<Simulator>) -> Self {
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

    /// Get the public API to send requests to the simulator (loading configuration, running, etc.).
    /// It is used by the GUI and the Python API to communicate with the simulator.
    pub fn get_api(&self) -> AsyncApi {
        self.public_api.clone()
    }

    /// Get the inner simulator.
    /// WARNING: for advanced use only
    pub fn get_simulator(&self) -> SharedMutex<Simulator> {
        self.simulator.clone()
    }

    /// Stop the runner and the threads waiting for API calls.
    ///
    /// This method is automatically called when the runner is dropped, but can be called manually to stop the threads before dropping the runner.
    /// If threads are blocked, they will be force stopped after a short delay using `pthread_cancel`.
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

    /// Start the runner.
    ///
    /// It will spawn a thread to listen to the API calls and execute them on the simulator.
    /// It will also spawn threads for each API call to execute them in parallel and not block the main thread. The threads will be stopped when the runner is stopped.
    ///
    /// # Arguments
    /// * `plugin_api` - Optional plugin API to use when loading the configuration. It can be None if the plugin API is not needed but required if the configuration use Externals.
    pub fn run(&mut self, plugin_api: Option<Arc<dyn PluginAPI>>) {
        let private_api = self.private_api.clone();
        let keep_alive_rx = self.keep_alive_rx.clone();
        let simulator_cloned = self.simulator.clone();
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
                        println!(
                            "Loading config: {}",
                            request.config.base_path.to_str().unwrap()
                        );
                        simulator.load_config_full(
                            &request.config,
                            plugin_api_threaded.clone(),
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
                    load_results.recv_closure(|result_path| {
                        let mut simulator = simulator_arc.lock().unwrap();
                        simulator.load_results(result_path)
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
                            simulator.reset(plugin_api_threaded.clone())?;
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

/// Plugin API implementation using the asynchronous API. It is used to call the plugin API from the simulator, and get the state estimator, controller, navigator and physics engine from the plugin.
/// It is owned by the simulator and passed to the plugin when loading the configuration.
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
    /// Create a new plugin API: creation of the remote function calls and their hosts.
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

    /// Get a client to the API.
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
        config: &serde_json::Value,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        network: &SharedRwLock<Network>,
        initial_time: f32,
    ) -> Box<dyn StateEstimator> {
        self.get_state_estimator
            .call(PluginAsyncAPIGetStateEstimatorRequest {
                config: config.clone(),
                global_config: global_config.clone(),
                va_factory: va_factory.clone(),
                network: network.clone(),
                initial_time,
            })
            .unwrap()
    }

    fn get_controller(
        &self,
        config: &serde_json::Value,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        network: &SharedRwLock<Network>,
        initial_time: f32,
    ) -> Box<dyn Controller> {
        self.get_controller
            .call(PluginAsyncAPIGetControllerRequest {
                config: config.clone(),
                global_config: global_config.clone(),
                va_factory: va_factory.clone(),
                network: network.clone(),
                initial_time,
            })
            .unwrap()
    }

    fn get_navigator(
        &self,
        config: &serde_json::Value,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        network: &SharedRwLock<Network>,
        initial_time: f32,
    ) -> Box<dyn Navigator> {
        self.get_navigator
            .call(PluginAsyncAPIGetNavigatorRequest {
                config: config.clone(),
                global_config: global_config.clone(),
                va_factory: va_factory.clone(),
                network: network.clone(),
                initial_time,
            })
            .unwrap()
    }

    fn get_physics(
        &self,
        config: &serde_json::Value,
        global_config: &SimulatorConfig,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        network: &SharedRwLock<Network>,
        initial_time: f32,
    ) -> Box<dyn Physics> {
        self.get_physics
            .call(PluginAsyncAPIGetPhysicsRequest {
                config: config.clone(),
                global_config: global_config.clone(),
                va_factory: va_factory.clone(),
                network: network.clone(),
                initial_time,
            })
            .unwrap()
    }
}

/// Client of the plugin API, to be used by the simulator to call the plugin API and get the state estimator, controller, navigator and physics engine from the plugin. It is owned by the simulator and passed to the plugin when loading the configuration.
#[derive(Clone)]
pub struct PluginAsyncAPIClient {
    /// Get a state estimator from the plugin API, with the given configuration, global configuration, random variable factory, network and initial time. It returns a state estimator to use in the simulator.
    pub get_state_estimator: Arc<
        RemoteFunctionCallHost<PluginAsyncAPIGetStateEstimatorRequest, Box<dyn StateEstimator>>,
    >,
    /// Get a controller from the plugin API, with the given configuration, global configuration, random variable factory, network and initial time. It returns a controller to use in the simulator.
    pub get_controller:
        Arc<RemoteFunctionCallHost<PluginAsyncAPIGetControllerRequest, Box<dyn Controller>>>,
    /// Get a navigator from the plugin API, with the given configuration, global configuration, random variable factory, network and initial time. It returns a navigator to use in the simulator.
    pub get_navigator:
        Arc<RemoteFunctionCallHost<PluginAsyncAPIGetNavigatorRequest, Box<dyn Navigator>>>,
    /// Get a physics engine from the plugin API, with the given configuration, global configuration, random variable factory, network and initial time. It returns a physics engine to use in the simulator.
    pub get_physics: Arc<RemoteFunctionCallHost<PluginAsyncAPIGetPhysicsRequest, Box<dyn Physics>>>,
}

/// Request to get a state estimator from the plugin API.
/// It contains the configuration for the state estimator to get, the global configuration of the simulator, a factory for random variables, a reference to the network and the initial time of the simulation.
pub struct PluginAsyncAPIGetStateEstimatorRequest {
    /// User custom configuration as a serialized JSON value. It is up to the plugin to parse it and use it as needed.
    pub config: serde_json::Value,
    /// Global configuration of the simulator, as defined in the configuration file. It can be used by the plugin to get information about the simulation and configure itself accordingly.
    pub global_config: SimulatorConfig,
    /// Factory for random variables, to create random variables with the same seed as the simulator and ensure reproducibility. It can be used by the plugin to create random variables for its internal use.
    pub va_factory: Arc<DeterministRandomVariableFactory>,
    /// Reference to the network, to subscribe to channels and create new ones.
    pub network: SharedRwLock<Network>,
    /// Initial time of the node, can be not 0 if the node is spawned during the simulation. It can be used by the plugin to initialize its state accordingly.
    pub initial_time: f32,
}

/// Request to get a controller from the plugin API.
/// It contains the configuration for the controller to get, the global configuration of the simulator, a factory for random variables, a reference to the network and the initial time of the simulation.
pub type PluginAsyncAPIGetControllerRequest = PluginAsyncAPIGetStateEstimatorRequest;

/// Request to get a navigator from the plugin API.
/// It contains the configuration for the navigator to get, the global configuration of the simulator, a factory for random variables, a reference to the network and the initial time of the simulation.
pub type PluginAsyncAPIGetNavigatorRequest = PluginAsyncAPIGetStateEstimatorRequest;

/// Request to get a physics engine from the plugin API.
/// It contains the configuration for the physics engine to get, the global configuration of the simulator, a factory for random variables, a reference to the network and the initial time of the simulation.
pub type PluginAsyncAPIGetPhysicsRequest = PluginAsyncAPIGetStateEstimatorRequest;

/// Request to call [`Simulator::load_config`] with the given configuration, a flag to force sending results to the API and an optional plugin API to use when loading the configuration.
#[derive(Clone, Debug, Default)]
pub struct AsyncApiLoadConfigRequest {
    /// Configuration to load in the simulator.
    pub config: SimulatorConfig,
    /// If true, the simulator will send results to the API even if results are disabled in the configuration. Needed for GUI.
    pub force_send_results: bool,
}

/// Request to call [`Simulator::run`] with the given parameters.
#[derive(Clone, Debug, Default)]
pub struct AsyncApiRunRequest {
    /// Maximum time to run the simulator, if None, it will use the max_time defined in the simulator config.
    pub max_time: Option<f32>,
    /// If true, the simulator will be reset before running, loosing its state. If false, the simulator will keep its state and run from it. This can be useful to run multiple steps.
    pub reset: bool,
}
