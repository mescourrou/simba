use std::{path::Path, sync::{mpsc, Arc, Mutex}, thread};

use crate::{plugin_api::PluginAPI, simulator::{self, Simulator, SimulatorAsyncApi}};


#[derive(Clone)]
pub struct AsyncApi {
    pub load_config: mpsc::Sender<String>,
    pub run: mpsc::Sender<f32>,
    pub simulator_api: Arc<SimulatorAsyncApi>,
}

#[derive(Clone)]
pub struct AsyncApiServer {
    pub load_config: Arc<Mutex<mpsc::Receiver<String>>>,
    pub run: Arc<Mutex<mpsc::Receiver<f32>>>,
}

#[derive(Clone)]
pub struct AsyncApiRunner {
    public_api: AsyncApi,
    private_api: AsyncApiServer,
    simulator: Arc<Mutex<Simulator>>,
    keep_alive_tx: mpsc::Sender<()>,
    keep_alive_rx: Arc<Mutex<mpsc::Receiver<()>>>,
}

impl AsyncApiRunner {
    pub fn new() -> Self {
        let (load_config_tx, load_config_rx) = mpsc::channel();
        let (run_tx, run_rx) = mpsc::channel();
        let (keep_alive_tx, keep_alive_rx) = mpsc::channel();
        let simulator = Arc::new(Mutex::new(Simulator::new()));
        let simulator_api = simulator.lock().unwrap().get_async_api();
        Self {
            public_api: AsyncApi {
                load_config: load_config_tx.clone(),
                run: run_tx.clone(),
                simulator_api,
            },
            private_api: AsyncApiServer {
                load_config: Arc::new(Mutex::new(load_config_rx)),
                run: Arc::new(Mutex::new(run_rx)),
            },
            simulator,
            keep_alive_rx: Arc::new(Mutex::new(keep_alive_rx)),
            keep_alive_tx,
        }
    }

    pub fn get_api(&self) -> AsyncApi {
        self.public_api.clone()
    }

    pub fn stop(&mut self) {
        self.keep_alive_tx.send(()).unwrap();
    }

    pub fn run(&mut self, plugin_api: Option<Box<&'static dyn PluginAPI>>) {
        let private_api = self.private_api.clone();
        let keep_alive_rx = self.keep_alive_rx.clone();
        let simulator_arc = self.simulator.clone();
        Some(thread::spawn(move || {
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
                }
                if let Ok(max_time) = private_api.run.lock().unwrap().try_recv() {
                    if need_reset {
                        simulator.reset(&plugin_api);
                    }
                    simulator.set_max_time(max_time);
                    simulator.run();
                    need_reset = true;
                }
            }
            log::info!("AsyncApiRunner thread exited");
        }));
    }
}