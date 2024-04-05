//! Simulator definitions: the Simulator struct, and the config and record linked structs
//!
//! The Simulator is the primary struct to be called to start the simulator.

// Configuration for Simulator
extern crate confy;
use serde_derive::{Deserialize, Serialize};

use crate::networking::network_manager::NetworkManager;
use crate::plugin_api::PluginAPI;
use crate::result_analyser;

use super::turtlebot::{Turtlebot, TurtlebotConfig, TurtlebotRecord};
use std::path::Path;

use serde_json;
use std::default::Default;
use std::sync::{Arc, Mutex, RwLock};
use std::thread;
// use csv::WriterBuilder;
use std::fs::File;
use std::io::prelude::*;

use log::{debug, error, info, log_enabled, Level};

#[derive(Clone)]
pub struct SimulatorMetaConfig {
    pub config_path: Option<Box<Path>>,
    pub result_path: Option<Box<Path>>,
    pub compute_results: bool,
    pub no_gui: bool,
}

impl SimulatorMetaConfig {
    pub fn default() -> Self {
        Self {
            config_path: None,
            result_path: None,
            compute_results: false,
            no_gui: true
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct SimulatorConfig {
    pub turtles: Vec<Box<TurtlebotConfig>>,
}

impl Default for SimulatorConfig {
    fn default() -> Self {
        Self {
            turtles: Vec::new(),
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Record {
    pub time: f32,
    pub turtle: TurtlebotRecord,
}

pub struct Simulator {
    turtles: Vec<Arc<RwLock<Turtlebot>>>,
    config: SimulatorConfig,
    meta_config: SimulatorMetaConfig,
    network_manager: Arc<RwLock<NetworkManager>>,
}

impl Simulator {
    pub fn new() -> Simulator {
        Simulator {
            turtles: Vec::new(),
            config: SimulatorConfig::default(),
            meta_config: SimulatorMetaConfig::default(),
            network_manager: Arc::new(RwLock::new(NetworkManager::new())),
        }
    }

    pub fn from_config_path(
        config_path: &Path,
        plugin_api: Option<Box<dyn PluginAPI>>,
        result_path: &Path,
        compute_results: bool,
        no_gui: bool
    ) -> Simulator {
        let config: SimulatorConfig = match confy::load_path(&config_path) {
            Ok(config) => config,
            Err(error) => {
                error!("Error from Confy while loading the config file : {}", error);
                return Simulator::new();
            }
        };
        debug!("Config: {:?}", config);
        let meta_config = SimulatorMetaConfig {
            config_path: Some(Box::from(config_path)),
            result_path: Some(Box::from(result_path)),
            compute_results,
            no_gui,
        };
        Simulator::from_config(&config, plugin_api, meta_config)
    }

    pub fn from_config(
        config: &SimulatorConfig,
        plugin_api: Option<Box<dyn PluginAPI>>,
        meta_config: SimulatorMetaConfig,
    ) -> Simulator {
        let mut simulator = Simulator::new();
        simulator.config = config.clone();
        simulator.meta_config = meta_config.clone();

        // Create turtles
        for turtle_config in &config.turtles {
            simulator.add_turtlebot(turtle_config, &plugin_api, meta_config.clone());
            // simulator.turtles.push(Box::new(Turtlebot::from_config(turtle_config, &plugin_api)));
            // simulator.network_manager.register_turtle_network(simulator.turtles.last().expect("No turtle added to the vector, how is it possible ??").name(), simulator.turtles.last().expect("No turtle added to the vector, how is it possible ??").network());
        }
        simulator
    }

    pub fn init_environment() {
        env_logger::builder()
            .target(env_logger::Target::Stdout)
            .init();
    }

    fn add_turtlebot(
        &mut self,
        turtle_config: &TurtlebotConfig,
        plugin_api: &Option<Box<dyn PluginAPI>>,
        meta_config: SimulatorMetaConfig,
    ) {
        self.turtles.push(Turtlebot::from_config(
            turtle_config,
            &plugin_api,
            meta_config,
        ));
        let last_turtle = self
            .turtles
            .last()
            .expect("No turtle added to the vector, how is it possible ??")
            .write()
            .unwrap();
        self.network_manager
            .write()
            .unwrap()
            .register_turtle_network(last_turtle.name(), last_turtle.network());
        last_turtle
            .network()
            .write()
            .unwrap()
            .set_network_manager(Arc::clone(&self.network_manager));
    }

    pub fn show(&self) {
        info!("Simulator:");
        for turtle in &self.turtles {
            info!("- {:?}", turtle);
        }
    }

    pub fn run(&mut self, max_time: f32) {
        // let mut wtr = WriterBuilder::new()
        //                 .has_headers(false)
        //                 .from_path("result.csv")
        //                 .expect("Impossible to create csv writer");

        let mut handles = vec![];

        for turtle in &self.turtles {
            turtle.write().unwrap().save_state(0.);
            let new_turtle = Arc::clone(turtle);
            let new_max_time = max_time.clone();
            let handle = thread::spawn(move || Self::run_one_turtle(new_turtle, new_max_time));
            handles.push(handle);
        }

        for handle in handles {
            let _ = handle.join();
        }

        self.save_results();
        self.compute_results();
    }

    pub fn get_results(&self) -> Vec<Record> {
        let mut records = Vec::new();
        for turtle in &self.turtles {
            let turtle_r = turtle.read().unwrap();
            let turtle_history = turtle_r.record_history();
            for (time, record) in turtle_history.iter() {
                records.push(Record {
                    time: time.clone(),
                    turtle: record.clone(),
                });
            }
        }
        records
    }

    pub fn save_results(&mut self) {
        let filename = match &self.meta_config.result_path {
            Some(f) => f,
            None => return
        };
        let mut recording_file = File::create(filename).expect("Impossible to create record file");

        let _ = recording_file.write(b"{\"config\": ");
        serde_json::to_writer(&recording_file, &self.config)
            .expect("Error during json serialization");
        let _ = recording_file.write(b",\n\"record\": [\n");

        let mut first_row = true;
        for turtle in &self.turtles {
            let turtle_r = turtle.read().unwrap();
            let turtle_history = turtle_r.record_history();
            for (time, record) in turtle_history.iter() {
                if first_row {
                    first_row = false;
                } else {
                    let _ = recording_file.write(b",\n");
                }
                serde_json::to_writer(
                    &recording_file,
                    &Record {
                        time: time.clone(),
                        turtle: record.clone(),
                    },
                )
                .expect("Error during json serialization");
            }
        }
        let _ = recording_file.write(b"\n]}");
    }

    fn run_one_turtle(turtle: Arc<RwLock<Turtlebot>>, max_time: f32) {
        info!("Start thread of turtle {}", turtle.read().unwrap().name());

        loop {
            let next_time = turtle.read().unwrap().next_time_step();
            if next_time > max_time {
                break;
            }

            let mut turtle_open = turtle.write().unwrap();
            turtle_open.run_next_time_step(next_time);
        }
    }

    pub fn compute_results(&self) {
        if !self.meta_config.compute_results {
            return;
        }

        let result_filename = match &self.meta_config.result_path {
            Some(f) => f,
            None => return
        };

        info!("Starting result analyse...");
        let show_figures = !self.meta_config.no_gui;
        match result_analyser::execute_python_analyser(
            result_filename.as_ref(),
            show_figures, 
            result_filename.parent().unwrap_or(Path::new(".")), 
            String::from(".pdf")) {
            Ok(()) => info!("Results analysed!"),
            Err(e) => error!("Error during python execution: {e}")
        };
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;

    #[test]
    fn replication_test() {
        let nb_replications = 100;
        env_logger::builder()
            .target(env_logger::Target::Stdout)
            .is_test(true)
            .filter_level(log::LevelFilter::Warn)
            .init();

        let mut results: Vec<Vec<Record>> = Vec::new();

        let config_path = Path::new("config_example/config.yaml");
        for i in 0..nb_replications {
            let mut simulator = Simulator::from_config_path(config_path, None);

            simulator.show();

            simulator.run(60.);

            results.push(simulator.get_results());
        }

        let reference_result = &results[0];
        for i in 1..nb_replications {
            assert_eq!(results[i].len(), reference_result.len());
            for j in 0..reference_result.len() {
                let result_as_str = format!("{:?}", results[i][j]);
                let reference_result_as_str = format!("{:?}", reference_result[j]);
                assert_eq!(result_as_str, reference_result_as_str);
            }
        }
    }
}
