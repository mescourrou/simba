// Configuration for Simulator
extern crate confy;
use serde_derive::{Serialize, Deserialize};

use crate::networking::network_manager::NetworkManager;
use crate::plugin_api::PluginAPI;

use super::turtlebot::{Turtlebot,TurtlebotConfig, TurtlebotRecord};
use std::path::Path;

use std::default::Default;
use serde_json;
use std::sync::{Arc, RwLock, Mutex};
use std::thread;
// use csv::WriterBuilder;
use std::io::prelude::*;
use std::fs::File;

#[derive(Clone)]
pub struct SimulatorMetaConfig {
    pub config_path: Option<Box<Path>>
}

impl SimulatorMetaConfig {
    pub fn new() -> Self {
        Self {
            config_path: None
        }
    }
}


#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct SimulatorConfig {
    pub turtles: Vec<Box<TurtlebotConfig>>
}

impl Default for SimulatorConfig {
    fn default() -> Self {
        Self {
            turtles: Vec::new()
        }
    }
}

#[derive(Serialize, Deserialize)]
struct Record {
    time: f32,
    turtle: TurtlebotRecord
}


pub struct Simulator {
    turtles: Vec<Arc<RwLock<Turtlebot>>>,
    config: SimulatorConfig,
    network_manager: Arc<RwLock<NetworkManager>>
}

impl Simulator {
    pub fn new() -> Simulator {
        Simulator {
            turtles: Vec::new(),
            config: SimulatorConfig::default(),
            network_manager: Arc::new(RwLock::new(NetworkManager::new()))
        }
    }

    pub fn from_config_path(config_path:&Path, plugin_api: Option<Box<dyn PluginAPI>>) -> Simulator {
        let config: SimulatorConfig = match confy::load_path(&config_path) {
            Ok(config) => config,
            Err(error) => {
                println!("Error from Confy while loading the config file : {}", error);
                return Simulator::new();
            }
        };
        println!("Config: {:?}", config);
        let meta_config = SimulatorMetaConfig {
            config_path: Some(Box::from(config_path))
        };
        Simulator::from_config(&config, plugin_api, meta_config)
    }

    pub fn from_config(config:&SimulatorConfig, plugin_api: Option<Box<dyn PluginAPI>>, meta_config: SimulatorMetaConfig) -> Simulator {
        let mut simulator = Simulator::new();
        simulator.config = config.clone();

        // Create turtles
        for turtle_config in &config.turtles {
            simulator.add_turtlebot(turtle_config, &plugin_api, meta_config.clone());
            // simulator.turtles.push(Box::new(Turtlebot::from_config(turtle_config, &plugin_api)));
            // simulator.network_manager.register_turtle_network(simulator.turtles.last().expect("No turtle added to the vector, how is it possible ??").name(), simulator.turtles.last().expect("No turtle added to the vector, how is it possible ??").network());
        }
        simulator
    }

    fn add_turtlebot(&mut self, turtle_config: &TurtlebotConfig, plugin_api: &Option<Box<dyn PluginAPI>>, meta_config: SimulatorMetaConfig) {
        self.turtles.push(Turtlebot::from_config(turtle_config, &plugin_api, meta_config));
        let last_turtle = self.turtles.last().expect("No turtle added to the vector, how is it possible ??").write().unwrap();
        self.network_manager.write().unwrap().register_turtle_network(last_turtle.name(), last_turtle.network());
        last_turtle.network().write().unwrap().set_network_manager(Arc::clone(&self.network_manager));
    }

    pub fn show(&self) {
        println!("Simulator:");
        for turtle in &self.turtles {
            println!("- {:?}", turtle);
        }

    }


    pub fn run(&mut self, max_time: f32) {
        // let mut wtr = WriterBuilder::new()
        //                 .has_headers(false)
        //                 .from_path("result.csv")
        //                 .expect("Impossible to create csv writer");
        let recording_file_rw: Arc<RwLock<File>> = Arc::new(RwLock::new(File::create("result.json").expect("Impossible to create record file")));
    
        let _ = recording_file_rw.write().unwrap().write(b"{\"config\": ");
        serde_json::to_writer(&*recording_file_rw.write().unwrap(), &self.config).expect("Error during json serialization");
        let _ = recording_file_rw.write().unwrap().write(b",\n\"record\": [\n");
    
        let mut handles = vec![];

        for turtle in &self.turtles {
            let new_recording_file = Arc::clone(&recording_file_rw);
            let new_turtle = Arc::clone(turtle);
            let new_max_time = max_time.clone();
            let handle = thread::spawn(move || Self::run_one_turtle(new_turtle, new_max_time, new_recording_file));
            handles.push(handle);
        }

        for handle in handles {
            handle.join().unwrap();
        }
        
        {
            let mut recording_file_open = recording_file_rw.write().unwrap();
            
            let pos = recording_file_open.stream_position().ok().unwrap() - 2;
            let _ = recording_file_open.set_len(pos);
            let _ = recording_file_open.write(b"\n]}");
        }
    }

    fn run_one_turtle(turtle: Arc<RwLock<Turtlebot>>, max_time: f32, recording_file: Arc<RwLock<File>>) {
        println!("Start thread of turtle {}", turtle.read().unwrap().name());
        
        loop {
            let next_time = turtle.read().unwrap().next_time_step();
            if next_time > max_time {
                break;
            }
            
            let mut turtle_open = turtle.write().unwrap();
            turtle_open.run_next_time_step(next_time);

            {
                let mut recording_file_open = recording_file.write().unwrap();
                serde_json::to_writer(&*recording_file_open, &Record {
                        time: next_time,
                        turtle: turtle_open.record()
                    }).expect("Error during json serialization");
                
                let _ = recording_file_open.write(b",\n");
            }
        }
    }
}
