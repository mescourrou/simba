// Configuration for Simulator
extern crate confy;
use serde_derive::{Serialize, Deserialize};

use crate::plugin_api::PluginAPI;

use super::turtlebot::{Turtlebot,TurtlebotConfig, TurtlebotRecord};
use std::path::Path;

use std::default::Default;
use serde_json;
// use csv::WriterBuilder;
use std::io::prelude::*;
use std::fs::File;

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
    turtles: Vec<Box<Turtlebot>>,
    config: SimulatorConfig
}

impl Simulator {
    pub fn new() -> Simulator {
        Simulator {
            turtles: Vec::new(),
            config: SimulatorConfig::default()
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
        Simulator::from_config(&config, plugin_api)
    }

    pub fn from_config(config:&SimulatorConfig, plugin_api: Option<Box<dyn PluginAPI>>) -> Simulator {
        let mut simulator = Simulator::new();
        simulator.config = config.clone();

        // Create turtles
        for turtle_config in &config.turtles {
            simulator.turtles.push(Box::new(Turtlebot::from_config(turtle_config, &plugin_api)));
        }
        simulator
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
        let mut recording_file = File::create("result.json").expect("Impossible to create record file");
        let _ = recording_file.write(b"{\"config\": ");
        serde_json::to_writer(&recording_file, &self.config).expect("Error during json serialization");
        let _ = recording_file.write(b",\n\"record\": [\n");
        let mut first_row_done = false;
        loop {
            let mut best_turtle: usize = 0;
            let mut best_time = f32::INFINITY;
            for i in 0..self.turtles.len() {
                let turtle = self.turtles[i].as_ref();
                let time = turtle.next_time_step();
                // println!("Check turtle {} => next time is {}", i, time);
                if time < best_time {
                    best_turtle = i;
                    best_time = time;
                }
            }
            if best_time > max_time {
                break;
            }
            if first_row_done {
                let _ = recording_file.write(b",\n");
            } else {
                first_row_done = true;
            }
            self.turtles[best_turtle].run_next_time_step(best_time);

            serde_json::to_writer(&recording_file, &Record {
                    time: best_time,
                    turtle: self.turtles[best_turtle].record()
                }).expect("Error during json serialization");
                
        }
        let _ = recording_file.write(b"]}");
    }
}
