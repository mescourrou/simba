use std::sync::{Arc, RwLock};


use super::navigators::navigator::{Navigator, NavigatorConfig, NavigatorRecord};
use super::navigators::trajectory_follower;

use crate::controllers::controller::{Controller, ControllerConfig, ControllerRecord};
use crate::controllers::pid;

use crate::networking::message_handler::MessageHandler;
use crate::networking::network::{Network, NetworkConfig};
use crate::physics::physic::{Physic, PhysicConfig, PhysicRecord};
use crate::physics::perfect_physic;

use crate::state_estimators::state_estimator::{StateEstimator, StateEstimatorConfig, StateEstimatorRecord};
use crate::state_estimators::{perfect_estimator, external_estimator};

use crate::sensors::sensor_manager::{SensorManagerConfig, SensorManager};

use crate::plugin_api::PluginAPI;

// Configuration for Turtlebot
extern crate confy;
use serde_derive::{Serialize, Deserialize};
use serde_json::Value;


#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct TurtlebotConfig {
    pub name: String,
    pub navigator: NavigatorConfig,
    pub controller: ControllerConfig,
    pub physic: PhysicConfig,
    pub state_estimator: StateEstimatorConfig,
    pub sensor_manager: SensorManagerConfig,
    pub network: NetworkConfig
    
}

impl Default for TurtlebotConfig {
    fn default() -> Self {
        TurtlebotConfig {
            name: String::from("NoName"),
            navigator: NavigatorConfig::TrajectoryFollower(Box::new(trajectory_follower::TrajectoryFollowerConfig::default())),
            controller: ControllerConfig::PID(Box::new(pid::PIDConfig::default())),
            physic: PhysicConfig::Perfect(Box::new(perfect_physic::PerfectPhysicConfig::default())),
            state_estimator: StateEstimatorConfig::Perfect(Box::new(perfect_estimator::PerfectEstimatorConfig::default())),
            sensor_manager: SensorManagerConfig::default(),
            network: NetworkConfig::default()
        }
    }
}


#[derive(Serialize, Deserialize)]
pub struct TurtlebotRecord {
    pub name: String,
    pub navigator: NavigatorRecord,
    pub controller: ControllerRecord,
    pub physic: PhysicRecord,
    pub state_estimator: StateEstimatorRecord
    
}


// Turtlebot itself

#[derive(Debug)]
pub struct Turtlebot {
    name: String,
    navigator: Arc<RwLock<Box<dyn Navigator>>>,
    controller: Arc<RwLock<Box<dyn Controller>>>,
    physic: Arc<RwLock<Box<dyn Physic>>>,
    state_estimator: Arc<RwLock<Box<dyn StateEstimator>>>,
    sensor_manager: Arc<RwLock<SensorManager>>,
    network: Arc<RwLock<Network>>,
    next_time_step: f32
}

impl Turtlebot {
    pub fn new(name:String) -> Arc<RwLock<Self>> {
        Arc::new(RwLock::new(Self { 
            name: name.clone(),
            navigator: Arc::new(RwLock::new(Box::new(trajectory_follower::TrajectoryFollower::new()))),
            controller: Arc::new(RwLock::new(Box::new(pid::PID::new()))),
            physic: Arc::new(RwLock::new(Box::new(perfect_physic::PerfectPhysic::new()))),
            state_estimator: Arc::new(RwLock::new(Box::new(perfect_estimator::PerfectEstimator::new()))),
            sensor_manager: Arc::new(RwLock::new(SensorManager::new())),
            network: Arc::new(RwLock::new(Network::new(name.clone()))),
            next_time_step: 0.
        }))
    }

    pub fn from_config(config:&TurtlebotConfig, plugin_api: &Option<Box<dyn PluginAPI>>) -> Arc<RwLock<Self>> {
        let mut turtle = Arc::new(RwLock::new(Self {
            name: config.name.clone(),
            navigator: Arc::new(RwLock::new(Box::new(
                match &config.navigator {
                    NavigatorConfig::TrajectoryFollower(c) => trajectory_follower::TrajectoryFollower::from_config(c, plugin_api)
                }
            ))),
            controller: Arc::new(RwLock::new(Box::new(
                match &config.controller {
                    ControllerConfig::PID(c) => pid::PID::from_config(c, plugin_api)
                }
            ))),
            physic: Arc::new(RwLock::new(Box::new(
                match &config.physic {
                    PhysicConfig::Perfect(c) => perfect_physic::PerfectPhysic::from_config(c, plugin_api)
                }
            ))),
            state_estimator: 
                match &config.state_estimator {
                    StateEstimatorConfig::Perfect(c) => Arc::new(RwLock::new(Box::new(perfect_estimator::PerfectEstimator::from_config(c, plugin_api)) as Box<dyn StateEstimator>)),
                    StateEstimatorConfig::External(c) => Arc::new(RwLock::new(Box::new(external_estimator::ExternalEstimator::from_config(c, plugin_api)) as Box<dyn StateEstimator>))
                },
            sensor_manager: Arc::new(RwLock::new(SensorManager::from_config(&config.sensor_manager, plugin_api))),
            network: Arc::new(RwLock::new(Network::from_config(config.name.clone(), &config.network))),
            next_time_step: 0.
        }));
        let next_time_step = turtle.read().unwrap().state_estimator.read().unwrap().next_time_step();
        turtle.write().unwrap().next_time_step = next_time_step;
        turtle.write().unwrap().network.write().unwrap().subscribe(Arc::<RwLock<Turtlebot>>::clone(&turtle));
        turtle
    }

    pub fn message_callback_str(&mut self, message: &Value) -> Result<(), ()> {
        if let Value::String(str_msg) = message {
            println!("I accept to receive your message: {}", str_msg);
            return Ok(());
        } else {
            println!("Use next handler please");
            return Err(());
        }
    }

    pub fn message_callback_number(&mut self, message: &Value) -> Result<(), ()> {
        if let Value::Number(nbr) = message {
            println!("I accept to receive your message: {}", nbr);
            return Ok(());
        } else {
            println!("Use next handler please");
            return Err(());
        }
    }

    
    
    pub fn run_next_time_step(&mut self, time: f32) -> f32 {
        if time < self.next_time_step {
            return self.next_time_step;
        }
        println!("Run time {}", time);
        self.physic.write().unwrap().update_state(time);
        let observations = self.sensor_manager.write().unwrap().get_observations(self.physic.read().unwrap().as_ref(), time);
        self.state_estimator.write().unwrap().correction_step(observations, time, self.physic.read().unwrap().as_ref());
        if time >= self.state_estimator.read().unwrap().next_time_step() {
            self.state_estimator.write().unwrap().prediction_step(time, self.physic.read().unwrap().as_ref());
            let state = self.state_estimator.read().unwrap().state();
            // println!("State: {:?}", state);
            let error = self.navigator.write().unwrap().compute_error(&state);
            // println!("Error: {:?}", error);
            let command = self.controller.write().unwrap().make_command(&error, time);
            // println!("Command: {:?}", command);
            self.physic.write().unwrap().apply_command(&command, time);

            self.network.write().unwrap().send_to(String::from("turtle2"), serde_json::Value::String(String::from("Bonjour")));
            self.network.write().unwrap().send_to(String::from("turtle2"), serde_json::Value::Number(serde_json::value::Number::from_f64(3.2).unwrap()));

            self.network.write().unwrap().broadcast(serde_json::Value::String(String::from("Bonjour tout le monde")));
            self.network.write().unwrap().broadcast(serde_json::Value::Number(serde_json::value::Number::from_f64(5.1).unwrap()));
        }
        
        self.next_time_step = 
            self.state_estimator.read().unwrap().next_time_step()
            .min(self.sensor_manager.read().unwrap().next_time_step());
        // println!("{}: {}", time, state);
        self.network().write().unwrap().handle_messages();

        self.next_time_step
    }

    pub fn next_time_step(&self) -> f32 {
        self.next_time_step
    }

    pub fn record(&self) -> TurtlebotRecord {
        TurtlebotRecord {
            name: self.name.clone(),
            navigator: self.navigator.read().unwrap().record(),
            controller: self.controller.read().unwrap().record(),
            physic: self.physic.read().unwrap().record(),
            state_estimator: self.state_estimator.read().unwrap().record(),
        }
    }

    pub fn name(&self) -> String {
        self.name.clone()
    }

    pub fn network(&self) -> Arc<RwLock<Network>> {
        Arc::clone(&self.network)
    }
}

impl MessageHandler for Turtlebot {
    fn handle_message(&mut self, from: &String, message: &Value) -> Result<(),()> {
        if self.message_callback_str(&message).is_ok() {
            return Ok(());
        } else if self.message_callback_number(&message).is_ok() {
            return Ok(());
        }
        Err(())
    }
}