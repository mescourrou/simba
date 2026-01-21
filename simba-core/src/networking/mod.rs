/*!
Module which provides communication tools between nodes.

Each [`Node`](crate::node::Node) includes a [`Network`](network::Network).
The [`NetworkManager`](network_manager::NetworkManager) makes the link between the
[`Network`](network::Network)s and is owned by the [`Simulator`](crate::simulator::Simulator).

There are two main ways to communicate between nodes.

1) One-way communication: A node sends a message to another node. This is done using the
   [`Network::send_to`](network::Network::send_to) and
   [`Network::broadcast`](network::Network::broadcast) methods. The message is sent to the
   receiver [`Network`](network::Network) and is stored in a time ordered buffer. The receiver
   unwraps the message when it reaches the time of the message. If the message is sent in the
   past, the receiver will go back in time to unwrap the message. The message treatment is done
   in [`run_time_step`](crate::node::Node::run_time_step), at the end. The message is
   then passed from one [`MessageHandler`](message_handler::MessageHandler) to the next until
   one of them handles the message.

2) Two-way communication: A node sends a request to another node and waits for the response.
   This is done using the [`Service`](service::Service) and [`ServiceClient`](service::ServiceClient).
   The server node proposes a service, and then a client node need to get a
   [`ServiceClient`](service::ServiceClient) instance to be able to make a request. The client
   sends a request to the server, and is blocked until the server sends a response. The server
   node should handle the requests in [`run_time_step`](crate::node::Node::run_time_step).
*/

use pyo3::{pyclass, pymethods};
use serde::{Deserialize, Serialize};
use simba_macros::EnumToString;

use crate::{navigators::go_to::GoToMessage, sensors::sensor_manager::SensorTriggerMessage};

pub mod message_handler;
pub mod network;
pub mod network_manager;
pub mod service;
pub mod service_manager;

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum NetworkError {
    ClosedChannel,
    NodeUnknown,
    Unknown(String),
    ClientSide,
    Other,
}

#[derive(Debug, Clone, Serialize, Deserialize, EnumToString)]
#[pyclass]
pub enum MessageTypes {
    String(String),
    GoTo(GoToMessage),
    SensorTrigger(SensorTriggerMessage),
}

#[pymethods]
impl MessageTypes {
    #[staticmethod]
    pub fn from_goto(message: GoToMessage) -> Self {
        MessageTypes::GoTo(message)
    }

    #[staticmethod]
    pub fn from_sensor_trigger(message: SensorTriggerMessage) -> Self {
        MessageTypes::SensorTrigger(message)
    }

    pub fn as_goto(&self) -> Option<GoToMessage> {
        match self {
            MessageTypes::GoTo(msg) => Some(msg.clone()),
            _ => None,
        }
    }

    pub fn as_sensor_trigger(&self) -> Option<SensorTriggerMessage> {
        match self {
            MessageTypes::SensorTrigger(msg) => Some(msg.clone()),
            _ => None,
        }
    }

    #[getter]
    pub fn kind(&self) -> String {
        self.to_string()
    }
}

// Network message exchange test
#[cfg(test)]
mod tests {
    use std::sync::{
        Arc, Mutex,
        mpsc::{self, Receiver, Sender},
    };

    use log::debug;
    use serde::{Deserialize, Serialize};
    use serde_json::Value;

    use crate::{
        constants::TIME_ROUND,
        logger::LogLevel,
        networking::{
            message_handler::MessageHandler,
            network::{Envelope, NetworkConfig},
        },
        node::{Node, node_factory::RobotConfig},
        plugin_api::PluginAPI,
        recordable::Recordable,
        sensors::{
            Observation, SensorConfig,
            robot_sensor::RobotSensorConfig,
            sensor_manager::{ManagedSensorConfig, SensorManagerConfig},
        },
        simulator::{Simulator, SimulatorConfig},
        state_estimators::{
            BenchStateEstimatorConfig, StateEstimator, StateEstimatorConfig, StateEstimatorRecord,
            WorldState,
            external_estimator::{ExternalEstimatorConfig, ExternalEstimatorRecord},
        },
        utils::{
            SharedMutex, determinist_random_variable::DeterministRandomVariableFactory,
            maths::round_precision,
        },
    };

    #[derive(Debug, Serialize, Deserialize, Clone)]
    struct StateEstimatorRecordTest {
        pub last_time: f32,
    }

    #[derive(Debug)]
    struct StateEstimatorTest {
        pub last_time: f32,
        pub message: String,
        pub letter_box: Option<SharedMutex<Receiver<Envelope>>>,
        pub letter_box_sender: Option<Sender<Envelope>>,
        pub last_message: SharedMutex<Option<String>>,
        pub last_from: SharedMutex<Option<String>>,
    }

    impl StateEstimator for StateEstimatorTest {
        fn correction_step(
            &mut self,
            _node: &mut crate::node::Node,
            _observations: &[Observation],
            _time: f32,
        ) {
        }

        fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
            debug!("Doing pre_loop_hook");
            if node.name() == "node1" {
                println!("{} Sending message...", node.name());
                node.network()
                    .as_ref()
                    .unwrap()
                    .write()
                    .unwrap()
                    .send_to(
                        "node2".to_string(),
                        serde_json::to_value(self.message.clone()).unwrap(),
                        time,
                        Vec::new(),
                    )
                    .unwrap();
            }
        }

        fn prediction_step(&mut self, node: &mut crate::node::Node, time: f32) {
            self.last_time = time;
            if node.name() == "node2" {
                while let Ok(envelope) =
                    self.letter_box.as_ref().unwrap().lock().unwrap().try_recv()
                {
                    let message = serde_json::from_value(envelope.message.clone()).unwrap();
                    println!("Receiving message: {} from {}", &message, envelope.from);
                    *self.last_message.lock().unwrap() = Some(message);
                    *self.last_from.lock().unwrap() = Some(envelope.from.clone());
                }
            }
        }

        fn next_time_step(&self) -> f32 {
            round_precision(self.last_time + 0.1, TIME_ROUND).unwrap()
        }
        fn world_state(&self) -> WorldState {
            WorldState::new()
        }
    }

    impl Recordable<StateEstimatorRecord> for StateEstimatorTest {
        fn record(&self) -> StateEstimatorRecord {
            StateEstimatorRecord::External(ExternalEstimatorRecord {
                record: serde_json::to_value(StateEstimatorRecordTest {
                    last_time: self.last_time,
                })
                .unwrap(),
            })
        }
    }

    impl MessageHandler for StateEstimatorTest {
        fn get_letter_box(&self) -> Option<Sender<Envelope>> {
            self.letter_box_sender.clone()
        }
    }

    struct PluginAPITest {
        pub message: String,
        pub last_message: SharedMutex<Option<String>>,
        pub last_from: SharedMutex<Option<String>>,
    }

    impl PluginAPI for PluginAPITest {
        fn get_state_estimator(
            &self,
            config: &serde_json::Value,
            _global_config: &SimulatorConfig,
            _va_factory: &Arc<DeterministRandomVariableFactory>,
            initial_time: f32,
        ) -> Box<dyn StateEstimator> {
            let (tx, rx) = if config.as_bool().unwrap() {
                let (tx, rx) = mpsc::channel();
                (Some(tx), Some(Arc::new(Mutex::new(rx))))
            } else {
                (None, None)
            };
            Box::new(StateEstimatorTest {
                last_time: initial_time,
                message: self.message.clone(),
                last_from: self.last_from.clone(),
                last_message: self.last_message.clone(),
                letter_box: rx,
                letter_box_sender: tx,
            }) as Box<dyn StateEstimator>
        }

        // fn get_message_handlers(
        //     &self,
        //     node: &Node,
        // ) -> Option<Vec<SharedRwLock<dyn MessageHandler>>>> {
        //     if node.name() == "node2" {
        //         Some(vec![self.message_handler.clone()])
        //     } else {
        //         None
        //     }
        // }
    }

    #[test]
    fn send_message_test() {
        // Simulator::init_environment(log::LevelFilter::Debug, Vec::new(), Vec::new()); // For debug
        let mut config = SimulatorConfig::default();
        config.log.log_level = LogLevel::Off;
        // config.log.log_level = LogLevel::Internal(vec![crate::logger::InternalLog::All]);
        config.max_time = RobotSensorConfig::default().period.unwrap() * 1.1;
        config.robots.push(RobotConfig {
            name: "node1".to_string(),
            state_estimator_bench: vec![BenchStateEstimatorConfig {
                name: "own".to_string(),
                config: StateEstimatorConfig::External(ExternalEstimatorConfig {
                    config: Value::Bool(false),
                }),
            }],
            ..Default::default()
        });
        config.robots.push(RobotConfig {
            name: "node2".to_string(),
            network: NetworkConfig {
                reception_delay: 0.,
                ..Default::default()
            },
            state_estimator_bench: vec![BenchStateEstimatorConfig {
                name: "own".to_string(),
                config: StateEstimatorConfig::External(ExternalEstimatorConfig {
                    config: Value::Bool(true),
                }),
            }],
            ..Default::default()
        });

        let message_to_send = String::from("HelloWorld");
        let plugin_api = PluginAPITest {
            message: message_to_send.clone(),
            last_from: Arc::new(Mutex::new(None)),
            last_message: Arc::new(Mutex::new(None)),
        };

        let plugin_api = Arc::new(plugin_api);
        let mut simulator = Simulator::from_config(&config, Some(plugin_api.clone())).unwrap();

        simulator.run().unwrap();

        assert!(
            plugin_api.last_message.lock().unwrap().is_some(),
            "Message not received"
        );
        assert!(
            *plugin_api.last_message.lock().unwrap().as_ref().unwrap() == message_to_send,
            "Wrong message received"
        );
        assert!(
            plugin_api
                .last_from
                .lock()
                .unwrap()
                .as_ref()
                .unwrap()
                .as_str()
                == "node1",
            "Wrong 'from'"
        );
    }

    #[test]
    fn deadlock_service_test() {
        // Simulator::init_environment(log::LevelFilter::Debug, Vec::new(), Vec::new()); // For debug
        let mut config = SimulatorConfig::default();
        config.log.log_level = LogLevel::Off;
        // config.log.log_level = LogLevel::Internal(vec![crate::logger::InternalLog::All]);
        config.max_time = RobotSensorConfig::default().period.unwrap() * 1.1;
        config.robots.push(RobotConfig {
            name: "node1".to_string(),
            sensor_manager: SensorManagerConfig {
                sensors: vec![ManagedSensorConfig {
                    name: "RobotSensor".to_string(),
                    send_to: Vec::new(),
                    config: SensorConfig::RobotSensor(RobotSensorConfig::default()), // Test valid while RobotSensor uses service for other node poses.
                    ..Default::default()
                }],
            },
            ..Default::default()
        });
        config.robots.push(RobotConfig {
            name: "node2".to_string(),
            sensor_manager: SensorManagerConfig {
                sensors: vec![ManagedSensorConfig {
                    name: "RobotSensor".to_string(),
                    send_to: Vec::new(),
                    config: SensorConfig::RobotSensor(RobotSensorConfig::default()),
                    ..Default::default()
                }],
            },
            ..Default::default()
        });

        let mut simulator = Simulator::from_config(&config, None).unwrap();

        simulator.run().unwrap();
    }
}
