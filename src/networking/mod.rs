/*!
Module which provides communication tools between nodes.

Each [`Robot`](crate::node::Robot) includes a [`Network`](network::Network).
The [`NetworkManager`](network_manager::NetworkManager) makes the link between the
[`Network`](network::Network)s and is owned by the [`Simulator`](crate::simulator::Simulator).

There are two main ways to communicate between nodes.

1) One-way communication: A node sends a message to another node. This is done using the
[`Network::send_to`](network::Network::send_to) and
[`Network::broadcast`](network::Network::broadcast) methods. The message is sent to the
receiver [`Network`](network::Network) and is stored in a time ordered buffer. The receiver
unwraps the message when it reaches the time of the message. If the message is sent in the
past, the receiver will go back in time to unwrap the message. The message treatment is done
in [`run_time_step`](crate::node::Robot::run_time_step), at the end. The message is
then passed from one [`MessageHandler`](message_handler::MessageHandler) to the next until
one of them handles the message.

2) Two-way communication: A node sends a request to another node and waits for the response.
This is done using the [`Service`](service::Service) and [`ServiceClient`](service::ServiceClient).
The server node proposes a service, and then a client node need to get a
[`ServiceClient`](service::ServiceClient) instance to be able to make a request. The client
sends a request to the server, and is blocked until the server sends a response. The server
node should handle the requests in [`run_time_step`](crate::node::Robot::run_time_step).
*/

pub mod message_handler;
pub mod network;
pub mod network_manager;
pub mod service;
pub mod service_manager;

// Network message exchange test
#[cfg(test)]
mod tests {
    use std::{
        panic,
        sync::{Arc, RwLock},
    };

    use serde::{Deserialize, Serialize};
    use serde_json::Value;

    use crate::{
        constants::TIME_ROUND,
        node::Node,
        node_factory::RobotConfig,
        plugin_api::PluginAPI,
        sensors::{
            robot_sensor::RobotSensorConfig, sensor::SensorConfig,
            sensor_manager::SensorManagerConfig,
        },
        simulator::{Simulator, SimulatorConfig},
        state_estimators::{
            external_estimator::{ExternalEstimatorConfig, ExternalEstimatorRecord},
            perfect_estimator::PerfectEstimatorConfig,
            state_estimator::{
                BenchStateEstimatorConfig, State, StateEstimator, StateEstimatorConfig,
                StateEstimatorRecord,
            },
        },
        stateful::Stateful,
        utils::maths::{closest_uint_modulo, round_precision},
    };

    use super::{message_handler::MessageHandler, *};

    #[derive(Debug, Default)]
    struct NetworkHandlerTest {
        pub last_message: Option<String>,
        pub last_from: Option<String>,
        pub last_time: Option<f32>,
    }

    impl MessageHandler for NetworkHandlerTest {
        fn handle_message(
            &mut self,
            _node: &mut crate::node::Node,
            from: &String,
            message: &serde_json::Value,
            time: f32,
        ) -> Result<(), ()> {
            self.last_message = Some(serde_json::from_value(message.clone()).unwrap());
            println!("Receiving message: {}", self.last_message.as_ref().unwrap());
            self.last_from = Some(from.clone());
            self.last_time = Some(time);

            Ok(())
        }
    }

    #[derive(Debug, Serialize, Deserialize, Clone)]
    struct StateEstimatorRecordTest {
        pub last_time: f32,
    }

    #[derive(Debug)]
    struct StateEstimatorTest {
        pub last_time: f32,
        pub message: String,
    }

    impl StateEstimator for StateEstimatorTest {
        fn correction_step(
            &mut self,
            node: &mut crate::node::Node,
            observations: &Vec<crate::sensors::sensor::Observation>,
            time: f32,
        ) {
        }

        fn prediction_step(&mut self, node: &mut crate::node::Node, time: f32) {
            self.last_time = time;
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

        fn next_time_step(&self) -> f32 {
            round_precision(self.last_time + 0.1, TIME_ROUND).unwrap()
        }
        fn state(&self) -> State {
            State::new()
        }
    }

    impl Stateful<StateEstimatorRecord> for StateEstimatorTest {
        fn from_record(&mut self, record: StateEstimatorRecord) {
            if let StateEstimatorRecord::External(record) = record {
                let record: StateEstimatorRecordTest =
                    serde_json::from_value(record.record).unwrap();
                self.last_time = record.last_time;
            } else {
                panic!("Should not happen");
            }
        }

        fn record(&self) -> StateEstimatorRecord {
            StateEstimatorRecord::External(ExternalEstimatorRecord {
                record: serde_json::to_value(StateEstimatorRecordTest {
                    last_time: self.last_time,
                })
                .unwrap(),
            })
        }
    }

    struct PluginAPITest {
        pub message: String,
        pub message_handler: Arc<RwLock<dyn MessageHandler>>,
    }

    impl PluginAPI for PluginAPITest {
        fn get_state_estimator(
            &self,
            _config: &serde_json::Value,
            _global_config: &SimulatorConfig,
        ) -> Box<dyn StateEstimator> {
            Box::new(StateEstimatorTest {
                last_time: 0.,
                message: self.message.clone(),
            }) as Box<dyn StateEstimator>
        }

        fn get_message_handlers(
            &self,
            node: &Node,
        ) -> Option<Vec<Arc<RwLock<dyn MessageHandler>>>> {
            if node.name() == "node2" {
                Some(vec![self.message_handler.clone()])
            } else {
                None
            }
        }
    }

    #[test]
    fn send_message_test() {
        // Simulator::init_environment(log::LevelFilter::Debug, Vec::new(), Vec::new()); // For debug
        let mut config = SimulatorConfig::default();
        config.max_time = PerfectEstimatorConfig::default().update_period * 1.1;
        config.robots.push(RobotConfig {
            name: "node1".to_string(),
            state_estimator_bench: vec![BenchStateEstimatorConfig {
                name: "own".to_string(),
                config: StateEstimatorConfig::External(ExternalEstimatorConfig {
                    config: Value::Null,
                }),
            }],
            ..Default::default()
        });
        config.robots.push(RobotConfig {
            name: "node2".to_string(),
            ..Default::default()
        });

        let message_to_send = String::from("HelloWorld");
        let message_handler: Arc<RwLock<NetworkHandlerTest>> =
            Arc::new(RwLock::new(NetworkHandlerTest::default()));

        let plugin_api = PluginAPITest {
            message: message_to_send.clone(),
            message_handler: message_handler.clone(),
        };

        let mut simulator = Simulator::from_config(&config, &Some(Box::new(&plugin_api)));

        simulator.run();

        assert!(
            message_handler.read().unwrap().last_message.is_some(),
            "Message not received"
        );
        assert!(
            message_handler
                .read()
                .unwrap()
                .last_message
                .as_ref()
                .unwrap()
                == &message_to_send,
            "Wrong message received"
        );
        assert!(
            message_handler
                .read()
                .unwrap()
                .last_from
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
        config.max_time = RobotSensorConfig::default().period * 1.1;
        config.robots.push(RobotConfig {
            name: "node1".to_string(),
            sensor_manager: SensorManagerConfig {
                sensors: vec![
                    SensorConfig::RobotSensor(RobotSensorConfig::default()), // Test valid while RobotSensor uses service for other node poses.
                ],
            },
            ..Default::default()
        });
        config.robots.push(RobotConfig {
            name: "node2".to_string(),
            sensor_manager: SensorManagerConfig {
                sensors: vec![SensorConfig::RobotSensor(RobotSensorConfig::default())],
            },
            ..Default::default()
        });

        let mut simulator = Simulator::from_config(&config, &None);

        simulator.run();
    }
}
