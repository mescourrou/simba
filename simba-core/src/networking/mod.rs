//! Networking primitives used by the simulator to exchange data between nodes.
//!
//! Each [`Node`](crate::node::Node) owns a [`Network`](network::Network). The
//! [`NetworkManager`](network_manager::NetworkManager) connects all node networks and is owned by
//! the [`Simulator`](crate::simulator::Simulator).
//!
//! # Communication models
//!
//! 1. One-way messaging
//!    Nodes send messages through [`Network::send_to`](network::Network::send_to) using the
//!    `pub_sub` system of `simba_com`. Other nodes can subscribe to these messages using the same system, and
//!    receive them with the configured `reception_delay`.

pub mod message_types;
pub mod network;
pub mod network_manager;
pub mod pybinds;

/// Errors that can occur while using networking and service communication APIs.
#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum NetworkError {
    /// The underlying channel is closed and can no longer send or receive data.
    ClosedChannel,
    /// The target node is not known by the networking layer.
    NodeUnknown,
    /// An unknown error represented by a free-form message.
    Unknown(String),
    /// The error happened on the client side of a request/response operation.
    ClientSide,
    /// A non-specific networking error.
    Other,
}

/// Definition of channels managed by the simulator.
pub mod channels {
    /// Root of the internal channels, used for internal communication between nodes and the simulator.
    pub const INTERNAL: &str = "/simba";
    /// Internal channel namespaces derived from [`INTERNAL`].
    pub mod internal {
        use constcat::concat;
        /// Root of the log channels, used for internal logs.
        pub const LOG: &str = concat!(super::INTERNAL, "/log");
        /// Log levels for internal logs, LOG/<node_name>/{error,warn,info,debug}
        pub mod log {
            /// ERROR log level channel
            pub const ERROR: &str = "/error";
            /// WARNING log level channel.
            pub const WARNING: &str = "/warn";
            /// INFO log level channel.
            pub const INFO: &str = "/info";
            /// DEBUG log level channel.
            pub const DEBUG: &str = "/debug";
        }
        /// Root path for node-scoped internal channels.
        pub const NODE: &str = concat!(super::INTERNAL, "/nodes");
        /// Root path for simulator command channels (special messages such as kill).
        pub const COMMAND: &str = concat!(super::INTERNAL, "/command");
    }
}

// Network message exchange test
#[cfg(test)]
mod tests {
    use std::{
        str::FromStr,
        sync::{Arc, Mutex},
    };

    use serde::{Deserialize, Serialize};
    use serde_json::Value;
    use simba_com::pub_sub::{MultiClientTrait, PathKey};

    use crate::{
        config::NumberConfig,
        constants::TIME_ROUND,
        context::Context,
        debug,
        logger::LogLevel,
        networking::{
            message_types::MessageTypes,
            network::{Envelope, Network, NetworkConfig},
        },
        node::{Node, node_factory::RobotConfig},
        physics::robot_models::Command,
        plugin_api::PluginAPI,
        recordable::Recordable,
        sensors::{
            Observation, SensorConfig,
            robot_sensor::RobotSensorConfig,
            sensor_manager::{ManagedSensorConfig, SensorManagerConfig},
        },
        simulator::{SimbaBrokerMultiClient, Simulator, SimulatorConfig},
        state_estimators::{
            BenchStateEstimatorConfig, StateEstimator, StateEstimatorConfig, StateEstimatorRecord,
            WorldState,
            external_estimator::{ExternalEstimatorConfig, ExternalEstimatorRecord},
        },
        utils::{
            SharedMutex, SharedRwLock,
            determinist_random_variable::DeterministRandomVariableFactory, maths::round_precision,
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
        pub last_message: SharedMutex<Option<String>>,
        pub last_from: SharedMutex<Option<String>>,
        pub message_client: SimbaBrokerMultiClient,
    }

    impl StateEstimator for StateEstimatorTest {
        fn correction_step(
            &mut self,
            _node: &mut crate::node::Node,
            _observations: &[Observation],
            _time: f32,
            _context: &Context,
        ) {
        }

        fn pre_loop_hook(&mut self, node: &mut Node, time: f32, context: &Context) {
            debug!(context, "Doing pre_loop_hook");
            if node.name() == "node1" {
                println!("{} Sending message...", node.name());
                let message = Envelope {
                    message: MessageTypes::String(self.message.clone()),
                    from: node.name().to_string(),
                    timestamp: time,
                    ..Default::default()
                };
                self.message_client
                    .send(&PathKey::from_str("/test").unwrap(), message, time);
            }
        }

        fn prediction_step(
            &mut self,
            node: &mut crate::node::Node,
            _command: Option<Command>,
            time: f32,
            _context: &Context,
        ) {
            self.last_time = time;
            if node.name() == "node2" {
                while let Some((_path, envelope)) = self.message_client.try_receive(time) {
                    let message: String = envelope.message.clone().into();
                    println!("Receiving message: {} from {}", &message, envelope.from);
                    *self.last_message.lock().unwrap() = Some(message);
                    *self.last_from.lock().unwrap() = Some(envelope.from.clone());
                }
            }
        }

        fn next_time_step(&self, _context: &Context) -> f32 {
            round_precision(self.last_time + 0.1, TIME_ROUND).unwrap()
        }
        fn world_state(&self, _context: &Context) -> WorldState {
            WorldState::new()
        }
    }

    impl Recordable<StateEstimatorRecord> for StateEstimatorTest {
        fn record(&self, _context: &Context) -> StateEstimatorRecord {
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
        pub last_message: SharedMutex<Option<String>>,
        pub last_from: SharedMutex<Option<String>>,
    }

    impl PluginAPI for PluginAPITest {
        fn get_state_estimator(
            &self,
            _config: &serde_json::Value,
            _global_config: &SimulatorConfig,
            _va_factory: &Arc<DeterministRandomVariableFactory>,
            network: &SharedRwLock<Network>,
            initial_time: f32,
            context: &Context,
        ) -> Box<dyn StateEstimator> {
            network
                .write()
                .unwrap()
                .make_channel(PathKey::from_str("/test").unwrap(), context);
            Box::new(StateEstimatorTest {
                last_time: initial_time,
                message: self.message.clone(),
                last_from: self.last_from.clone(),
                last_message: self.last_message.clone(),
                message_client: network.write().unwrap().subscribe_to(
                    &[PathKey::from_str("/test").unwrap()],
                    None,
                    context,
                ),
            }) as Box<dyn StateEstimator>
        }
    }

    #[test]
    fn send_message_test() {
        // Simulator::init_environment(log::LevelFilter::Debug, Vec::new(), Vec::new()); // For debug
        let mut config = SimulatorConfig::default();
        config.log.log_level = LogLevel::Off;
        // config.log.log_level = LogLevel::Internal(vec![crate::logger::InternalLog::All]);
        config.max_time = match RobotSensorConfig::default().activation_time.unwrap().period {
            NumberConfig::Num(p) => p,
            _ => 1.,
        } * 1.1;
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
        config.max_time = match RobotSensorConfig::default().activation_time.unwrap().period {
            NumberConfig::Num(p) => p,
            _ => 1.,
        } * 1.1;
        config.robots.push(RobotConfig {
            name: "node1".to_string(),
            sensor_manager: SensorManagerConfig {
                sensors: vec![ManagedSensorConfig {
                    name: "RobotSensor".to_string(),
                    send_to: Vec::new(),
                    config: SensorConfig::Robot(RobotSensorConfig::default()), // Test valid while RobotSensor uses service for other node poses.
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
                    config: SensorConfig::Robot(RobotSensorConfig::default()),
                    ..Default::default()
                }],
            },
            ..Default::default()
        });

        let mut simulator = Simulator::from_config(&config, None).unwrap();

        simulator.run().unwrap();
    }
}
