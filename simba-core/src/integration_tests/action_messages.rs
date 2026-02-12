use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
};

use crate::{
    constants::TIME_ROUND,
    logger::LogLevel,
    navigators::{
        NavigatorConfig, go_to::GoToConfig, trajectory_follower::TrajectoryFollowerConfig,
    },
    networking::network::Network,
    node::node_factory::{NodeRecord, RobotConfig},
    plugin_api::PluginAPI,
    sensors::{
        SensorConfig,
        robot_sensor::RobotSensorConfig,
        sensor_manager::{ManagedSensorConfig, SensorManagerConfig},
    },
    simulator::{ResultConfig, Simulator, SimulatorConfig},
    state_estimators::{
        BenchStateEstimatorConfig, StateEstimator, StateEstimatorConfig,
        external_estimator::ExternalEstimatorConfig, perfect_estimator::PerfectEstimatorConfig,
    },
    utils::{SharedRwLock, determinist_random_variable::DeterministRandomVariableFactory},
};

struct PluginAPITest<SE: StateEstimator + 'static> {
    se: Mutex<Option<SE>>,
}

impl<SE: StateEstimator> PluginAPI for PluginAPITest<SE> {
    fn get_state_estimator(
        &self,
        _config: &serde_json::Value,
        _global_config: &SimulatorConfig,
        _va_factory: &Arc<DeterministRandomVariableFactory>,
        _network: &SharedRwLock<Network>,
        _initial_time: f32,
    ) -> Box<dyn StateEstimator> {
        // let se = std::mem::replace(&mut *self.se.lock().unwrap(), None)
        //     .expect("StateEstimator already taken");
        let se = self
            .se
            .lock()
            .unwrap()
            .take()
            .expect("StateEstimator already taken");
        Box::new(se) as Box<dyn StateEstimator>
    }
}

mod kill_node {
    use std::str::FromStr;

    use simba_com::pub_sub::PathKey;

    use crate::{
        constants::TIME_ROUND,
        networking::network::{Envelope, MessageFlag},
        node::Node,
        recordable::Recordable,
        sensors::Observation,
        state_estimators::{
            StateEstimator, StateEstimatorRecord, WorldState,
            external_estimator::ExternalEstimatorRecord,
        },
        utils::maths::round_precision,
    };

    #[derive(Debug, Clone)]
    pub struct StateEstimatorTest {
        pub last_time: f32,
        pub kill_time: f32,
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
            if time >= self.kill_time {
                node.network().as_ref().unwrap().write().unwrap().send_to(
                    PathKey::from_str("/simba/command/node2").unwrap(),
                    Envelope {
                        from: node.name(),
                        message: serde_json::Value::Null,
                        timestamp: time,
                        message_flags: vec![MessageFlag::Kill],
                    },
                    time,
                );
                self.kill_time = f32::INFINITY;
            }
        }

        fn prediction_step(&mut self, _node: &mut crate::node::Node, time: f32) {
            self.last_time = time;
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
                record: serde_json::Value::default(),
            })
        }
    }
}

#[test]
fn kill_node() {
    let kill_time = 5.;
    let mut config = SimulatorConfig::default();
    config.log.log_level = LogLevel::Off;
    // config.log.log_level = LogLevel::Internal(vec![crate::logger::InternalLog::All]);
    config.max_time = 10.;
    config.results = Some(ResultConfig::default());
    config.robots.push(RobotConfig {
        name: "node1".to_string(),
        state_estimator: StateEstimatorConfig::Perfect(PerfectEstimatorConfig {
            targets: vec!["self".to_string(), "node2".to_string(), "node3".to_string()],
            ..Default::default()
        }),
        state_estimator_bench: vec![BenchStateEstimatorConfig {
            name: "own".to_string(),
            config: StateEstimatorConfig::External(ExternalEstimatorConfig {
                config: serde_json::Value::Null,
            }),
        }],
        ..Default::default()
    });
    config.robots.push(RobotConfig {
        name: "node2".to_string(),
        state_estimator: StateEstimatorConfig::Perfect(PerfectEstimatorConfig {
            targets: vec!["self".to_string(), "node1".to_string(), "node3".to_string()],
            ..Default::default()
        }),
        ..Default::default()
    });
    config.robots.push(RobotConfig {
        name: "node3".to_string(),
        state_estimator: StateEstimatorConfig::Perfect(PerfectEstimatorConfig {
            targets: vec!["self".to_string(), "node1".to_string(), "node2".to_string()],
            ..Default::default()
        }),
        ..Default::default()
    });

    let plugin_api = PluginAPITest::<kill_node::StateEstimatorTest> {
        se: Mutex::new(Some(kill_node::StateEstimatorTest {
            last_time: 0.,
            kill_time,
        })),
    };

    let plugin_api = Arc::new(plugin_api);
    let mut simulator = Simulator::from_config(&config, Some(plugin_api.clone())).unwrap();

    simulator.run().unwrap();

    let records = simulator.get_records(false);
    let mut last_node2_time: f32 = 0.;
    for record in records {
        let t = record.time;
        if let NodeRecord::Robot(r) = record.node
            && r.name.as_str() == "node2"
        {
            last_node2_time = last_node2_time.max(t);
        }
    }
    assert!(
        kill_time == last_node2_time,
        "Node not killed at right time (last time is {})",
        last_node2_time
    );
}

mod trigger_sensor {
    use simba_com::pub_sub::PathKey;

    use crate::{
        constants::TIME_ROUND,
        networking::network::{Envelope, Network},
        node::Node,
        plugin_api::PluginAPI,
        recordable::Recordable,
        sensors::{Observation, sensor_manager::SensorTriggerMessage},
        simulator::SimulatorConfig,
        state_estimators::{
            StateEstimator, StateEstimatorRecord, WorldState,
            external_estimator::ExternalEstimatorRecord,
        },
        utils::{
            SharedMutex, SharedRwLock,
            determinist_random_variable::DeterministRandomVariableFactory, maths::round_precision,
        },
    };
    use std::{collections::VecDeque, str::FromStr, sync::Arc};

    #[derive(Debug, Clone)]
    pub struct StateEstimatorTest {
        pub last_time: f32,
        pub is_the_triggered: bool,
        pub trigger_times: SharedMutex<VecDeque<f32>>,
    }

    impl StateEstimator for StateEstimatorTest {
        fn correction_step(
            &mut self,
            _node: &mut crate::node::Node,
            observations: &[Observation],
            _time: f32,
        ) {
            if self.is_the_triggered {
                for obs in observations {
                    self.trigger_times.lock().unwrap().push_back(obs.time);
                }
            }
        }

        fn pre_loop_hook(&mut self, node: &mut Node, time: f32) {
            if !self.is_the_triggered
                && time
                    >= self
                        .trigger_times
                        .lock()
                        .unwrap()
                        .front()
                        .copied()
                        .unwrap_or(f32::INFINITY)
            {
                log::info!("Triggering sensor at time {}", time);
                node.network().as_ref().unwrap().write().unwrap().send_to(
                    PathKey::from_str("/simba/node/robot1/sensors/RobotSensor").unwrap(),
                    Envelope {
                        from: node.name(),
                        message: serde_json::to_value(SensorTriggerMessage {}).unwrap(),
                        timestamp: time,
                        ..Default::default()
                    },
                    time,
                );
                self.trigger_times.lock().unwrap().pop_front();
            }
        }

        fn prediction_step(&mut self, _node: &mut crate::node::Node, time: f32) {
            self.last_time = time;
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
                record: serde_json::Value::default(),
            })
        }
    }

    pub struct PluginAPITest {
        pub trigger_times: SharedMutex<VecDeque<f32>>,
        pub triggered_times: SharedMutex<VecDeque<f32>>,
    }

    impl PluginAPI for PluginAPITest {
        fn get_state_estimator(
            &self,
            config: &serde_json::Value,
            _global_config: &SimulatorConfig,
            _va_factory: &Arc<DeterministRandomVariableFactory>,
            _network: &SharedRwLock<Network>,
            initial_time: f32,
        ) -> Box<dyn StateEstimator> {
            if config.as_bool().unwrap() {
                Box::new(StateEstimatorTest {
                    last_time: initial_time,
                    is_the_triggered: false,
                    trigger_times: self.trigger_times.clone(),
                }) as Box<dyn StateEstimator>
            } else {
                Box::new(StateEstimatorTest {
                    last_time: initial_time,
                    is_the_triggered: true,
                    trigger_times: self.triggered_times.clone(),
                }) as Box<dyn StateEstimator>
            }
        }
    }
}

#[test]
fn trigger_sensor() {
    let trigger_times = VecDeque::<f32>::from(vec![3.2, 5., 6., 7.5, 9.1, 9.9, 10.0, 21.3]);

    let mut config = SimulatorConfig::default();
    config.log.log_level = LogLevel::Off;
    // config.log.included_nodes = vec!["robot1".to_string()];
    // config.log.excluded_nodes = vec!["simulator".to_string()];
    // config.log.log_level = LogLevel::Internal(vec![crate::logger::InternalLog::SensorManager, InternalLog::SensorManagerDetailed]);
    // config.log.log_level = LogLevel::Internal(vec![crate::logger::InternalLog::All]);
    config.max_time = 25.;
    config.results = None;
    config.robots.push(RobotConfig {
        name: "robot1".to_string(),
        navigator: NavigatorConfig::TrajectoryFollower(TrajectoryFollowerConfig {
            trajectory_path: env!("CARGO_MANIFEST_DIR").to_string()
                + "/test_config/paths/path1.yaml",
            ..Default::default()
        }),
        state_estimator_bench: vec![BenchStateEstimatorConfig {
            name: "triggered".to_string(),
            config: StateEstimatorConfig::External(ExternalEstimatorConfig {
                config: serde_json::Value::Bool(false),
            }),
        }],
        sensor_manager: SensorManagerConfig {
            sensors: vec![ManagedSensorConfig {
                name: "RobotSensor".to_string(),
                config: SensorConfig::RobotSensor(RobotSensorConfig {
                    detection_distance: 100.,
                    period: None,
                    ..Default::default()
                }),
                triggered: true,
                ..Default::default()
            }],
        },
        ..Default::default()
    });
    config.robots.push(RobotConfig {
        name: "robot2".to_string(),
        navigator: NavigatorConfig::GoTo(GoToConfig::default()),
        state_estimator_bench: vec![BenchStateEstimatorConfig {
            name: "triggering".to_string(),
            config: StateEstimatorConfig::External(ExternalEstimatorConfig {
                config: serde_json::Value::Bool(true),
            }),
        }],
        ..Default::default()
    });

    let plugin_api = trigger_sensor::PluginAPITest {
        trigger_times: Arc::new(Mutex::new(trigger_times.clone())),
        triggered_times: Arc::new(Mutex::new(VecDeque::<f32>::new())),
    };
    let plugin_api = Arc::new(plugin_api);
    let mut simulator =
        Simulator::from_config(&config, Some(plugin_api.clone() as Arc<dyn PluginAPI>)).unwrap();

    simulator.run().unwrap();

    println!(
        "Triggered times: {:?}",
        plugin_api.triggered_times.lock().unwrap()
    );
    assert!(
        plugin_api.trigger_times.lock().unwrap().is_empty(),
        "Not all triggers were used"
    );
    assert!(
        plugin_api.triggered_times.lock().unwrap().len() == trigger_times.len(),
        "Not all triggers were received"
    );
    for t in trigger_times.iter() {
        assert!(
            (plugin_api
                .triggered_times
                .lock()
                .unwrap()
                .pop_front()
                .unwrap()
                - *t)
                .abs()
                < TIME_ROUND,
            "Triggered time {} was not in the trigger times (remaining triggered_times: {:?})",
            t,
            plugin_api.triggered_times.lock().unwrap()
        );
    }
    assert!(
        plugin_api.triggered_times.lock().unwrap().is_empty(),
        "There are extra triggered times"
    );
}
