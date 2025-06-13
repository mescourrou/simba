/*!
Provides a [`Sensor`] which can provide position and velocity in the global frame.
*/

use std::sync::{Arc, Mutex, RwLock};

use super::fault_models::fault_model::{
    make_fault_model_from_config, FaultModel, FaultModelConfig,
};
use super::sensor::{Sensor, SensorObservation, SensorRecord};

use crate::constants::TIME_ROUND;
#[cfg(feature = "gui")]
use crate::gui::UIComponent;
use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorConfig;
use crate::stateful::Stateful;
use crate::utils::determinist_random_variable::DeterministRandomVariableFactory;
use crate::utils::maths::round_precision;
use config_checker::macros::Check;
use nalgebra::Vector2;
use serde_derive::{Deserialize, Serialize};

extern crate nalgebra as na;

/// Configuration of the [`GNSSSensor`].
#[derive(Serialize, Deserialize, Debug, Clone, Check)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct GNSSSensorConfig {
    /// Observation period of the sensor.
    #[check(ge(0.))]
    pub period: f32,
    /// Fault on the x, y positions, and on the x and y velocities
    #[check]
    pub faults: Vec<FaultModelConfig>,
}

impl Default for GNSSSensorConfig {
    fn default() -> Self {
        Self {
            period: 1.,
            faults: Vec::new(),
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for GNSSSensorConfig {
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::HashMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &String,
    ) {
        egui::CollapsingHeader::new("GNSS sensor")
            .id_source(format!("gnss-sensor-{}", unique_id))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Period:");
                    if self.period < TIME_ROUND {
                        self.period = TIME_ROUND;
                    }
                    ui.add(egui::DragValue::new(&mut self.period).max_decimals((1./TIME_ROUND) as usize));
                });

                FaultModelConfig::show_faults(
                    &mut self.faults,
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );
            });
    }
}

/// Record of the [`GNSSSensor`], which contains nothing for now.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct GNSSSensorRecord {
    last_time: f32,
}

impl Default for GNSSSensorRecord {
    fn default() -> Self {
        Self { last_time: 0. }
    }
}

/// Observation of the odometry.
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct GNSSObservation {
    pub position: Vector2<f32>,
    pub velocity: Vector2<f32>,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct GNSSObservationRecord {
    pub position: [f32; 2],
    pub velocity: [f32; 2],
}

impl Stateful<GNSSObservationRecord> for GNSSObservation {
    fn record(&self) -> GNSSObservationRecord {
        GNSSObservationRecord {
            position: [self.position.x, self.position.y],
            velocity: [self.velocity.x, self.velocity.y],
        }
    }

    fn from_record(&mut self, record: GNSSObservationRecord) {
        self.position = Vector2::from_vec(record.position.to_vec());
        self.velocity = Vector2::from_vec(record.velocity.to_vec());
    }
}

/// Sensor which observes the robot's odometry
#[derive(Debug)]
pub struct GNSSSensor {
    /// Observation period
    period: f32,
    /// Last observation time.
    last_time: f32,
    /// Fault models for x and y positions and on x and y velocities
    faults: Arc<Mutex<Vec<Box<dyn FaultModel>>>>,
}

impl GNSSSensor {
    /// Makes a new [`GNSSSensor`].
    pub fn new() -> Self {
        GNSSSensor::from_config(
            &GNSSSensorConfig::default(),
            &None,
            &SimulatorConfig::default(),
            &"NoName".to_string(),
            &DeterministRandomVariableFactory::default(),
        )
    }

    /// Makes a new [`GNSSSensor`] from the given config.
    pub fn from_config(
        config: &GNSSSensorConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        global_config: &SimulatorConfig,
        robot_name: &String,
        va_factory: &DeterministRandomVariableFactory,
    ) -> Self {
        let fault_models = Arc::new(Mutex::new(Vec::new()));
        let mut unlock_fault_model = fault_models.lock().unwrap();
        for fault_config in &config.faults {
            unlock_fault_model.push(make_fault_model_from_config(
                fault_config,
                global_config,
                robot_name,
                va_factory,
            ));
        }
        drop(unlock_fault_model);
        Self {
            period: config.period,
            last_time: 0.,
            faults: fault_models,
        }
    }
}

use crate::node::Node;

impl Sensor for GNSSSensor {
    fn init(&mut self, _robot: &mut Node) {}

    fn get_observations(&mut self, robot: &mut Node, time: f32) -> Vec<SensorObservation> {
        let arc_physic = robot
            .physics()
            .expect("Node with GNSS sensor should have Physics");
        let physic = arc_physic.read().unwrap();
        let mut observation_list = Vec::<SensorObservation>::new();
        if (time - self.next_time_step()).abs() > TIME_ROUND / 2. {
            return observation_list;
        }
        let state = physic.state(time);

        let velocity = Vector2::<f32>::from_vec(vec![
            state.velocity * state.pose.z.cos(),
            state.velocity * state.pose.z.sin(),
        ]);

        observation_list.push(SensorObservation::GNSS(GNSSObservation {
            position: state.pose.fixed_rows::<2>(0).into(),
            velocity,
        }));
        for fault_model in self.faults.lock().unwrap().iter() {
            fault_model.add_faults(
                time,
                self.period,
                &mut observation_list,
                SensorObservation::GNSS(GNSSObservation::default()),
            );
        }

        self.last_time = time;
        observation_list
    }

    fn next_time_step(&self) -> f32 {
        round_precision(self.last_time + self.period, TIME_ROUND).unwrap()
    }

    fn period(&self) -> f32 {
        self.period
    }
}

impl Stateful<SensorRecord> for GNSSSensor {
    fn record(&self) -> SensorRecord {
        SensorRecord::GNSSSensor(GNSSSensorRecord {
            last_time: self.last_time,
        })
    }

    fn from_record(&mut self, record: SensorRecord) {
        if let SensorRecord::GNSSSensor(gnss_record) = record {
            self.last_time = gnss_record.last_time;
        }
    }
}
