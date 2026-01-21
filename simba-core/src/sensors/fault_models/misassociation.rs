//! Misassociation faults
//!
//! Remark: the order of the application of the random value is alphabetical on the name of the observation variables if no order is specified.

use std::{
    ops::Rem,
    path::Path,
    sync::{Arc, Mutex},
};

use nalgebra::Vector2;
use rand::prelude::*;
use rand::seq::SliceRandom;
use rand_chacha::ChaCha8Rng;
use serde_json::from_str;
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::{
    UIComponent,
    utils::{enum_combobox, string_combobox},
};
use crate::{
    sensors::{
        SensorObservation, fault_models::fault_model::FaultModelConfig,
        oriented_landmark_sensor::OrientedLandmarkSensor,
    },
    simulator::SimulatorConfig,
    utils::{
        SharedMutex,
        determinist_random_variable::{
            DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig,
        },
        distributions::{
            bernouilli::{BernouilliRandomVariableConfig, DeterministBernouilliRandomVariable},
            uniform::UniformRandomVariableConfig,
        },
    },
};

use super::fault_model::FaultModel;

#[config_derives]
#[derive(Default)]
pub enum Sort {
    None,
    Random,
    #[default]
    Distance,
}

#[config_derives]
pub enum Source {
    Map(String),
    Robots,
}

#[config_derives]
pub struct MisassociationFaultConfig {
    #[check(eq(self.apparition.probability.len(), 1))]
    pub apparition: BernouilliRandomVariableConfig,
    #[check]
    pub distribution: RandomVariableTypeConfig,
    #[check(if(is_enum(self.source, Source::Robots), !is_enum(Sort::Distance)))]
    pub sort: Sort,
    pub source: Source,
}

impl Default for MisassociationFaultConfig {
    fn default() -> Self {
        Self {
            apparition: BernouilliRandomVariableConfig {
                probability: vec![0.1],
            },
            distribution: RandomVariableTypeConfig::Uniform(UniformRandomVariableConfig {
                max: vec![10.],
                min: vec![0.],
            }),
            sort: Sort::Random,
            source: Source::Robots,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for MisassociationFaultConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &crate::simulator::SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.label("Apparition probability: ");
                self.apparition.show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );
            });
            ui.horizontal(|ui| {
                ui.label("Distribution:");
                self.distribution.show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );
            });

            ui.horizontal(|ui| {
                ui.label("Sort:");
                enum_combobox(ui, &mut self.sort, unique_id);
            });

            ui.horizontal(|ui| {
                ui.label("Source:");
                let possible_values = vec!["Robots".to_string(), "Map".to_string()];
                let mut current_source = self.source.to_string();
                string_combobox(ui, &possible_values, &mut current_source, unique_id);
                if let Source::Map(path) = &mut self.source {
                    ui.text_edit_singleline(path);
                }
                if current_source != self.source.to_string() {
                    match current_source.as_str() {
                        "Robots" => self.source = Source::Robots,
                        "Map" => self.source = Source::Map("".to_string()),
                        _ => panic!("Where did you find this value?"),
                    };
                }
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.label("Apparition probability: ");
                self.apparition.show(ui, ctx, unique_id);
            });
            ui.horizontal(|ui| {
                ui.label("Distribution:");
                self.distribution.show(ui, ctx, unique_id);
            });

            ui.horizontal(|ui| {
                ui.label(format!("Sort: {}", self.sort));
            });

            ui.horizontal(|ui| {
                ui.label(format!("Source: {}", self.source));

                if let Source::Map(path) = &self.source {
                    ui.label(format!("({})", path));
                }
            });
        });
    }
}

#[derive(Debug)]
pub struct MisassociationFault {
    apparition: DeterministBernouilliRandomVariable,
    distribution: SharedMutex<Box<dyn DeterministRandomVariable>>,
    sort: Sort,
    id_list: Vec<(String, Vector2<f32>)>,
    _source: Source,
    global_seed: f32,
    config: MisassociationFaultConfig,
}

impl MisassociationFault {
    pub fn from_config(
        config: &MisassociationFaultConfig,
        global_config: &SimulatorConfig,
        robot_name: &String,
        va_factory: &DeterministRandomVariableFactory,
        _initial_time: f32,
    ) -> Self {
        let distribution = Arc::new(Mutex::new(
            va_factory.make_variable(config.distribution.clone()),
        ));
        assert!(
            distribution.lock().unwrap().dim() == 1,
            "Misassociation distribution should be of dimension 1 as it will search in a list"
        );
        let id_list = match &config.source {
            Source::Map(path) => {
                let path = Path::new(&path);
                if !path.exists() {
                    panic!(
                        "The correct map path should be given for Misassociation fault (when source is Map)"
                    );
                }
                let map = OrientedLandmarkSensor::load_map_from_path(path);
                map.iter()
                    .map(|l| (l.id.to_string(), l.pose.fixed_rows::<2>(0).into()))
                    .collect()
            }
            Source::Robots => global_config
                .robots
                .iter()
                .map(|r| {
                    if r.name != *robot_name {
                        Some((r.name.clone(), Vector2::zeros()))
                    } else {
                        None
                    }
                })
                .filter(|x| x.is_some())
                .flatten()
                .collect(),
        };
        Self {
            apparition: DeterministBernouilliRandomVariable::from_config(
                va_factory.global_seed(),
                config.apparition.clone(),
            ),
            distribution,
            sort: config.sort.clone(),
            _source: config.source.clone(),
            id_list,
            global_seed: va_factory.global_seed(),
            config: config.clone(),
        }
    }
}

impl FaultModel for MisassociationFault {
    fn add_faults(
        &mut self,
        _time: f32,
        seed: f32,
        period: f32,
        obs_list: &mut Vec<SensorObservation>,
        obs_type: SensorObservation,
    ) {
        let obs_seed_increment = 1. / (100. * period);
        let mut seed = seed;
        let mut id_list = self.id_list.clone();

        match self.sort {
            Sort::Random => {
                let mut rng = ChaCha8Rng::seed_from_u64((self.global_seed + seed).to_bits() as u64);
                id_list.shuffle(&mut rng);
            }
            Sort::Distance => {
                if let SensorObservation::OrientedRobot(_) = obs_type {
                    panic!(
                        "MisassociationFault: Distance sorting is not implemented for Robot Observation"
                    );
                }
            }
            _ => {}
        }
        for obs in obs_list {
            seed += obs_seed_increment;
            if self.apparition.generate(seed)[0] < 1. {
                continue;
            }
            let random_sample = self.distribution.lock().unwrap().generate(seed);
            match obs {
                SensorObservation::OrientedRobot(o) => {
                    let new_id = id_list
                        [(random_sample[0].abs().floor() as usize).rem(id_list.len())]
                    .0
                    .clone();
                    o.name = new_id;
                    o.applied_faults
                        .push(FaultModelConfig::Misassociation(self.config.clone()));
                }
                SensorObservation::GNSS(_) => {
                    panic!("Not implemented (appropriated for this sensor?)");
                }
                #[allow(deprecated)]
                SensorObservation::Speed(_) | SensorObservation::Odometry(_) => {
                    panic!("Not implemented (appropriated for this sensor?)");
                }
                SensorObservation::Displacement(_) => {
                    panic!("Not implemented (appropriated for this sensor?)");
                }
                SensorObservation::OrientedLandmark(o) => {
                    if let Sort::Distance = self.sort {
                        id_list.sort_by_key(|i| {
                            ((i.1 - o.pose.fixed_rows::<2>(0)).norm_squared() * 1000.) as usize
                        });
                    }
                    let new_id = id_list
                        [(random_sample[0].abs().floor() as usize).rem(id_list.len())]
                    .0
                    .clone();
                    o.id = from_str(&new_id).expect(
                        "Unexpected error: id_list should only contain int represented as string",
                    );
                    o.applied_faults
                        .push(FaultModelConfig::Misassociation(self.config.clone()));
                }
                SensorObservation::External(_) => {
                    panic!("MisassociationFault cannot fault ExternalObservation");
                }
            }
        }
    }
}

pub struct MisassociationRecord {}
