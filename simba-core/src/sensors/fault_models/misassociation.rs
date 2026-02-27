//! Misassociation faults
//!
//! Remark: the order of the application of the random value is alphabetical on the name of the observation variables if no order is specified.

use std::{
    ops::Rem,
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
    environment::Environment,
    node::NodeState,
    sensors::{
        SensorObservation, fault_models::fault_model::FaultModelConfig,
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
    Map,
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
                if current_source != self.source.to_string() {
                    match current_source.as_str() {
                        "Robots" => self.source = Source::Robots,
                        "Map" => self.source = Source::Map,
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
            });
        });
    }
}

#[derive(Debug)]
pub struct MisassociationFault {
    apparition: DeterministBernouilliRandomVariable,
    distribution: SharedMutex<DeterministRandomVariable>,
    sort: Sort,
    source: Source,
    global_seed: f32,
    config: MisassociationFaultConfig,
}

impl MisassociationFault {
    pub fn from_config(
        config: &MisassociationFaultConfig,
        _global_config: &SimulatorConfig,
        _robot_name: &str,
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
        Self {
            apparition: DeterministBernouilliRandomVariable::from_config(
                va_factory.global_seed(),
                config.apparition.clone(),
            ),
            distribution,
            sort: config.sort.clone(),
            source: config.source.clone(),
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
        obs_list: &mut Vec<SensorObservation>,
        _obs_type: SensorObservation,
        environment: &Arc<Environment>,
    ) {
        let obs_seed_increment = 1. / (100. * obs_list.len() as f32);
        let mut seed = seed;

        let mut id_list: Vec<(String, Vector2<f32>)> = match &self.source {
            Source::Robots => environment
                .get_meta_data()
                .read()
                .unwrap()
                .iter()
                .filter_map(|(name, data)| {
                    if matches!(data.read().unwrap().state, NodeState::Running)
                        && let Some(position) = data.read().unwrap().position
                    {
                        Some((name.clone(), Vector2::new(position[0], position[1])))
                    } else {
                        None
                    }
                })
                .collect(),
            Source::Map => environment
                .map()
                .landmarks
                .iter()
                .map(|l| (l.id.to_string(), l.pose.fixed_rows::<2>(0).into()))
                .collect(),
        };

        if self.sort == Sort::Random {
            let mut rng = ChaCha8Rng::seed_from_u64((self.global_seed + seed).to_bits() as u64);
            id_list.shuffle(&mut rng);
        }
        for obs in obs_list {
            seed += obs_seed_increment;
            if self.apparition.generate(seed)[0] < 1. {
                continue;
            }
            let random_sample = self.distribution.lock().unwrap().generate(seed);
            match obs {
                SensorObservation::OrientedRobot(o) => {
                    if let Sort::Distance = self.sort {
                        id_list.sort_by_key(|i| {
                            ((i.1 - o.pose.fixed_rows::<2>(0)).norm_squared() * 1000.) as usize
                        });
                    }
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
                SensorObservation::Speed(_) => {
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
