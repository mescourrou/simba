//! Misassociation fault model.
//!
//! This module defines a fault model that can replace an observation label with another one,
//! according to a configurable random process.
//! Behavior is controlled by [`MisassociationFaultConfig`], and runtime decisions are executed by
//! [`MisassociationFault`].

use std::{
    ops::Rem,
    sync::{Arc, Mutex},
};

use nalgebra::Vector2;
use rand::prelude::*;
use rand::seq::SliceRandom;
use rand_chacha::ChaCha8Rng;
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::{
    UIComponent,
    utils::{enum_combobox, string_combobox},
};
use crate::{
    environment::Environment,
    node::NodeState,
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

/// Ordering strategy used to pick replacement labels.
///
/// This controls how candidate labels are arranged before indexing with a sampled value.
#[config_derives]
#[derive(Default)]
pub enum Sort {
    /// Keep the source-provided order of candidates (alphabetically).
    None,
    /// Shuffle candidates using a deterministic random generator seeded from simulation state.
    Random,
    /// Sort candidates by increasing squared distance to the observer position.
    #[default]
    Distance,
}

/// Source used to build the candidate label list for misassociation.
#[config_derives]
pub enum Source {
    /// Use landmark identifiers from the current map.
    Map,
    /// Use identifiers of running robots from environment metadata.
    Robots,
}

/// Configuration for the misassociation fault model.
///
/// This configuration defines when misassociation happens and how the replacement label
/// is selected from candidates.
/// The `apparition` variable decides if misassociation is applied for a sample,
/// while `distribution` provides the sampled index used to pick a candidate label.
/// The `sort` and `source` options define how the candidate list is built and ordered.
#[config_derives]
pub struct MisassociationFaultConfig {
    /// Bernoulli variable controlling whether misassociation is triggered.
    #[check]
    pub apparition: BernouilliRandomVariableConfig,
    /// Random variable used to sample the candidate index.
    #[check]
    pub distribution: RandomVariableTypeConfig,
    /// Strategy used to order candidate labels before sampling.
    pub sort: Sort,
    /// Source from which candidate labels are collected.
    pub source: Source,
}

impl Check for MisassociationFaultConfig {
    fn do_check(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.apparition.probability.len() != 1 {
            errors.push(format!(
                "Apparition probability should be of length 1, got {}",
                self.apparition.probability.len()
            ));
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
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
                let possible_values = vec!["Robots", "Map"];
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

/// Runtime implementation of label misassociation faults.
///
/// This type samples configured random variables and maps an input label to another label
/// selected from map landmarks or robot names.
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
    /// Builds a runtime fault model from [`MisassociationFaultConfig`].
    pub fn from_config(
        config: &MisassociationFaultConfig,
        va_factory: &DeterministRandomVariableFactory,
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

    /// Returns the label after applying one misassociation decision.
    ///
    /// If misassociation is not triggered for `seed`, this returns `old_label` unchanged.
    /// Otherwise, it selects a new label from candidates extracted from [`Environment`].
    pub fn new_label(
        &mut self,
        seed: f32,
        old_label: String,
        position: Vector2<f32>,
        environment: &Arc<Environment>,
    ) -> String {
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
        if self.apparition.generate(seed)[0] < 1. {
            return old_label;
        }
        let random_sample = self.distribution.lock().unwrap().generate(seed);
        if let Sort::Distance = self.sort {
            id_list.sort_by_key(|i| ((i.1 - position).norm_squared() * 1000.) as usize);
        }
        id_list[(random_sample[0].abs().floor() as usize).rem(id_list.len())]
            .0
            .clone()
    }

    /// Returns the configuration used to build this fault model.
    pub fn config(&self) -> &MisassociationFaultConfig {
        &self.config
    }
}

/// Record type for misassociation faults.
pub struct MisassociationRecord {}
