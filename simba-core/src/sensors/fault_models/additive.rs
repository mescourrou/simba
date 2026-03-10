//! AdditiveObservationCenteredPolarPolar faults
//!
//! Remark: the order of the application of the random value is alphabetical on the name of the observation variables if no order is specified.

use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

use libm::atan2f;
use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::{
    UIComponent,
    utils::{enum_combobox, string_combobox},
};
use crate::utils::{
    SharedMutex,
    determinist_random_variable::{
        DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig,
    },
    distributions::{
        bernouilli::{BernouilliRandomVariableConfig, DeterministBernouilliRandomVariable},
        normal::NormalRandomVariableConfig,
    },
    enum_tools::EnumVariables,
    geometry::mod2pi,
};

#[config_derives]
pub struct AdditiveFaultConfig<SVO: EnumVariables, SVProp: EnumVariables> {
    #[check]
    pub apparition: BernouilliRandomVariableConfig,
    #[check]
    pub distributions: Vec<RandomVariableTypeConfig>,
    pub variable_order: Vec<SVO>,
    pub proportional_to: Option<SVProp>,
    pub proportional_factor: Option<f32>,
}

impl<SVO: EnumVariables, SVProp: EnumVariables> Check for AdditiveFaultConfig<SVO, SVProp> {
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

impl<SVO: EnumVariables, SVProp: EnumVariables> Default for AdditiveFaultConfig<SVO, SVProp> {
    fn default() -> Self {
        Self {
            apparition: BernouilliRandomVariableConfig {
                probability: vec![1.0],
            },
            distributions: vec![RandomVariableTypeConfig::Normal(
                NormalRandomVariableConfig::default(),
            )],
            variable_order: Vec::new(),
            proportional_to: None,
            proportional_factor: None,
        }
    }
}

#[cfg(feature = "gui")]
impl<SVO: EnumVariables, SVProp: EnumVariables> UIComponent for AdditiveFaultConfig<SVO, SVProp> {
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
            RandomVariableTypeConfig::show_vector_mut(
                &mut self.distributions,
                ui,
                ctx,
                buffer_stack,
                global_config,
                current_node_name,
                unique_id,
            );
            let possible_variables: Vec<SVO> = SVO::to_vec();
            ui.horizontal(|ui| {
                ui.label("Variable order:");
                for (i, var) in self.variable_order.iter_mut().enumerate() {
                    let unique_var_id = format!("variable-{i}-{unique_id}");
                    enum_combobox(ui, var, unique_var_id);
                }
                if !self.variable_order.is_empty() && ui.button("-").clicked() {
                    self.variable_order.pop();
                }
                if ui.button("+").clicked() {
                    self.variable_order.push(
                        possible_variables
                            .get(self.variable_order.len().min(possible_variables.len()))
                            .unwrap()
                            .clone(),
                    );
                }
            });

            let possible_variables: Vec<SVProp> = SVProp::to_vec();
            ui.horizontal(|ui| {
                ui.label("Proportional to:");
                if let Some(variable) = &mut self.proportional_to {
                    let unique_var_id = format!("proportional-to-{unique_id}");
                    enum_combobox(ui, variable, unique_var_id);
                    if ui.button("-").clicked() {
                        self.proportional_to = None;
                        self.proportional_factor = None;
                    }
                } else if ui.button("+").clicked() {
                    self.proportional_to = Some(possible_variables.first().unwrap().clone());
                    self.proportional_factor = Some(1.0);
                }
            });

            if self.proportional_to.is_some() {
                ui.horizontal(|ui| {
                    ui.label("Proportional factor:");
                    if let Some(factor) = &mut self.proportional_factor {
                        ui.add(egui::DragValue::new(factor).speed(0.1));
                    }
                });
            }
        });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.label("Apparition probability: ");
                self.apparition.show(ui, ctx, unique_id);
            });
            RandomVariableTypeConfig::show_vector(&self.distributions, ui, ctx, unique_id);
            ui.horizontal(|ui| {
                ui.label("Variable order: ");
                for var in self.variable_order.iter() {
                    ui.label(format!("{}, ", var));
                }
            });

            ui.horizontal(|ui| {
                ui.label("Proportional to: ");
                if let Some(variable) = &self.proportional_to {
                    ui.label(variable.to_string());
                } else {
                    ui.label("None");
                }
            });

            if let Some(factor) = &self.proportional_factor {
                ui.horizontal(|ui| {
                    ui.label("Proportional factor: ");
                    ui.label(factor.to_string());
                });
            }
        });
    }
}

#[derive(Debug)]
pub struct AdditiveFault<SVO: EnumVariables, SVProp: EnumVariables> {
    apparition: DeterministBernouilliRandomVariable,
    distributions: SharedMutex<Vec<DeterministRandomVariable>>,
    variable_order: Vec<SVO>,
    config: AdditiveFaultConfig<SVO, SVProp>,
}

impl<SVO: EnumVariables, SVProp: EnumVariables> AdditiveFault<SVO, SVProp> {
    pub fn from_config(
        config: &AdditiveFaultConfig<SVO, SVProp>,
        va_factory: &Arc<DeterministRandomVariableFactory>,
        _initial_time: f32,
    ) -> Self {
        let mut config = config.clone();
        let distributions = Arc::new(Mutex::new(
            config
                .distributions
                .iter()
                .map(|conf| va_factory.make_variable(conf.clone()))
                .collect::<Vec<DeterministRandomVariable>>(),
        ));
        if !config.variable_order.is_empty() {
            assert!(
                config.variable_order.len()
                    == distributions
                        .lock()
                        .unwrap()
                        .iter()
                        .map(|d| d.dim())
                        .sum::<usize>(),
                "If variable order is given, its length must match the distribution dimension."
            );
        }
        if config.proportional_to.is_some() && config.proportional_factor.is_none() {
            config.proportional_factor = Some(1.0);
        }
        Self {
            apparition: DeterministBernouilliRandomVariable::from_config(
                va_factory.global_seed(),
                config.apparition.clone(),
            ),
            distributions,
            variable_order: config.variable_order.clone(),
            config,
        }
    }

    /// Adds faults to the given variable map according to the fault configuration and the seed.
    /// Returns the map with the new value of the modified variables
    pub fn add_faults(
        &mut self,
        seed: f32,
        variable_map: HashMap<SVO, f32>,
        proportionnal_map: &HashMap<SVProp, f32>,
    ) -> HashMap<SVO, f32> {
        let mut variable_map = variable_map;
        let mut diff_map = HashMap::new();
        if self.apparition.generate(seed)[0] < 1. {
            return diff_map;
        }
        let mut random_sample = Vec::new();
        for d in self.distributions.lock().unwrap().iter() {
            random_sample.extend_from_slice(&d.generate(seed));
        }

        if let Some(prop_var) = &self.config.proportional_to {
            if let Some(prop_value) = proportionnal_map.get(prop_var) {
                let prop_value = prop_value * self.config.proportional_factor.unwrap_or(1.0);
                random_sample.iter_mut().for_each(|v| *v *= prop_value);
            } else {
                panic!(
                    "Proportionnal variable '{}' not accepted. Accepted proportional variables: [{}]",
                    prop_var,
                    proportionnal_map
                        .keys()
                        .into_iter()
                        .map(|v| v.to_string())
                        .collect::<Vec<_>>()
                        .join(", "),
                );
            }
        }

        if !self.variable_order.is_empty() {
            for (i, var) in self.variable_order.iter().enumerate() {
                if let Some(value) = variable_map.remove(var) {
                    diff_map.insert(var.clone(), value + random_sample[i]);
                } else {
                    panic!(
                        "Variable '{}' not accepted in this situation. Accepted variables: [{}]",
                        var,
                        variable_map
                            .keys()
                            .into_iter()
                            .map(|v| v.to_string())
                            .collect::<Vec<_>>()
                            .join(", "),
                    );
                }
            }
        }
        diff_map
    }

    pub fn config(&self) -> &AdditiveFaultConfig<SVO, SVProp> {
        &self.config
    }
}

pub struct AdditiveObservationCenteredPolarRecord {}
