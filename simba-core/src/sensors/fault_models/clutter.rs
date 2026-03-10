use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

use simba_macros::config_derives;

#[cfg(feature = "gui")]
use crate::gui::{
    UIComponent,
    utils::{enum_combobox, string_combobox, text_singleline_with_apply},
};
use crate::utils::{
    SharedMutex,
    determinist_random_variable::{
        DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig,
    },
    distributions::{poisson::PoissonRandomVariableConfig, uniform::UniformRandomVariableConfig},
    enum_tools::EnumVariables,
    geometry::mod2pi,
};

use super::fault_model::FaultModel;

#[config_derives]
pub struct ClutterFaultConfig<SV: EnumVariables> {
    #[check]
    pub apparition: RandomVariableTypeConfig,
    #[check]
    pub distributions: Vec<RandomVariableTypeConfig>,
    pub variable_order: Vec<SV>,
    pub observation_id: String,
}

impl<SV: EnumVariables> Check for ClutterFaultConfig<SV> {
    fn do_check(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.apparition.dim() != 1 {
            errors.push(format!(
                "Apparition probability should be of length 1, got {}",
                self.apparition.dim()
            ));
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

impl<SV: EnumVariables> Default for ClutterFaultConfig<SV> {
    fn default() -> Self {
        Self {
            apparition: RandomVariableTypeConfig::Poisson(PoissonRandomVariableConfig {
                lambda: vec![10.],
            }),
            distributions: vec![RandomVariableTypeConfig::Uniform(
                UniformRandomVariableConfig {
                    min: vec![-10., -10.],
                    max: vec![10., 10.],
                },
            )],
            variable_order: Vec::new(),
            observation_id: "clutter".to_string(),
        }
    }
}

#[cfg(feature = "gui")]
impl<SV: EnumVariables> UIComponent for ClutterFaultConfig<SV> {
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
            let possible_variables: Vec<SV> = SV::to_vec();
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
            let buffer_key = format!("observation-id-{unique_id}");
            text_singleline_with_apply(
                ui,
                buffer_key.as_str(),
                buffer_stack,
                &mut self.observation_id,
            );
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

            ui.label(format!("Observation id: {}", self.observation_id));
        });
    }
}

#[derive(Debug)]
pub struct ClutterFault<SV: EnumVariables> {
    apparition: SharedMutex<DeterministRandomVariable>,
    distributions: SharedMutex<Vec<DeterministRandomVariable>>,
    variable_order: Vec<SV>,
    observation_id: String,
    config: ClutterFaultConfig<SV>,
}

impl<SV: EnumVariables> ClutterFault<SV> {
    pub fn from_config(
        config: &ClutterFaultConfig<SV>,
        va_factory: &DeterministRandomVariableFactory,
        _initial_time: f32,
    ) -> Self {
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
        let apparition_distrib = Arc::new(Mutex::new(
            va_factory.make_variable(config.apparition.clone()),
        ));
        assert!(
            apparition_distrib.lock().unwrap().dim() == 1,
            "The dimension of the apparition distribution should be 1."
        );
        Self {
            apparition: apparition_distrib,
            distributions,
            variable_order: config.variable_order.clone(),
            observation_id: config.observation_id.clone(),
            config: config.clone(),
        }
    }

    pub fn add_faults(
        &mut self,
        seed: f32,
        seed_increment: f32,
    ) -> Vec<(String, HashMap<SV, f32>)> {
        let mut seed = seed;

        let n_obs = self.apparition.lock().unwrap().generate(seed)[0]
            .abs()
            .floor() as usize;
        let mut new_obs = Vec::new();
        for _ in 0..n_obs {
            let mut o = HashMap::new();
            seed += seed_increment;
            let mut random_sample = Vec::new();
            for d in self.distributions.lock().unwrap().iter() {
                random_sample.extend_from_slice(&d.generate(seed));
            }
            if !self.variable_order.is_empty() {
                for (i, variable) in self.variable_order.iter().enumerate() {
                    o.insert(variable.clone(), random_sample[i]);
                }
            }
            new_obs.push((self.observation_id.clone(), o));
            // let mut new_obs = obs_type.clone();
            // match &mut new_obs {
            //     SensorObservation::OrientedRobot(o) => {
            //         if !self.variable_order.is_empty() {
            //             for (i, variable) in self.variable_order.iter().enumerate() {
            //                 match variable.as_str() {
            //                     "x" => o.pose.x = random_sample[i],
            //                     "y" => o.pose.y = random_sample[i],
            //                     "z" | "orientation" => o.pose.z = random_sample[i],
            //                     &_ => panic!(
            //                         "Unknown variable name: '{}'. Available variable names: [x, y, z | orientation]",
            //                         variable
            //                     ),
            //                 }
            //             }
            //         } else {
            //             assert!(
            //                 random_sample.len() >= 3,
            //                 "The distribution of an Clutter fault for OrientedRobot observation need to be of dimension 3."
            //             );
            //             o.pose.x = random_sample[0];
            //             o.pose.y = random_sample[1];
            //             o.pose.z = random_sample[2];
            //         }
            //         o.pose.z = mod2pi(o.pose.z);
            //         o.name = self.observation_id.clone();
            //         o.applied_faults
            //             .push(FaultModelConfig::Clutter(self.config.clone()));
            //     }
            //     SensorObservation::GNSS(o) => {
            //         if !self.variable_order.is_empty() {
            //             for (i, variable) in self.variable_order.iter().enumerate() {
            //                 match variable.as_str() {
            //                     "position_x" | "x" => o.pose.x = random_sample[i],
            //                     "position_y" | "y" => o.pose.y = random_sample[i],
            //                     "orientation" | "z" => o.pose.z = random_sample[i],
            //                     "velocity_x" => o.velocity.x = random_sample[i],
            //                     "velocity_y" => o.velocity.y = random_sample[i],
            //                     &_ => panic!(
            //                         "Unknown variable name: '{}'. Available variable names: [position_x | x, position_y | y, orientation | z, velocity_x, velocity_y]",
            //                         variable
            //                     ),
            //                 }
            //             }
            //         } else {
            //             assert!(
            //                 random_sample.len() >= 2,
            //                 "The distribution of an Clutter fault for GNSS observation need to be at least of dimension 2 (to 5 for orientation and velocities)."
            //             );
            //             o.pose.x = random_sample[0];
            //             o.pose.y = random_sample[1];
            //             if random_sample.len() >= 3 {
            //                 o.pose.z = random_sample[2];
            //             }
            //             if random_sample.len() >= 4 {
            //                 o.velocity.x = random_sample[3];
            //             }
            //             if random_sample.len() >= 5 {
            //                 o.velocity.y = random_sample[4];
            //             }
            //         }
            //         o.applied_faults
            //             .push(FaultModelConfig::Clutter(self.config.clone()));
            //     }
            //     SensorObservation::Speed(o) => {
            //         if !self.variable_order.is_empty() {
            //             for (i, variable) in self.variable_order.iter().enumerate() {
            //                 match variable.as_str() {
            //                     "w" | "angular" | "angular_velocity" => {
            //                         o.angular_velocity = random_sample[i]
            //                     }
            //                     "v" | "linear" | "linear_velocity" => {
            //                         o.linear_velocity = random_sample[i]
            //                     }
            //                     &_ => panic!(
            //                         "Unknown variable name: '{}'. Available variable names: [w | angular | angular_velocity, v | linear | linear_velocity]",
            //                         variable
            //                     ),
            //                 }
            //             }
            //         } else {
            //             assert!(
            //                 random_sample.len() >= 2,
            //                 "The distribution of an Clutter fault for Speed observation need to be of dimension 2."
            //             );
            //             o.angular_velocity = random_sample[0];
            //             o.linear_velocity = random_sample[1];
            //         }
            //         o.applied_faults
            //             .push(FaultModelConfig::Clutter(self.config.clone()));
            //     }
            //     SensorObservation::Displacement(o) => {
            //         if !self.variable_order.is_empty() {
            //             for (i, variable) in self.variable_order.iter().enumerate() {
            //                 match variable.as_str() {
            //                     "dx" | "x" => o.translation.x = random_sample[i],
            //                     "dy" | "y" => o.translation.y = random_sample[i],
            //                     "r" | "rotation" => o.rotation = random_sample[i],
            //                     &_ => panic!(
            //                         "Unknown variable name: '{}'. Available variable names: [dx | x, dy | y, r | rotation]",
            //                         variable
            //                     ),
            //                 }
            //             }
            //         } else {
            //             assert!(
            //                 random_sample.len() >= 2,
            //                 "The distribution of an Clutter fault for Displacement observation need to be at least of dimension 2 (to 3 for rotation)."
            //             );
            //             o.translation.x = random_sample[0];
            //             o.translation.y = random_sample[1];
            //             if random_sample.len() >= 3 {
            //                 o.rotation = random_sample[2];
            //             }
            //         }
            //         o.applied_faults
            //             .push(FaultModelConfig::Clutter(self.config.clone()));
            //     }
            //     SensorObservation::OrientedLandmark(o) => {
            //         if !self.variable_order.is_empty() {
            //             for (i, variable) in self.variable_order.iter().enumerate() {
            //                 match variable.as_str() {
            //                     "x" => o.pose.x = random_sample[i],
            //                     "y" => o.pose.y = random_sample[i],
            //                     "z" | "orientation" => o.pose.z = random_sample[i],
            //                     &_ => panic!(
            //                         "Unknown variable name: '{}'. Available variable names: [x, y, z | orientation]",
            //                         variable
            //                     ),
            //                 }
            //             }
            //         } else {
            //             assert!(
            //                 random_sample.len() >= 3,
            //                 "The distribution of an Clutter fault for OrientedLandmark observation need to be of dimension 3."
            //             );
            //             o.pose.x = random_sample[0];
            //             o.pose.y = random_sample[1];
            //             o.pose.z = random_sample[2];
            //         }
            //         o.pose.z = mod2pi(o.pose.z);
            //         o.id = self.observation_id.parse().unwrap_or(-1);
            //         o.applied_faults
            //             .push(FaultModelConfig::Clutter(self.config.clone()));
            //     }
            //     SensorObservation::Scan(_) => {
            //         // Add points directly to the observation list. Each point is not a different observation
            //         let mut new_r = 0.;
            //         let mut new_theta = 0.;
            //         let mut new_radial_velocity = 0.;
            //         if !self.variable_order.is_empty() {
            //             let mut new_x = 0.;
            //             let mut new_y = 0.;
            //             for (i, variable) in self.variable_order.iter().enumerate() {
            //                 match variable.as_str() {
            //                     "x" => new_x = random_sample[i],
            //                     "y" => new_y = random_sample[i],
            //                     "r" => new_r = random_sample[i],
            //                     "theta" => new_theta = random_sample[i],
            //                     "v" => new_radial_velocity = random_sample[i],
            //                     &_ => panic!(
            //                         "Unknown variable name: '{}'. Available variable names: [x, y, r, theta, v]",
            //                         variable
            //                     ),
            //                 }
            //                 new_r += new_x.hypot(new_y);
            //                 new_theta += new_y.atan2(new_x);
            //             }
            //         } else {
            //             assert!(
            //                 random_sample.len() >= 3,
            //                 "The distribution of an Clutter fault for Scan observation need to be of dimension 3 (r, theta, radial_velocity)."
            //             );
            //             new_r = random_sample[0];
            //             new_theta = random_sample[1];
            //             new_radial_velocity = random_sample[2];
            //         }
            //         if let SensorObservation::Scan(o) = &mut obs_list
            //             .first_mut()
            //             .expect("obs_list should not be empty (ClutterFault for Scan observations)")
            //         {
            //             o.distances.push(new_r);
            //             o.angles.push(new_theta);
            //             o.radial_velocities.push(new_radial_velocity);
            //             if clutter_number == 0 {
            //                 o.applied_faults
            //                     .push(FaultModelConfig::Clutter(self.config.clone()));
            //             }
            //         } else {
            //             panic!("obs_list should contain Scan observations (ClutterFault)");
            //         }
            //     }
            //     SensorObservation::External(_) => {
            //         panic!("ClutterFault cannot fault ExternalObservation");
            //     }
            // }
            // if matches!(new_obs, SensorObservation::Scan(_)) {
            //     continue;
            // }
            // obs_list.push(new_obs);
        }
        new_obs
    }

    pub fn config(&self) -> &ClutterFaultConfig<SV> {
        &self.config
    }
}
