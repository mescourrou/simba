use config_checker::macros::Check;
use serde::{Deserialize, Serialize};

#[cfg(feature = "gui")]
use crate::gui::{utils::string_combobox, UIComponent};
use crate::{
    sensors::{sensor::SensorObservation, sensor_filters::SensorFilter},
    state_estimators::state_estimator::State,
};

#[derive(Debug, Clone, Serialize, Deserialize, Check)]
#[serde(deny_unknown_fields)]
#[serde(default)]
pub struct RangeFilterConfig {
    #[check(and(and(eq(self.variables.len(), self.min_range.len()), eq(self.variables.len(), self.max_range.len())), inside(self.variables.clone(), vec![
        "x".to_string(),
        "y".to_string(),
        "orientation".to_string(),
        "theta".to_string(),
        "position_x".to_string(),
        "position_y".to_string(),
        "velocity_x".to_string(),
        "velocity_y".to_string(),
        "w".to_string(),
        "v".to_string(),
        "self_velocity".to_string(),
        "target_velocity".to_string()]
    )))]
    pub variables: Vec<String>,
    pub min_range: Vec<f32>,
    pub max_range: Vec<f32>,
    pub inside: bool,
}

impl Default for RangeFilterConfig {
    fn default() -> Self {
        Self {
            variables: Vec::new(),
            min_range: Vec::new(),
            max_range: Vec::new(),
            inside: true,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for RangeFilterConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        _ctx: &egui::Context,
        _buffer_stack: &mut std::collections::BTreeMap<String, String>,
        _global_config: &crate::simulator::SimulatorConfig,
        _current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        ui.vertical(|ui| {
            let possible_variables = [
                "x",
                "y",
                "orientation",
                "theta",
                "position_x",
                "position_y",
                "velocity_x",
                "velocity_y",
                "w",
                "v",
                "self_velocity",
                "target_velocity",
            ]
            .iter()
            .map(|x| String::from(*x))
            .collect();
            ui.horizontal(|ui| {
                ui.label("Variable order:");
                for (i, var) in self.variables.iter_mut().enumerate() {
                    let unique_var_id = format!("variable-{i}-{unique_id}");
                    string_combobox(ui, &possible_variables, var, unique_var_id);
                }
                if !self.variables.is_empty() && ui.button("-").clicked() {
                    self.variables.pop();
                    self.max_range.pop();
                    self.min_range.pop();
                }
                if ui.button("+").clicked() {
                    self.variables.push(
                        possible_variables
                            .get(self.variables.len().min(possible_variables.len()))
                            .unwrap()
                            .clone(),
                    );
                    self.min_range.push(0.);
                    self.max_range.push(1.);
                }
            });

            for (i, variable) in self.variables.iter().enumerate() {
                ui.horizontal(|ui| {
                    ui.label(format!("Range for {}:", variable));
                    ui.add(egui::DragValue::new(&mut self.min_range[i]));
                    ui.label("-");
                    ui.add(egui::DragValue::new(&mut self.max_range[i]));
                });
            }

            ui.horizontal(|ui| {
                ui.label("Inside range:");
                ui.checkbox(&mut self.inside, "");
            });
        });
    }

    fn show(&self, ui: &mut egui::Ui, _ctx: &egui::Context, _unique_id: &str) {
        ui.vertical(|ui| {
            ui.label("Variable order:");
            for (i, var) in self.variables.iter().enumerate() {
                ui.label(format!(
                    "{}: [{} - {}]",
                    var, self.min_range[i], self.max_range[i]
                ));
            }
            ui.label(format!("Inside range: {}", self.inside));
        });
    }
}

#[derive(Debug, Clone)]
pub struct RangeFilter {
    config: RangeFilterConfig,
}

impl RangeFilter {
    pub fn from_config(config: &RangeFilterConfig) -> Self {
        Self {
            config: config.clone(),
        }
    }
}

impl SensorFilter for RangeFilter {
    fn filter(
        &self,
        observation: SensorObservation,
        observer_state: &State,
        observee_state: Option<&State>,
    ) -> Option<SensorObservation> {
        let mut keep = true;

        match &observation {
            SensorObservation::GNSS(obs) => {
                for (i, var) in self.config.variables.iter().enumerate() {
                    let value = match var.as_str() {
                        "x" | "position_x" => obs.position.x,
                        "y" | "position_y" => obs.position.y,
                        "velocity_x" => obs.velocity.x,
                        "velocity_y" => obs.velocity.y,
                        "self_velocity" => observer_state.velocity,
                        &_ => panic!("Unknown variable name: '{}'. Available variable names: [position_x | x, position_y | y, velocity_x, velocity_y, self_velocity]", self.config.variables[i])
                    };
                    let in_range =
                        value >= self.config.min_range[i] && value <= self.config.max_range[i];
                    if self.config.inside {
                        if !in_range {
                            keep = false;
                            break;
                        }
                    } else if in_range {
                        keep = false;
                        break;
                    }
                }
            }
            SensorObservation::Odometry(obs) => {
                for (i, var) in self.config.variables.iter().enumerate() {
                    let value = match var.as_str() {
                        "w" | "angular" | "angular_velocity" => obs.angular_velocity,
                        "v" | "linear" | "linear_velocity" => obs.linear_velocity,
                        &_ => panic!("Unknown variable name: '{}'. Available variable names: [w | angular | angular_velocity, v | linear | linear_velocity]", self.config.variables[i])
                    };
                    let in_range =
                        value >= self.config.min_range[i] && value <= self.config.max_range[i];
                    if self.config.inside {
                        if !in_range {
                            keep = false;
                            break;
                        }
                    } else if in_range {
                        keep = false;
                        break;
                    }
                }
            }
            SensorObservation::OrientedLandmark(obs) => {
                for (i, var) in self.config.variables.iter().enumerate() {
                    let in_range = match var.as_str() {
                        "r" => {
                            let d = obs.pose.fixed_rows::<2>(0).norm();
                            d >= self.config.min_range[i] && d <= self.config.max_range[i]
                        }
                        "theta" => {
                            let theta = obs.pose.y.atan2(obs.pose.x);
                            theta >= self.config.min_range[i] && theta <= self.config.max_range[i]
                        }
                        "x" => {
                            obs.pose.x >= self.config.min_range[i] && obs.pose.x <= self.config.max_range[i]
                        },
                        "y" => {
                            obs.pose.y >= self.config.min_range[i] && obs.pose.y <= self.config.max_range[i]
                        },
                        "z" | "orientation" => {
                            obs.pose.z >= self.config.min_range[i] && obs.pose.z <= self.config.max_range[i]
                        }
                        "self_velocity" => {
                            observer_state.velocity >= self.config.min_range[i] && observer_state.velocity <= self.config.max_range[i]
                        }
                        &_ => panic!("Unknown variable name: '{}'. Available variable names: [r, theta, x, y, z | orientation, self_velocity]", self.config.variables[i])
                    };
                    if self.config.inside {
                        if !in_range {
                            keep = false;
                            break;
                        }
                    } else if in_range {
                        keep = false;
                        break;
                    }
                }
            }
            SensorObservation::OrientedRobot(obs) => {
                for (i, var) in self.config.variables.iter().enumerate() {
                    let in_range = match var.as_str() {
                        "r" => {
                            let d = obs.pose.fixed_rows::<2>(0).norm();
                            d >= self.config.min_range[i] && d <= self.config.max_range[i]
                        }
                        "theta" => {
                            let theta = obs.pose.y.atan2(obs.pose.x);
                            theta >= self.config.min_range[i] && theta <= self.config.max_range[i]
                        }
                        "x" => {
                            obs.pose.x >= self.config.min_range[i] && obs.pose.x <= self.config.max_range[i]
                        },
                        "y" => {
                            obs.pose.y >= self.config.min_range[i] && obs.pose.y <= self.config.max_range[i]
                        },
                        "z" | "orientation" => {
                            obs.pose.z >= self.config.min_range[i] && obs.pose.z <= self.config.max_range[i]
                        }
                        "self_velocity" => {
                            observer_state.velocity >= self.config.min_range[i] && observer_state.velocity <= self.config.max_range[i]
                        }
                        "target_velocity" => {
                            if let Some(observee_state) = observee_state {
                                observee_state.velocity >= self.config.min_range[i] && observee_state.velocity <= self.config.max_range[i]
                            } else {
                                panic!("Observee state is required to filter on target_velocity with OrientedRobot observation");
                            }
                        }
                        &_ => panic!("Unknown variable name: '{}'. Available variable names: [r, theta, x, y, z | orientation, self_velocity, target_velocity]", self.config.variables[i])
                    };
                    if self.config.inside {
                        if !in_range {
                            keep = false;
                            break;
                        }
                    } else if in_range {
                        keep = false;
                        break;
                    }
                }
            }
        }

        if keep {
            Some(observation)
        } else {
            None
        }
    }
}
