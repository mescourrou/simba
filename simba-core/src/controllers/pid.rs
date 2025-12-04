/*!
Module providing the [`PID`] specification for the [`Controller`] strategy.

The [`PID`] controller uses three derivative of the error:
- `Proportional` - error itself
- `Integral` - integration of the error
- `Derivative` - derivative of the error

Each component has a gain, which can be set in [`PIDConfig`].
*/

use std::sync::mpsc::Sender;

use crate::networking::message_handler::MessageHandler;
use crate::networking::network::Envelope;
use crate::physics::internal_physics::InternalPhysicConfig;
use crate::physics::robot_models::holonomic::HolonomicCommand;
use crate::physics::robot_models::unicycle::UnicycleCommand;
use crate::physics::robot_models::{Command, RobotModelConfig};
use crate::physics::PhysicsConfig;
use crate::recordable::Recordable;
use crate::utils::maths::{Derivator, Integrator};
#[cfg(feature = "gui")]
use crate::{gui::UIComponent, simulator::SimulatorConfig};
use config_checker::ConfigCheckable;
use log::warn;
use serde::de::{MapAccess, SeqAccess, Visitor};
use serde::{de, Deserializer};
use serde_derive::{Deserialize, Serialize};
use simba_macros::config_derives;

/// Configuration of the [`PID`], it contains the 3 list of gains:
/// proportional gains, derivative gains and integral gains.
/// The size of each gains depends on the model used, and follow this order:
/// - longitudinal (All models)
/// - lateral (Holonomic model)
/// - angular (All models)
#[config_derives(skip_check, skip_deserialize)]
#[derive(Default)]
pub struct PIDConfig {
    pub robot_model: Option<RobotModelConfig>,
    pub proportional_gains: Vec<f32>,
    pub derivative_gains: Vec<f32>,
    pub integral_gains: Vec<f32>,
}

impl PIDConfig {
    fn default_from_model(config: &RobotModelConfig) -> Self {
        match config {
            // Order: longitudinal, angular
            RobotModelConfig::Unicycle(_) => Self {
                robot_model: Some(config.clone()),
                proportional_gains: vec![1., 1.],
                derivative_gains: vec![0., 0.1],
                integral_gains: vec![0., 0.],
            },
            // Order: longitudinal, lateral, angular
            RobotModelConfig::Holonomic(_) => Self {
                robot_model: Some(config.clone()),
                proportional_gains: vec![1., 1., 1.],
                derivative_gains: vec![0., 0., 0.1],
                integral_gains: vec![0., 0., 0.],
            },
        }
    }

    fn kp_longitudinal(&self) -> Option<f32> {
        Some(self.proportional_gains[0])
    }

    fn kp_lateral(&self) -> Option<f32> {
        match self.robot_model.as_ref().unwrap() {
            RobotModelConfig::Unicycle(_) => None,
            RobotModelConfig::Holonomic(_) => Some(self.proportional_gains[1]),
        }
    }

    fn kp_angular(&self) -> Option<f32> {
        match self.robot_model.as_ref().unwrap() {
            RobotModelConfig::Unicycle(_) => Some(self.proportional_gains[1]),
            RobotModelConfig::Holonomic(_) => Some(self.proportional_gains[2]),
        }
    }

    fn ki_longitudinal(&self) -> Option<f32> {
        Some(self.integral_gains[0])
    }

    fn ki_lateral(&self) -> Option<f32> {
        match self.robot_model.as_ref().unwrap() {
            RobotModelConfig::Unicycle(_) => None,
            RobotModelConfig::Holonomic(_) => Some(self.integral_gains[1]),
        }
    }

    fn ki_angular(&self) -> Option<f32> {
        match self.robot_model.as_ref().unwrap() {
            RobotModelConfig::Unicycle(_) => Some(self.integral_gains[1]),
            RobotModelConfig::Holonomic(_) => Some(self.integral_gains[2]),
        }
    }

    fn kd_longitudinal(&self) -> Option<f32> {
        Some(self.derivative_gains[0])
    }

    fn kd_lateral(&self) -> Option<f32> {
        match self.robot_model.as_ref().unwrap() {
            RobotModelConfig::Unicycle(_) => None,
            RobotModelConfig::Holonomic(_) => Some(self.derivative_gains[1]),
        }
    }

    fn kd_angular(&self) -> Option<f32> {
        match self.robot_model.as_ref().unwrap() {
            RobotModelConfig::Unicycle(_) => Some(self.derivative_gains[1]),
            RobotModelConfig::Holonomic(_) => Some(self.derivative_gains[2]),
        }
    }
}

impl ConfigCheckable for PIDConfig {
    fn __check(&self, depth: usize) -> Result<(), String> {
        use colored::Colorize;
        let depth_space = vec!["| "; depth].join("");
        let mut ret = Ok(());
        if self.robot_model.is_none() {
            warn!(
                "{} No model given to PID controller, will use physics' one or default",
                "NOTE:".blue()
            );
            return Ok(());
        }
        let canonical_config = Self::default_from_model(self.robot_model.as_ref().unwrap());
        if canonical_config.proportional_gains.len() != self.proportional_gains.len() {
            if ret.is_ok() {
                ret = Err(String::new());
            }
            ret = Err(ret.err().unwrap() + format!("{}  {depth_space}Length of proportional gains mismatch ({} vs {} expected for {} model)\n", "ERROR:".red(), self.proportional_gains.len(), canonical_config.proportional_gains.len(), self.robot_model.as_ref().unwrap()).as_str());
        }
        if canonical_config.integral_gains.len() != self.integral_gains.len() {
            if ret.is_ok() {
                ret = Err(String::new());
            }
            ret = Err(ret.err().unwrap() + format!("{}  {depth_space}Length of integral gains mismatch ({} vs {} expected for {} model)\n", "ERROR:".red(), self.integral_gains.len(), canonical_config.integral_gains.len(), self.robot_model.as_ref().unwrap()).as_str());
        }
        if canonical_config.derivative_gains.len() != self.derivative_gains.len() {
            if ret.is_ok() {
                ret = Err(String::new());
            }
            ret = Err(ret.err().unwrap() + format!("{}  {depth_space}Length of derivative gains mismatch ({} vs {} expected for {} model)\n", "ERROR:".red(), self.derivative_gains.len(), canonical_config.derivative_gains.len(), self.robot_model.as_ref().unwrap()).as_str());
        }
        ret
    }

    fn check(&self) -> Result<(), String> {
        self.__check(0)
    }
}

/// Custom deserialization to fill missing gains with zeros depending on the model.
/// Looking for a method to use serde derive instead with the additional logic.
impl<'de> serde::Deserialize<'de> for PIDConfig {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        enum Field {
            RobotModel,
            ProportionalGains,
            DerivativeGains,
            IntegralGains,
        }

        // This part could also be generated independently by:
        //
        //    #[derive(Deserialize)]
        //    #[serde(field_identifier, rename_all = "lowercase")]
        //    enum Field { Secs, Nanos }
        impl<'de> serde::Deserialize<'de> for Field {
            fn deserialize<D>(deserializer: D) -> Result<Field, D::Error>
            where
                D: Deserializer<'de>,
            {
                struct FieldVisitor;

                impl<'de> Visitor<'de> for FieldVisitor {
                    type Value = Field;

                    fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                        formatter.write_str("`robot_model`, `proportional_gains`, `derivative_gains` or `integral_gains`")
                    }

                    fn visit_str<E>(self, value: &str) -> Result<Field, E>
                    where
                        E: de::Error,
                    {
                        match value {
                            "robot_model" => Ok(Field::RobotModel),
                            "proportional_gains" => Ok(Field::ProportionalGains),
                            "derivative_gains" => Ok(Field::DerivativeGains),
                            "integral_gains" => Ok(Field::IntegralGains),
                            _ => Err(de::Error::unknown_field(value, FIELDS)),
                        }
                    }
                }

                deserializer.deserialize_identifier(FieldVisitor)
            }
        }

        struct PIDConfigVisitor;

        impl<'de> Visitor<'de> for PIDConfigVisitor {
            type Value = PIDConfig;

            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                formatter.write_str("struct PIDConfig")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<PIDConfig, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let robot_model = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                let proportional_gains = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(1, &self))?;
                let derivative_gains = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(2, &self))?;
                let integral_gains = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(3, &self))?;
                Ok(PIDConfig {
                    robot_model,
                    proportional_gains,
                    derivative_gains,
                    integral_gains,
                })
            }

            fn visit_map<V>(self, mut map: V) -> Result<PIDConfig, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut robot_model = None;
                let mut proportional_gains = None;
                let mut derivative_gains = None;
                let mut integral_gains = None;
                while let Some(key) = map.next_key()? {
                    match key {
                        Field::RobotModel => {
                            if robot_model.is_some() {
                                return Err(de::Error::duplicate_field("robot_model"));
                            }
                            robot_model = Some(map.next_value()?);
                        }
                        Field::ProportionalGains => {
                            if proportional_gains.is_some() {
                                return Err(de::Error::duplicate_field("proportional_gains"));
                            }
                            proportional_gains = Some(map.next_value()?);
                        }
                        Field::DerivativeGains => {
                            if derivative_gains.is_some() {
                                return Err(de::Error::duplicate_field("derivative_gains"));
                            }
                            derivative_gains = Some(map.next_value()?);
                        }
                        Field::IntegralGains => {
                            if integral_gains.is_some() {
                                return Err(de::Error::duplicate_field("integral_gains"));
                            }
                            integral_gains = Some(map.next_value()?);
                        }
                    }
                }
                let default_config = PIDConfig::default();
                let robot_model = robot_model.unwrap_or(default_config.robot_model);
                let proportional_gains =
                    proportional_gains.unwrap_or(default_config.proportional_gains);
                let derivative_gains = derivative_gains.unwrap_or(default_config.derivative_gains);
                let integral_gains = integral_gains.unwrap_or(default_config.integral_gains);
                Ok(PIDConfig {
                    robot_model,
                    proportional_gains,
                    derivative_gains,
                    integral_gains,
                })
            }
        }

        const FIELDS: &[&str] = &[
            "robot_model",
            "proportional_gains",
            "derivative_gains",
            "integral_gains",
        ];
        let mut config = deserializer.deserialize_struct("PIDConfig", FIELDS, PIDConfigVisitor)?;

        if config.robot_model.is_some() {
            let canonical_config = Self::default_from_model(config.robot_model.as_ref().unwrap());
            if config.proportional_gains.is_empty() {
                config.proportional_gains = vec![0.0; canonical_config.proportional_gains.len()];
            }
            if config.integral_gains.is_empty() {
                config.integral_gains = vec![0.0; canonical_config.integral_gains.len()];
            }
            if config.derivative_gains.is_empty() {
                config.derivative_gains = vec![0.0; canonical_config.derivative_gains.len()];
            }
        }
        Ok(config)
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PIDConfig {
    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        if self.robot_model.is_none() {
            *self = Self::default();
        }
        egui::CollapsingHeader::new("PID")
            .id_salt(format!("pid-{}", unique_id))
            .show(ui, |ui| {
                let previous_model = self.robot_model.as_ref().unwrap().to_string();
                self.robot_model.as_mut().unwrap().show_mut(
                    ui,
                    ctx,
                    buffer_stack,
                    global_config,
                    current_node_name,
                    unique_id,
                );
                if self.robot_model.as_ref().unwrap().to_string() != previous_model {
                    *self = Self::default_from_model(self.robot_model.as_ref().unwrap());
                }

                let label_order = match self.robot_model.as_ref().unwrap() {
                    RobotModelConfig::Holonomic(_) => vec!["Longitudinal", "Lateral", "Angular"],
                    RobotModelConfig::Unicycle(_) => vec!["Longitudinal", "Angular"],
                };

                for (i, label) in label_order.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("{} - P:", *label));
                        ui.add(
                            egui::DragValue::new(&mut self.proportional_gains[i]).max_decimals(10),
                        );
                    });

                    ui.horizontal(|ui| {
                        ui.label(format!("{} - I:", *label));
                        ui.add(egui::DragValue::new(&mut self.integral_gains[i]).max_decimals(10));
                    });

                    ui.horizontal(|ui| {
                        ui.label(format!("{} - D:", *label));
                        ui.add(
                            egui::DragValue::new(&mut self.derivative_gains[i]).max_decimals(10),
                        );
                    });
                }
            });
    }

    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        egui::CollapsingHeader::new("PID")
            .id_salt(format!("pid-{}", unique_id))
            .show(ui, |ui| {
                self.robot_model.as_ref().unwrap().show(ui, ctx, unique_id);

                let label_order = match self.robot_model.as_ref().unwrap() {
                    RobotModelConfig::Holonomic(_) => vec!["Longitudinal", "Lateral", "Angular"],
                    RobotModelConfig::Unicycle(_) => vec!["Longitudinal", "Angular"],
                };

                for (i, label) in label_order.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("{} - P: {}", *label, self.proportional_gains[i]));
                    });

                    ui.horizontal(|ui| {
                        ui.label(format!("{} - I: {}", *label, self.integral_gains[i]));
                    });

                    ui.horizontal(|ui| {
                        ui.label(format!("{} - D: {}", *label, self.derivative_gains[i]));
                    });
                }
            });
    }
}

/// Record of the [`PID`] controller.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PIDRecord {
    pub config: PIDConfig,
    pub velocity: f32,
    pub command: Command,
    pub last_command_time: f32,
}

impl Default for PIDRecord {
    fn default() -> Self {
        Self {
            config: PIDConfig::default(),
            velocity: 0.,
            command: Command::default(),
            last_command_time: 0.,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PIDRecord {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| {
            egui::CollapsingHeader::new("Config").show(ui, |ui| {
                self.config.show(ui, ctx, unique_id);
            });
            egui::CollapsingHeader::new("Command").show(ui, |ui| {
                self.command.show(ui, ctx, unique_id);
            });
            ui.label(format!("velocity: {}", self.velocity));
            ui.label(format!("last command time: {}", self.last_command_time));
        });
    }
}

/// Proportional-integral-derivative controller for velocity and angle.
#[derive(Debug)]
pub struct PID {
    config: PIDConfig,
    /// Time of the last command
    last_command_time: f32,
    /// Integrator for longitudinal error
    longitudinal_integrator: Integrator,
    /// Integrator for lateral error
    lateral_integrator: Integrator,
    /// Integrator for angular error
    angular_integrator: Integrator,
    /// Derivator for longitudinal error
    longitudinal_derivator: Derivator,
    /// Derivator for lateral error
    lateral_derivator: Derivator,
    /// Derivator for angular error
    angular_derivator: Derivator,
    /// Current velocity
    velocity: f32,
    /// Integrator for velocity error
    velocity_integrator: Integrator,
    /// Derivator for velocity error
    velocity_derivator: Derivator,
    /// Current record
    current_record: PIDRecord,
}

impl PID {
    /// Makes a new default PID.
    pub fn new() -> Self {
        Self::from_config(
            &PIDConfig::default(),
            &PhysicsConfig::Internal(InternalPhysicConfig::default()),
        )
    }

    /// Makes a new [`PID`] from the given `config`.
    pub fn from_config(config: &PIDConfig, physics_config: &PhysicsConfig) -> Self {
        let mut config_clone = config.clone();
        if config.robot_model.is_none() {
            if let PhysicsConfig::Internal(InternalPhysicConfig {
                model,
                faults: _,
                initial_state: _,
            }) = physics_config
            {
                config_clone.robot_model = Some(model.clone());
                if config_clone.check().is_err() {
                    config_clone = PIDConfig::default_from_model(model);
                    warn!("No model given in PID Config and gains given mismatch physics model ({}) => resetting gains", model);
                }
            } else {
                config_clone = PIDConfig::default();
                warn!("No model given in PID Config... using default one and resetting gains");
            }
        }
        PID {
            config: config_clone,
            last_command_time: 0.,
            longitudinal_integrator: Integrator::new(),
            lateral_integrator: Integrator::new(),
            angular_integrator: Integrator::new(),
            longitudinal_derivator: Derivator::new(),
            lateral_derivator: Derivator::new(),
            angular_derivator: Derivator::new(),
            velocity: 0.,
            velocity_integrator: Integrator::new(),
            velocity_derivator: Derivator::new(),
            current_record: PIDRecord::default(),
        }
    }
}

impl Default for PID {
    fn default() -> Self {
        Self::new()
    }
}

use super::{Controller, ControllerRecord};
use crate::controllers::ControllerError;
use crate::node::Node;

impl Controller for PID {
    fn make_command(&mut self, _robot: &mut Node, error: &ControllerError, time: f32) -> Command {
        let dt = time - self.last_command_time;
        assert!(
            dt > 0.,
            "PID delta time should be positive: {} - {} = {} > 0",
            time,
            self.last_command_time,
            dt
        );

        let command = match self.config.robot_model.as_ref().expect("Robot model should be set in PID config at least automatically from physics (if physics is internal)") {
            RobotModelConfig::Unicycle(model) => {
                self.velocity_integrator.integrate(error.velocity, dt);
                self.angular_integrator.integrate(error.theta, dt);

                let v_derivative = self.velocity_derivator.derivate(error.velocity, dt);
                let theta_derivative = self.angular_derivator.derivate(error.theta, dt);

                let correction_theta = (self.config.kp_angular().unwrap() * error.theta
                    + self.config.ki_angular().unwrap() * self.angular_integrator.integral_value()
                    + self.config.kd_angular().unwrap() * theta_derivative)
                    * model.wheel_distance;
                let correction_v = self.config.kp_longitudinal().unwrap() * error.velocity + self.config.ki_longitudinal().unwrap() * self.velocity_integrator.integral_value() + self.config.kd_longitudinal().unwrap() * v_derivative;

                self.velocity += correction_v;
                self.last_command_time = time;

                Command::Unicycle(UnicycleCommand {
                        left_wheel_speed: self.velocity - correction_theta / 2.,
                        right_wheel_speed: self.velocity + correction_theta / 2.,
                })
            },
            RobotModelConfig::Holonomic(_model) => {
                self.velocity_integrator.integrate(error.velocity, dt);
                self.lateral_integrator.integrate(error.lateral, dt);
                self.angular_integrator.integrate(error.theta, dt);
                self.longitudinal_integrator.integrate(error.longitudinal, dt);

                let v_derivative = self.velocity_derivator.derivate(error.velocity, dt);
                let lateral_derivative = self.lateral_derivator.derivate(error.lateral, dt);
                let theta_derivative = self.angular_derivator.derivate(error.theta, dt);
                let longitudinal_derivative = self.longitudinal_derivator.derivate(error.longitudinal, dt);
                let correction_v = self.config.kp_longitudinal().unwrap() * error.velocity + self.config.ki_longitudinal().unwrap() * self.velocity_integrator.integral_value() + self.config.kd_longitudinal().unwrap() * v_derivative;

                let correction_theta = self.config.kp_angular().unwrap() * error.theta
                    + self.config.ki_angular().unwrap() * self.angular_integrator.integral_value()
                    + self.config.kd_angular().unwrap() * theta_derivative;

                self.velocity += correction_v;
                self.last_command_time = time;

                let correction_lateral =
                    (self.config.kp_lateral().unwrap() * error.lateral + self.config.ki_lateral().unwrap() * self.lateral_integrator.integral_value() + self.config.kd_lateral().unwrap() * lateral_derivative) * self.velocity;
                let correction_longitudinal =
                    (self.config.kp_longitudinal().unwrap() * error.longitudinal + self.config.ki_longitudinal().unwrap() * self.longitudinal_integrator.integral_value() + self.config.kd_longitudinal().unwrap() * longitudinal_derivative) * self.velocity;
                Command::Holonomic(HolonomicCommand {
                    longitudinal_velocity: correction_longitudinal,
                    lateral_velocity: correction_lateral,
                    angular_velocity: correction_theta,
                })
            }
        };

        self.current_record = PIDRecord {
            config: self.config.clone(),
            velocity: self.velocity,
            command: command.clone(),
            last_command_time: self.last_command_time,
        };
        command
    }

    fn pre_loop_hook(&mut self, _node: &mut Node, _time: f32) {}
}

impl Recordable<ControllerRecord> for PID {
    fn record(&self) -> ControllerRecord {
        ControllerRecord::PID(self.current_record.clone())
    }
}

impl MessageHandler for PID {
    fn get_letter_box(&self) -> Option<Sender<Envelope>> {
        None
    }
}
