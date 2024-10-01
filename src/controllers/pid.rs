/*!
Module providing the [`PID`] specification for the [`Controller`] strategy.

The [`PID`] controller uses three derivative of the error:
- `Proportional` - error itself
- `Integral` - integration of the error
- `Derivative` - derivative of the error

Each component has a gain, which can be set in [`PIDConfig`].
*/

use crate::plugin_api::PluginAPI;
use crate::simulator::SimulatorMetaConfig;
use crate::stateful::Stateful;
use log::error;
use serde_derive::{Deserialize, Serialize};

/// Configuration of the [`PID`], it contains the 3 gains for the velocity
/// control, 3 gain for the orientation control, and the wheel distance.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct PIDConfig {
    pub kp_v: f32,
    pub kd_v: f32,
    pub ki_v: f32,
    pub kp_theta: f32,
    pub kd_theta: f32,
    pub ki_theta: f32,
    pub wheel_distance: f32,
}

impl Default for PIDConfig {
    /// Defaut PID configuration.
    ///
    /// - Velocity: Proportionnal 1, other 0
    /// - Orientation: P=0.8, D=0.3
    /// - Wheel distance: 0.25
    fn default() -> Self {
        Self {
            kp_v: 1.0,
            kd_v: 0.,
            ki_v: 0.,
            kp_theta: 0.8,
            kd_theta: 0.3,
            ki_theta: 0.,
            wheel_distance: 0.25,
        }
    }
}

/// Record of the [`PID`] controller.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PIDRecord {
    pub v_integral: f32,
    pub theta_integral: f32,
    pub velocity: f32,
    pub command: Command,
    pub last_command_time: f32,
    pub previous_velocity_error: f32,
    pub previous_theta_error: f32,
}

impl Default for PIDRecord {
    fn default() -> Self {
        Self {
            v_integral: 0.,
            theta_integral: 0.,
            velocity: 0.,
            command: Command {
                left_wheel_speed: 0.,
                right_wheel_speed: 0.,
            },
            last_command_time: 0.,
            previous_velocity_error: 0.,
            previous_theta_error: 0.,
        }
    }
}

/// Proportional-integral-derivative controller for velocity and angle.
#[derive(Debug)]
pub struct PID {
    /// Velocity proportional gain
    kp_v: f32,
    /// Velocity derivative gain
    kd_v: f32,
    /// Velocity integral gain
    ki_v: f32,
    /// Orientation proportional gain
    kp_theta: f32,
    /// Orientation derivative gain
    kd_theta: f32,
    /// Orientation integral gain
    ki_theta: f32,
    /// Distance between the wheels
    wheel_distance: f32,
    /// Time of the last command
    last_command_time: f32,
    /// Current integral of the velocity
    v_integral: f32,
    /// Current integral of the orientation
    theta_integral: f32,
    /// Previous velocity error to compute the derivative
    previous_velocity_error: f32,
    /// Previous orientation error to compute the derivative
    previous_theta_error: f32,
    /// Current velocity
    velocity: f32,
    /// Current record
    current_record: PIDRecord,
}

impl PID {
    /// Makes a new default PID.
    pub fn new() -> Self {
        Self::from_config(&PIDConfig::default(), &None, SimulatorMetaConfig::default())
    }

    /// Makes a new [`PID`] from the given `config`.
    pub fn from_config(
        config: &PIDConfig,
        _plugin_api: &Option<Box<&dyn PluginAPI>>,
        _meta_config: SimulatorMetaConfig,
    ) -> Self {
        PID {
            kp_v: config.kp_v,
            kd_v: config.kd_v,
            ki_v: config.ki_v,
            kp_theta: config.kp_theta,
            kd_theta: config.kd_theta,
            ki_theta: config.ki_theta,
            wheel_distance: config.wheel_distance,
            last_command_time: 0.,
            v_integral: 0.,
            theta_integral: 0.,
            previous_velocity_error: 0.,
            previous_theta_error: 0.,
            velocity: 0.,
            current_record: PIDRecord::default(),
        }
    }
}

use super::controller::{Controller, ControllerRecord};
use crate::controllers::controller::ControllerError;
use crate::physics::physic::Command;
use crate::turtlebot::Turtlebot;

impl Controller for PID {
    fn make_command(
        &mut self,
        _turtle: &mut Turtlebot,
        error: &ControllerError,
        time: f32,
    ) -> Command {
        let dt = time - self.last_command_time;
        assert!(
            dt > 0.,
            "PID delta time should be positive: {} - {} = {} > 0",
            time,
            self.last_command_time,
            dt
        );

        self.v_integral += error.velocity * dt;
        self.theta_integral += error.theta * dt;

        let v_derivative = (error.velocity - self.previous_velocity_error) / dt;
        let theta_derivative = (error.theta - self.previous_theta_error) / dt;

        let correction_theta = (self.kp_theta * error.theta
            + self.ki_theta * self.theta_integral
            + self.kd_theta * theta_derivative)
            * self.wheel_distance;
        let correction_v =
            self.kp_v * error.velocity + self.ki_v * self.v_integral + self.kd_v * v_derivative;

        self.velocity = self.velocity + correction_v;
        self.previous_theta_error = error.theta;
        self.previous_velocity_error = error.velocity;
        self.last_command_time = time;

        let command = Command {
            left_wheel_speed: self.velocity - correction_theta / 2.,
            right_wheel_speed: self.velocity + correction_theta / 2.,
        };
        self.current_record = PIDRecord {
            v_integral: self.v_integral,
            theta_integral: self.theta_integral,
            velocity: self.velocity,
            command: command.clone(),
            last_command_time: self.last_command_time,
            previous_theta_error: self.previous_theta_error,
            previous_velocity_error: self.previous_velocity_error,
        };
        command
    }
}

impl Stateful<ControllerRecord> for PID {
    fn record(&self) -> ControllerRecord {
        ControllerRecord::PID(self.current_record.clone())
    }

    fn from_record(&mut self, record: ControllerRecord) {
        if let ControllerRecord::PID(pid_record) = record {
            self.current_record = pid_record.clone();
            self.v_integral = pid_record.v_integral;
            self.theta_integral = pid_record.theta_integral;
            self.velocity = pid_record.velocity;
            self.last_command_time = pid_record.last_command_time;
            self.previous_theta_error = pid_record.previous_theta_error;
            self.previous_velocity_error = pid_record.previous_velocity_error;
        } else {
            error!("Using a ControllerRecord type which does not match the used Controller (PID)");
        }
    }
}
