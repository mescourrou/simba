// Configuration for PID
use serde_derive::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Debug)]
#[serde(default)]
pub struct PIDConfig {
    kp_v: f32,
    kd_v: f32,
    ki_v: f32,
    kp_theta: f32,
    kd_theta: f32,
    ki_theta: f32,
    wheel_distance: f32,
}

impl Default for PIDConfig {
    fn default() -> Self {
        Self {
            kp_v: 1.0,
            kd_v: 0.,
            ki_v: 0.,
            kp_theta: 0.8,
            kd_theta: 0.3,
            ki_theta: 0.,
            wheel_distance: 0.25
        }
    }
}


#[derive(Debug)]
pub struct PID {
    kp_v: f32,
    kd_v: f32,
    ki_v: f32,
    kp_theta: f32,
    kd_theta: f32,
    ki_theta: f32,
    wheel_distance: f32,
    last_command_time: f32,
    v_integral: f32,
    theta_integral: f32,
    previous_velocity_error: f32,
    previous_theta_error: f32,
    velocity: f32
}

impl PID {
    pub fn new() -> Self {
        Self::from_config(&PIDConfig::default())
    }

    pub fn from_config(config: &PIDConfig) -> Self {
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
            velocity: 0.
        }
    }
}

use crate::controllers::controller::ControllerError;
use crate::physics::physic::Command;
use super::controller::Controller;

impl Controller for PID {
    fn make_command(&mut self, error: &ControllerError, time: f32) -> Command {
        let dt = time - self.last_command_time;
        assert!(dt > 0., "PID delta time should be positive: {} - {} = {} > 0", time, self.last_command_time, dt);
        
        self.v_integral += error.velocity * dt;
        self.theta_integral += error.theta * dt;

        let v_derivative = (error.velocity - self.previous_velocity_error) / dt;
        let theta_derivative = (error.theta - self.previous_theta_error) / dt;

        let correction_theta = (self.kp_theta * error.theta + self.ki_theta * self.theta_integral + self.kd_theta * theta_derivative) * self.wheel_distance;
        let correction_v = self.kp_v * error.velocity + self.ki_v * self.v_integral + self.kd_v * v_derivative;

        self.velocity = self.velocity + correction_v;
        self.previous_theta_error = error.theta;
        self.previous_velocity_error = error.velocity;
        self.last_command_time = time;

        Command {
            left_wheel_speed: self.velocity - correction_theta / 2.,
            right_wheel_speed: self.velocity + correction_theta / 2.
        }
    }
}