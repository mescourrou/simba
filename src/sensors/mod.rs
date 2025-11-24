/*!
This module provides [`Sensor management`](sensor_manager::SensorManager) and
[`Sensor Implementation`](sensor::Sensor).

The [`SensorManager`](sensor_manager::SensorManager) manages all the sensors of a robot.

## How to add a new sensor ?

To add a new [`Sensor`](sensor::Sensor), you should implement the
[`Sensor`](sensor::Sensor) trait, and add your new implementation to the
[`SensorConfig`](sensor::SensorConfig) and the
[`SensorRecord`](sensor::SensorRecord) enumarations.
*/

pub mod gnss_sensor;
pub mod odometry_sensor;
pub mod oriented_landmark_sensor;
pub mod robot_sensor;
pub mod sensor;
pub mod sensor_manager;

pub mod fault_models;
pub mod sensor_filters;
