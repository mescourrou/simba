/*!
Module providing the [`Controller`](controller::Controller) strategy, which computes the
[`Command`](crate::physics::physics::Command) sent to the [`Physics`](crate::physics::physics::Physics).
*/
pub mod controller;
pub mod external_controller;
pub mod pid;
pub mod python_controller;

pub mod pybinds;
