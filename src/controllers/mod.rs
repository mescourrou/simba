/*!
Module providing the [`Controller`](controller::Controller) strategy, which computes the
[`Command`](crate::physics::physic::Command) sent to the [`Physic`](crate::physics::physic::Physic).
*/
pub mod controller;
pub mod external_controller;
pub mod pid;

pub mod pybinds;
