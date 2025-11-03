/*!
Module providing the [`Navigator`](navigator::Navigator) strategy, which
compute the error from the desired position.

This module also propose a implemented strategy, [`trajectory_follower`].
*/

pub mod go_to;
pub mod navigator;
pub mod trajectory;
pub mod trajectory_follower;

pub mod external_navigator;
pub mod python_navigator;

pub mod pybinds;
