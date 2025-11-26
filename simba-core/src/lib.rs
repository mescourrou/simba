/*!
Multi robot simulator with multi-threading and without ticking time.

This crate provides a multi-robots simulator where each robot is executed in a separated thread.
The time is not linear, the simulator jump to the next time step as soon as possible, letting
the possibility to run faster than time, but also to take more time if needed.

All the simulator execution is based on the configuration file given. The configuration documentation is available below
and more extensively [here](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/user).

The simulator is modular and external modules can be used (through the [`plugin_api`]). There
are three main components:
- The [`physics`] module manages the real state of the robot. An external can be a real robot,
but the ground truth should be provided.
- The [`navigators`] module manages the definition of the trajectory, and calls the controller
which is part of the [`controllers`] module.
- The [`state_estimators`] module propose algorithms to estimate the state of the robot. It's
the module which was first considered as external, to test localization algorithms. To estimate
the state of the robot, this module uses the data produced by the [`sensors`] modules.

The entry-point for the simulator is [`simulator`], which provides a [`simulator::Simulator`]
struct to load a configuration, run the simulation, save the results and process the results.

For example, the simulator can be used as follows:
```no_run
use std::path::Path;
use simba::simulator::Simulator;

// Initialize the environment
Simulator::init_environment();
println!("Load configuration...");
let mut simulator = Simulator::from_config_path(
    Path::new("config_example/config.yaml"),
    &None, //<- plugin API, to load external modules
).unwrap();
// Show the simulator loaded configuration
simulator.show();
// Run the simulator for the time given in the configuration
// It also save the results to json
simulator.run().unwrap();
simulator.compute_results().unwrap();

```

*/

#![doc = include_str!("../../doc/user_manual/docs/config_documentation.md")]

use pyo3::prelude::*;

pub mod controllers;
pub mod logger;
pub mod navigators;
pub mod networking;
pub mod node;
pub mod physics;
pub mod recordable;
pub mod sensors;
pub mod simulator;
pub mod state_estimators;
pub mod utils;

pub mod plugin_api;
pub mod pybinds;
pub mod pywrappers;

pub mod api;
pub mod time_analysis;

#[cfg(feature = "gui")]
pub mod gui;

#[pymodule]
pub fn simba(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pybinds::make_python_bindings(m)?;
    Ok(())
}

pub mod constants;
pub mod errors;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
pub const AUTHORS: &str = env!("CARGO_PKG_AUTHORS");
