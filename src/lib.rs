/*!
Multi robot simulator with multi-threading and without ticking time

This crate provides a multi-robots simulator where each robot is executed in a separated thread.
The time is not linear, the simulator jump to the next time step as soon as possible, letting
the possibility to run faster than time, but also to take more time if needed.

All the simulator execution is based on the configuration file given. The configuration documentation is available below.

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
```
use std::path::Path;
use simba::simulator::Simulator;

fn main() {
    // Initialize the environment, essentially the logging part
    Simulator::init_environment();

    // Load the configuration
    let config_path = Path::new("config_example/config.yaml");
    let mut simulator = Simulator::from_config_path(
        config_path,              //<- configuration path
        None,                     //<- plugin API, to load external modules
        Some(Box::from(Path::new("result.json"))), //<- path to save the results (None to not save)
        true,                     //<- Analyse the results
        false,                    //<- Show the figures after analyse
    );

    // Show the simulator loaded configuration
    simulator.show();

    // Run the simulation for 60 seconds.
    // It also save the results to "result.json",
    // compute the results and show the figures.
    simulator.run(60.);
}


```

*/

#![doc = include_str!("../doc/config_documentation.md")]

use pyo3::prelude::*;

pub mod controllers;
pub mod navigators;
pub mod networking;
pub mod node;
pub mod node_factory;
pub mod physics;
pub mod sensors;
pub mod simulator;
pub mod state_estimators;
pub mod stateful;
pub mod utils;

pub mod plugin_api;
pub mod pybinds;
mod pywrappers;

pub mod api;
pub mod time_analysis;

#[cfg(feature = "gui")]
pub mod gui;

#[pymodule]
fn simba(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pybinds::make_python_bindings(m)?;
    Ok(())
}

pub mod constants;
pub mod errors;
