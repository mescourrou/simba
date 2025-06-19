# SiMBA: Multi-Robot Backend Simulator 

## User documentation

SiMBA is a simulator designed to abstract sensor measurements to facilitate the implementation of backend fusion algorithms or visual servoing. The simulator supports multiple robots, and the time rate is dynamic, waiting for the algorithms to finish before jumping to the next time step. It is written in Rust, and Python bindings are available, with a loss of performance. Its modular architecture allows for extension of different components while using the default version of the others.

Most of the simulator behavior is defined in the configuration file, using YAML syntax. A default behavior is defined for each component so that you do not need to specify the full configuration. The complete file structure is available [here](config_documentation.md). A detailed explanation of each config element is available [here](config/0_introduction.md).

## Node types
The simulator manages multiple nodes in a network. There are two kinds of nodes.

**Robots** are connected nodes that can move, sense and compute.

**Computation Units** are connected nodes that do not have a physical presence in the simulated world. They can only compute.

## Rust crate documentation
The simulator technical documentation (maybe useful for plugin development) is available [here](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba).