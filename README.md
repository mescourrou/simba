# SiMBA: Simulator for Multi-robot Backend Algorithms

SiMBA is a simulator designed to abstract sensor measurements to facilitate the implementation of backend fusion algorithms or visual servoing. The simulator supports multiple robots, and the time rate is dynamic, waiting for the algorithms to finish before jumping to the next time step. It is written in Rust, and Python bindings are available, with a loss of performance. Its modular architecture allows for extension of different components while using the default version of the others.

## Features
Here is a list of available and considered features. The order is not a priority order.
- [X] First simple control-state estimation-physics loop
- [X] Message exchange between robots
- [X] Automatic result treatment at the end of runs
- [X] Basic sensors: displacement, speed, GNSS, relative landmark, inter-robot, scan
- [X] State estimator test bench
- [X] Determinist random variables for noise simulation
- [X] Python bindings: state estimators, controllers, navigators and physics
- [X] Time performance analysis
- [X] Complex noise addition and faults
- [X] Graphical User Interface (real time + replay)
- [X] PluginAPI to provide custom implementations of components (controllers, state estimators, physics, etc.)
- [X] Triggerable sensors
- [X] Centralized algorithm possibility
- [ ] Export data to file
- [ ] Possibility to read data from file instead of simulation: test state estimation algorithms on real data
- [ ] ROS2 interface (rosbag reading)
- [ ] Gazebo interface (maybe)
- [ ] C++ bindings
- [ ] New faults: delay, activation windows

## Crates
- `simba-core`: Main library.
- `simba-cmd`: Command line tool to launch the simulator on a given configuration file.
- `simba-tools`: Command line tool for developers.
- `simba-macros`: Procedural macros used in `simba-core`.
- `simba-com`: Library for message exchange between nodes and synchronization.

## Cargo Features:
- "gui": enable GUI running. Use `gui::run_gui` to start a GUI.

## Documentation
You can compile the documentation using Cargo:
```
cargo doc --no-deps
```

The last Rust documentation is available [here](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba).

The last user documentation is available [here](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/user).

**Configuration autocompletion**: To autocomplete YAML config files, you can use the json schemas available in the repo or in the release.
Just add the following line at the beginning of your file: `# yaml-language-server: $schema=path/to/your/config.schema.json`

## Build
```
cargo build
```

## Spirit of this simulator
- All the behaviors should be controlled by the config file. To avoid huge config files, default behavior should be given.
- Modularity: the different behaviors should have common interfaces.
- The simulation should be as fast as possible, without loosing data. All algorithms should finish before going to the next time step.
- Determinism: two run should give exactly the same results: no randomness.

## Contribution
You can contribute by submitting a Pull request. Discussion should occur in case of breaking changes (especially in the configuration structure).

Don't forget to install git hooks: `./install-hooks.sh`

## Python bindings
To make python bindings, we use `PyO3`.

1) Install a virtual env and activate it:
```
python -m venv .env
source .env/bin/activate
```
2) Then, install maturin:
`pip install maturin`
3) To build the bindings, use `maturin develop`. It needs to be run in the virtualenv
4) To test that the package was built successfully, you can import it in python: `python -m simba`
