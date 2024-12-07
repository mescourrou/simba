# SiMBA: Simulator for Multi-robot Backend Algorithms

## Features
Here is a list of available and considered features. The order is not a priority order.
- [X] First simple control-state estimation-physics loop
- [X] One-way messages exchanges between robots
- [X] Two-way messages with client blocking (services)
- [X] Automatic result treatment at the end of runs
- [X] Basic sensors: odometry, GNSS, relative landmark, inter-robot
- [X] State estimator test bench
- [X] Determinist random variables for noise simulation
- [X] Python bindings (for only one robot)
- [ ] Synchronous robot run option
- [ ] Back-in-time optimization: jump to present if no modification was made in the past to avoid unnecessary computation
- [ ] Time performance analysis
- [ ] Possibility to read data from file instead of simulation: test state estimation algorithms on real data
- [ ] ROS2 interface (rosbag reading)
- [ ] Gazebo interface (maybe)
- [ ] C++ bindings
- [ ] More complex noise addition (+ faults)
- [ ] Graphical User Interface (real time + replay)
- [ ] Extend PluginAPI to controller, navigators, physics and sensors

## Documentation
You can compile the documentation using Cargo:
```
cargo doc --no-deps --document-private-items
```

The last documentation is available [here](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/doc/turtlebot_simulator/index.html).

## Spirit of this simulator
- All the behaviors should be controlled by the config file. To avoid huge config files, default behavior should be given.
- Modularity: the different behaviors should have common interfaces.
- The simulation should be as fast as possible, without loosing data. All algorithm should finish before going to the next time step.
- Determinism: two run should give exactly the same results: no randomness.

## Contribution
You can contribute by submitting a Pull request. Discussion should occur in case of breaking changes (especially in the configuration structure).

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
4) To test that the package was built successfully, you can import it in python: `python -m turtlebot_simulator`


### Waiting for PyO3
Python bindings are almost set for state estimators. However, the Python GIL locks with turtles threads. I wait for the PyO3 community to implement sub interpreters to fix that (see [Issue 576](https://github.com/PyO3/pyo3/issues/576)).