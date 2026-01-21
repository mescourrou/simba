# Changelogs

## v1.4.3
GUI:
- Previous results loading only on frontend side: possibility to load results without required plugins
- Add < and > buttons to jump to previous/next computed instant (1st version)
- GNSS observations visualization (position and velocity)

Features:
- Add Identifier and Label filter for landmark and robot sensors
- Displacement sensor
- Add orientation to GNSS observation
- Additive noise option to be proportional to time or distance (only available for Displacement sensor and AdditiveRobotCentered noise).
 
Fixes:
- Goto error computation
- PID Holonomic command computation
- Remove lie integration in odometry sensor
- Velocity norm. Bug introduced with the addition of angular velocity in velocity state vector

Deprecated:
- Rename OdometrySensor to SpeedSensor

## v1.4.2
- hotfix: Use Python abi3 for Python >= 3.10 for Python call from Rust.

## v1.4.1
- hotfix: Make possible to create Goto and SensorTrigger Messages in Python

## v1.4.0
- [breaking] Add scenarios: kill or spawn nodes on events triggered by time, node proximity or area.
- [breaking] Multiple robot models: Unicycle and Holonomic for now.
- [breaking] Remove unique seeds for each sensors
- Planar landmarks, with occlusions.
- Physics fault models
- Sensor triggering by message
- [breaking] Config file autocompletion
- External sensors, external sensor filters and external sensor fault (python)
- Rust edition 2024

## v1.3.0
- [breaking rust] Huge refactoring
- [breaking] Choice on robot model, impacts PID configuration and controller implementations
- Sensor observation filters: Acceptable range on multiple variables (x, y, orientation, theta,
  position_x, position_y, velocity_x, velocity_y, w, v, self_velocity, target_velocity)
- [breaking] Tracking config checker commit tag (previous was `623bde9`)
- Physics fault models
- Optimize record collection when no results are saved
- Possible step-by-step (ie 10 secs then 20 secs) run
- [breaking] Plugin into plugin possibility (see stacked_plugin example)
- [breaking] Message passing for plugins (rust and python). Careful, no filter on messages for now
- [minor break] Loading previous results to only analyse them using command line executable

## v1.2.0
- GoTo Navigator: send a location and the robot goes.
- Option for saving: At the end, continuously, by batch or periodically.
- [breaking] Killing nodes by message.
- [breaking] Send and receive messages with Python bindings.
- Version tag in configuration file with warning if it differs.
- Fix TrajectoryFollower when trajectory without loop.

## v1.1.0
- [breaking] Synchronous parallel execution, remove need for from_record implementation
- [breaking] Change of enumeration representation YAML configuration file: now using tags (ie `!RobotSensor`)
- New command line tool
- External Python possible directly from Rust (unstable with Python package)
- Random seed if not specified
- Default config loading when running GUI

## v1.0.2
- Facilitate python development
- Upload wheel file with the right name for pip install
- Fix missing observations generation
- Fix error return for run and results in GUI

## v1.0.1
- Introspection GUI tools for records and current config
- Extend simulation GUI area dynamically
- Fix PluginAPI usage for GUI in Rust
- Export base_path in SimulatorConfig when serializing
- Fix config doc link to rust doc

## v1.0.0
- New GUI for simulation and configuration
- Better documentation
- Async API wait optimization
- [soft breaking] Error management
- [breaking] World State given to the navigator instead of only robot pose

## v0.5.0
- [breaking] Network node generalization
    Allows:
    - Different types of nodes (robots, computation unit for now, sensor and
      movable object later).
    - Mechanism to automatically send node observation to another, allowing
      centralized algorithms.
    `Warning`: if network are configured with 0 reception delay, messages can
    be read later or deadlock can occur.
- [breaking] Move log config into config file, add internal level of debug
- [breaking] Change sensor configuration to auto-send observation to another node.
- Synchronous execution and time management (faster when messages are exchanged).

## v0.4.3
- Fix CI to build Multi-version python wheels
- Fix state estimator python example

## v0.4.2
- Python bindings for controllers, navigators and physics
- Working result analysis with python invocation
- Config documentation (auto-generated)
- No more submodule for rust dependencies
- Using Lie theory for physics simulation

## v0.4.0
- [breaking][Rust] Filter logs by robot
- [breaking][Rust and Config] Run time from configuration
- [breaking][Rust] Change nalgebra version
- [breaking][Config] Specific configuration for results analysis, including custom parameter for analysis script
- Fault models for noise, bias, clutter, misidentification and misdetection
- Basic GUI
- Python bindings, but result analysis is not working yet
- Asynchronous API for GUI and Python bindings
- Time analysis (statistics and execution graph)
