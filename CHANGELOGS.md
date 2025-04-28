# Changelogs

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