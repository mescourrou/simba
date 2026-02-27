# Configuration Components

This section contains detailed documentation for each configurable component of SiMBA robots and computation units.

## Robot Components

- [Navigation](navigation.md) - How robots choose their path and goals
- [Controller](controller.md) - How robots generate motor commands
- [Physics](physics.md) - How robots move and interact with the environment
- [Sensors](sensors.md) - What robots can perceive
- [State Estimator](state_estimator.md) - How robots estimate their state

## Network & Coordination

- [Network](network.md) - Communication between robots

## Simulator-level Configuration
- [`log`](20_log.md): Defines the logging behavior.
- [`results`](30_results.md): Defines how results are computed, after the simulation.
- `max_time`: Simulation end time, in seconds. Must be positive.
- [`time_analysis`](40_time_analysis.md): Defines the generation of time performance data.
- `random_seed`: Optional seed to have a deterministic behavior. If no seed is given, a random one is chosen.
- [`environment`](environment.md): Defines the environment in which the robots evolve. For now, only landmarks are supported.

## Advanced

- [Computation Units](computation_units.md) - Centralized algorithms
- [Scenario](scenario.md) - Dynamic events during simulation
