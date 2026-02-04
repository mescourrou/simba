# Configuration Reference Overview

!!! tip "Navigation"
    - New user? Start with → [Getting Started Guide](config_getting_started.md)
    - **You are here:** Configuration topics 👈
    - Need best practices? → [Tips & Tricks](config_tips.md)
    - Quick field lookup? → [Auto-generated Docs](config_documentation.md)

Welcome to the comprehensive SiMBA configuration reference. This page organizes configuration topics by component. Each topic has a dedicated page with detailed explanations, examples, and troubleshooting.

## How SiMBA Works

SiMBA is a time-stepped simulator. Each robot/computation unit has multiple components that work together: navigation, control, physics simulation, state estimation, and sensors.

Your configuration file controls each of these components for every robot and computation unit.

## Configuration Topics

### Core Components

**Each robot has configuration for:**

- [Navigation Configuration](config/navigation.md) - GoTo, TrajectoryFollower, Python, External
  - How robots choose targets and plan paths

- [Controller Configuration](config/controller.md) - PID, Python, External
  - How robots convert targets into motor commands

- [Physics Configuration](config/physics.md) - Internal, Python, External
  - How robots move (unicycle, holonomic, bicycle)

- [State Estimator Configuration](config/state_estimator.md) - Perfect, Python, External
  - How robots estimate their state from sensors

- [Sensor Manager Configuration](config/sensors.md) - RobotSensor, OrientedLandmarkSensor, SpeedSensor, etc.
  - What sensors the robot has and where data is sent

- [Network Configuration](config/network.md) - Communication ranges and delays
  - How robots communicate with each other

Without configuration specified, SiMBA uses default settings for each component.

### Advanced Topics

- [Computation Units Configuration](config/computation_units.md) - Non-physical nodes
  - Global state estimation, coordination, planning
  - No physical presence, only computation

- [Scenario Configuration](config/scenario.md) - Dynamic events
  - Robot spawning, collision events, area triggers

### Top-Level Settings

The root of your configuration file includes:

- **`version`**: Configuration format version
- **`max_time`**: Total simulation duration in seconds
- **`log`**: Logging configuration
- **`results`**: Result saving configuration
- **`time_analysis`**: Performance analysis settings
- **`random_seed`**: Reproducible randomness
- **`robots`**: List of robots to simulate
- **`computation_units`**: Centralized computing nodes
- **`scenario`**: Dynamic events

## Configuration File Structure

```yaml
version: 1.4.4
max_time: 100.0
log:
  log_level: Info
results:
  result_path: simulation_results
  save_mode: AtTheEnd
random_seed: 42
robots:
  - name: robot1
    navigator: { ... }        # See Navigation Configuration
    controller: { ... }       # See Controller Configuration
    physics: { ... }          # See Physics Configuration
    state_estimator: { ... }  # See State Estimator Configuration
    sensor_manager: { ... }   # See Sensor Manager Configuration
    network: { ... }          # See Network Configuration
computation_units: [ ... ]    # See Computation Units
scenario: { ... }             # See Scenario Configuration
```

## Getting Started

1. **For a quick introduction**: Start with [Getting Started Guide](config_getting_started.md)
2. **For your first robot**: Follow [Navigation](config/navigation.md) → [Controller](config/controller.md) → [Physics](config/physics.md) → [Sensors](config/sensors.md)
3. **For multi-robot systems**: Add [Network](config/network.md) and [Computation Units](config/computation_units.md)
4. **For dynamic simulations**: See [Scenario](config/scenario.md)

## Common Setups by Use Case

See [Getting Started Guide](config_getting_started.md) for a minimal working example with complete YAML structure.

For each use case, refer to the specific component documentation:
- **Single robot with path**: [Navigation](config/navigation.md) (TrajectoryFollower) + [Controller](config/controller.md)
- **Multi-robot coordination**: Add [Network](config/network.md) and configure [Sensor Manager](config/sensors.md) with `send_to`
- **Centralized control**: See [Computation Units](config/computation_units.md)
- **Dynamic scenarios**: See [Scenario](config/scenario.md) for event configuration

## Finding What You Need

**Know what component?**
- [Navigation](config/navigation.md) - Computes the error of the robot to its target
- [Controller](config/controller.md) - Computes the command to send to the robot actuators to minimize the error
- [Physics](config/physics.md) - How robots move
- [Sensors](config/sensors.md) - Robot measurements, intrinsic and extrinsic, and with faults models (noise, mis-detection...)
- [State Estimator](config/state_estimator.md) - State estimation of the robot pose and environment
- [Network](config/network.md) - Inter-robot communication
- [Computation Units](config/computation_units.md) - Non-physical computing nodes, e.g., centralized planner
- [Scenario](config/scenario.md) - Dynamic events

**Looking for examples?**
- See [Tips & Tricks](config_tips.md) for patterns and best practices
- See example configurations in the `config_example/` directory in the repository for complete working examples

**Need exact field names?**
- See [Auto-generated Reference](config_documentation.md) for complete field listing with Rust API links

## Tips for Configuration

1. **Start Simple**: Use perfect sensors and estimators, build up complexity
2. **One Robot First**: Get a single robot working before multi-robot setups
3. **Use Examples**: Reference the `config_example/` folder in the repository or `examples/` directory for specific scenarios cases
4. **Check Syntax**: Use a YAML validator before running simulation (`config.schema.json`)
5. **Review Logs**: Check simulation logs for configuration errors

## Advanced Topics

- [Python Integration Guide](python/using_python.md) - Custom Python components

## Related Documentation

- **[Rust API Documentation](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/)** - Complete technical API reference
- **[Getting Started](config_getting_started.md)** - Beginner-friendly introduction
- **[Tips & Tricks](config_tips.md)** - Best practices and troubleshooting
- **Example Configurations** - Working real-world examples in the `config_example/` directory
