# Getting Started with Configuration

!!! tip "Navigation"
    - **New user?** You're in the right place! 👈
    - Want all details? → [Complete Configuration Reference](config_reference.md)
    - Need help troubleshooting? → [Tips & Tricks](config_tips.md)
    - Looking for specific field? → [Auto-generated Docs](config_documentation.md)

This guide will help you create your first configuration file for SiMBA. We'll start with a minimal working example and gradually explore what you can configure.

## What is a Configuration File?

A SiMBA configuration file is a YAML file that describes:
- **Simulation parameters**: simulation duration, random seed, logging
- **Robots**: their navigation strategy, control method, physics model, and sensors
- **Computation units**: non-physical nodes that can perform state estimation based on message reception
- **Scenario events**: dynamic events during simulation (robot spawning, killing)
- **Results**: how and where to save simulation outputs

## Minimal Working Example

Here's the simplest configuration that will run a simulation with one robot:

```yaml
max_time: 10.0
robots:
  - name: robot1
```

After default application, it will result in:

```yaml
version: 1.6.0
max_time: 10.0
log:
  log_level: 
    type: Info
results: null
time_analysis:
  exporter:
    type: TraceEventExporter
  output_path: time_performance # .json or .csv will be appended
  analysis_unit: s
random_seed: null # Different seed each run
environment:
  map_path: null
robots:
  - name: robot1
    navigator:
      type: GoTo
      target_point: null
      target_speed: 0.5
      stop_distance: 0.2
      stop_ramp_coefficient: 0.5
    controller:
      type: PID
      robot_model:
        type: Unicycle
        wheel_distance: 0.25
      proportional_gains: [1.0, 1.0]
      derivative_gains: [0.0, 0.1]
      integral_gains: [0.0, 0.0]
    physics:
      type: Internal
      model:
        type: Unicycle
        wheel_distance: 0.25
      initial_state:
        pose: [0.0, 0.0, 0.0]
        velocity: [0.0, 0.0]
    state_estimator:
      type: Perfect
      prediction_activation:
        perdiod: {type: Num, value: 0.1}
      targets:
        - self
    sensor_manager:
      sensors: []
    network:
      range: 0.0 # unlimited range
      reception_delay: 0.0
computation_units: []
scenario:
  events: []
```

### What This Means

| Section | Purpose |
|---------|---------|
| `version` | Configuration file version (must match simulator version) |
| `max_time` | Simulation runs for 10 seconds |
| `log` | Logging is enabled at Info level |
| `results` | No results will be saved |
| `time_analysis` | Time performance analysis is enabled, saved as `time_performance.json` and as a statistic table `time_performance.csv` |
| `random_seed` | Random seed is not set, so different each run |
| `robots` | List of robots in the simulation |
| `navigator` | How the robot chooses where to go (GoTo: simple point-to-point) |
| `controller` | How the robot converts navigation commands to motor commands (PID controller) |
| `physics` | Physics simulation (Internal: built-in model, Unicycle: 2D ground robot) |
| `initial_state` | Robot's starting position and velocity |
| `state_estimator` | How the robot estimates its state (Perfect: knows exactly where it is) |
| `sensor_manager` | Robot's sensors (empty list = no sensors yet) |
| `network` | Communication capabilities (0.0 range = no communication) |

## Adding More Robots

To add a second robot, simply add another entry to the `robots` list:

```yaml
robots:
  - name: robot1
    # ... robot1 configuration ...
  - name: robot2
    # ... robot2 configuration ...
```

Each robot can have different:
- Navigation strategies
- Controllers
- Physics models
- Sensors
- Initial states

## Adding Sensors

Let's add a simple sensor to detect other robots. Replace the `sensor_manager` section:

```yaml
sensor_manager:
  sensors:
    - name: RobotDetector
      send_to: [other_robot]
      config:
        type: RobotSensor
        detection_distance: 10.0  # Can detect robots within 10 meters
        activation_time: # Sensor updates every 0.1 seconds
          period: {type: Num, value: 0.1}
        faults: []   # No sensor faults
```

## Understanding Enums (Type Selection)

Many configuration fields use a `type` field to select from options. For example:

```yaml
navigator:
  type: GoTo  # Choose between: GoTo, TrajectoryFollower, External, Python
```

When you select a type, additional fields specific to that type become available:

```yaml
navigator:
  type: TrajectoryFollower
  trajectory_path: paths/path1.yaml  # Additional field specific to this type
  forward_distance: 0.2
  target_speed: 0.5
  stop_distance: 0.2
  stop_ramp_coefficient: 0.5
```

## Common Robot Models

### Unicycle (Differential Drive)
Simulates a wheeled robot with two motors (like a TurtleBot):

```yaml
physics:
  model:
    type: Unicycle
    wheel_distance: 0.25  # Distance between wheels (meters)
```

It takes a velocity command of `[linear_velocity, angular_velocity]`.

### Holonomic
Simulates a robot that can move in any direction independently (like a drone or omnidirectional robot):

```yaml
physics:
  model:
    type: Holonomic
    max_longitudinal_velocity: 1.0
    max_lateral_velocity: 1.0
    max_angular_velocity: 3.14
```

It takes a velocity command of `[longitudinal_velocity, lateral_velocity, angular_velocity]`.

## Units and Coordinate System

- **Distance**: meters or anything consistent (e.g., kilometers, light-years, etc.)
- **Time**: seconds
- **Speed**: _distance_ per second
- **Angle**: radians
- **Pose**: `[x, y, theta]` where `x` and `y` are in _distance_ and `theta` is in radians
- **Coordinate system**: Cartesian, standard XY plane

## Next Steps

- Read the **[Complete Configuration Reference](config_reference.md)** for detailed documentation of all configuration options
- Check **[Configuration Tips & Tricks](config_tips.md)** for common patterns and best practices
- Explore example configurations in `config_example/` directory
- Review the [auto-generated documentation](config_documentation.md) for detailed field information

## Tips for Writing Your Configuration

1. **Start small**: Begin with the minimal example and add features gradually
2. **Use schema validation**: Add this line to your YAML file for auto-completion:
```yaml
# yaml-language-server: $schema=path/to/config.schema.json
```
The json schema file is located in available with each release and at the root of the repository under `config.schema.json`.
3. **Check the examples**: The `config_example/` folder contains working configurations with various features
4. **Enable logging**: Add logging configuration to debug issues:
```yaml
log:
  log_level:
    type: Debug
```
5. **Validate paths**: Ensure all file paths (trajectories, maps, scripts) exist relative to your config file location

## Common Errors and Solutions

| Error | Solution |
|-------|----------|
| `Unknown field in robots` | Check field names match exactly (case-sensitive) |
| `Invalid enum value` | Make sure `type` value is one of the allowed options |
| `File not found` | Verify path is relative to config file location |

For more troubleshooting, see [Configuration Tips & Tricks](config_tips.md#troubleshooting).
