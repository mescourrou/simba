# Configuration Tips & Tricks

!!! tip "Navigation"
    - New user? Start with → [Getting Started Guide](config_getting_started.md)
    - Want all details? → [Complete Configuration Reference](config_reference.md)
    - **You are here:** Tips & Tricks 👈
    - Quick field lookup? → [Auto-generated Docs](config_documentation.md)

This guide provides best practices, common patterns, and troubleshooting advice for SiMBA configuration.

## Table of Contents
- [Common Patterns](#common-patterns)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)
- [Performance Optimization](#performance-optimization)
- [Advanced Techniques](#advanced-techniques)

---

## Common Patterns

### Pattern 1: Multi-Robot Coordination Scenario

Two robots that coordinate through communication:

```yaml
version: 1.4.5
max_time: 30.0

robots:
  - name: robot1
    navigator:
      type: TrajectoryFollower
      trajectory_path: paths/robot1_path.yaml
      forward_distance: 0.2
      target_speed: 0.5
      stop_distance: 0.2
      stop_ramp_coefficient: 0.5
    controller:
      type: PID
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
      faults: []
    state_estimator:
      type: External
      config: ...                       # Your external plugin configuration
    sensor_manager:
      sensors:
        - name: RobotDetector
          send_to: [robot2]             # Share detections with robot2
          config:
            type: RobotSensor
            detection_distance: 20.0    # Can see 20m away
            period: 0.1
            faults: []
    network:
      range: 25.0                       # Can communicate up to 25m
      reception_delay: 0.01             # 10ms network delay
    state_estimator_bench: []

  - name: robot2
    navigator:
      type: TrajectoryFollower
      trajectory_path: paths/robot2_path.yaml
      forward_distance: 0.2
      target_speed: 0.5
      stop_distance: 0.2
      stop_ramp_coefficient: 0.5
    controller:
      type: PID
      proportional_gains: [1.0, 1.0]
      derivative_gains: [0.0, 0.1]
      integral_gains: [0.0, 0.0]
    physics:
      type: Internal
      model:
        type: Unicycle
        wheel_distance: 0.25
      initial_state:
        pose: [10.0, 0.0, 0.0]
        velocity: [0.0, 0.0]
      faults: []
    state_estimator:
      type: External
      config: ...                       # Your external plugin configuration
    sensor_manager:
      sensors:
        - name: RobotDetector
          send_to: [robot1]             # Share detections with robot1
          config:
            type: RobotSensor
            detection_distance: 20.0
            period: 0.1
            faults: []
    network:
      range: 25.0
      reception_delay: 0.01
```

### Pattern 2: Sensor and Physics Noise Configuration

Configure realistic sensor noise and faults at startup:

```yaml
version: 1.4.5
max_time: 20.0

robots:
  - name: robot1
    navigator:
      type: TrajectoryFollower
      trajectory_path: paths/robot_path.yaml
    controller:
      type: PID
    physics:
      type: Internal
      model:
        type: Unicycle
        wheel_distance: 0.25
      initial_state:
        pose: [0.0, 0.0, 0.0]
        velocity: [0.0, 0.0]
      # Slippage noise proportional to velocity
      faults:
        - type: AdditiveRobotCentered
          distributions:
            - type: Normal
              mean: [0.0, 0.0]
              covariance: [0.01, 0.0, 0.0, 0.01]
              variable_order: [x, y]
          proportionnal_to_velocity: 1.0
    state_estimator:
      type: Perfect
      prediction_period: 0.1
    sensor_manager:
      sensors:
        # GPS with occasional errors
        - name: GPS
          config:
            type: GNSSSensor
            period: 0.5                     # Low update rate
            faults:
              - type: AdditiveRobotCentered
                apparition:
                  probability: [1.0]        # Always present (default)
                distributions:
                  - type: Normal
                    mean: [0.0, 0.0]
                    covariance: [0.1, 0.0, 0.0, 0.1]
                    variable_order: [x, y]
              - type: Misdetection           # Sometimes missing
                apparition:
                  probability: [0.1]        # 10% chance
        # Landmark sensor with missed detections
        - name: Landmarks
          config:
            type: OrientedLandmarkSensor
            detection_distance: 15.0
            map_path: maps/landmarks.yaml
            period: 0.1
            faults:
              - type: Misdetection
                apparition:
                  probability: [0.05]      # 5% miss rate
              - type: AdditiveRobotCenteredPolar
                distributions:
                  - type: Normal
                    mean: [0.0, 0.0]
                    covariance: [0.05, 0.0, 0.0, 0.01]
                    variable_order: [r, theta]
```

### Pattern 3: Central Processing Unit

One computation unit receives sensor data from all robots and performs centralized state estimation:

```yaml
version: 1.4.5
max_time: 20.0

robots:
  - name: robot1
    sensor_manager:
      sensors:
        - name: landmark_sensor
          send_to: [Central Processing]  # Send to central unit
          config:
            type: OrientedLandmarkSensor
            detection_distance: 15.0
            map_path: maps/landmarks.yaml
            period: 0.1
        - name: robot_detector
          send_to: [Central Processing]  # Share robot detections
          config:
            type: RobotSensor
            detection_distance: 10.0
            period: 0.1
    network:
      range: 100.0                       # Large communication range
      reception_delay: 0.01
  
  - name: robot2
    # Similar configuration to robot1
    sensor_manager:
      sensors:
        - name: landmark_sensor
          send_to: [Central Processing]  # Send to central unit
          config:
            type: OrientedLandmarkSensor
            detection_distance: 15.0
            map_path: maps/landmarks.yaml
            period: 0.1
        - name: robot_detector
          send_to: [Central Processing]  # Share robot detections
          config:
            type: RobotSensor
            detection_distance: 10.0
            period: 0.1
    network:
      range: 100.0
      reception_delay: 0.01

computation_units:
  - name: Central Processing
    state_estimators:
      - name: global_slam
        config:
          type: External
          config:
            # Your external plugin configuration
```

**Key features of this pattern:**
1. Each robot sends its sensor data to the central unit via `send_to: [Central Processing]`
2. The central unit runs a state estimator that processes all sensor data
3. Network delay is not required and can be set to zero for instant communication

**When to use:**
- Centralized SLAM or state estimation
- Distributed robots relying on a central controller
- Multi-robot coordination with global awareness

---

## Best Practices

### 1. Version Management
Specify the correct version matching your simulator to get a warning when using different major versions:

```yaml
version: 1.4.5  # Match your installed SiMBA version
```

Check with: `simba-cmd --version`

### 2. File Path Organization

Use relative paths and organize consistently:

```
config/
├── my_simulation.yaml
├── paths/
│   ├── robot1_path.yaml
│   └── robot2_path.yaml
├── maps/
│   ├── landmarks.yaml
│   └── obstacles.yaml
└── results.json
```

In configuration:
```yaml
navigator:
  trajectory_path: paths/robot1_path.yaml  # Relative to config file
state_estimator:
  map_path: maps/landmarks.yaml
```

### 3. Use Schema Validation

Enable auto-completion in your editor:

```yaml
# yaml-language-server: $schema=../config.schema.json
version: 1.4.5
```

This provides:
- Auto-completion for field names
- Type checking
- Detection of unknown fields

### 4. Realistic Sensor Configuration

Match sensor properties to real-world devices:

```yaml
# High-precision GPS (1Hz, 0.1m accuracy)
- name: GPS_HighRes
  config:
    type: GNSSSensor
    period: 1.0
    faults:
      - type: AdditiveRobotCentered
        distributions:
          - type: Normal
            mean: [0.0, 0.0]
            covariance: [0.01, 0.0, 0.0, 0.01]

# Low-cost GPS (10Hz, 1m accuracy)
- name: GPS_LowCost
  config:
    type: GNSSSensor
    period: 0.1
    faults:
      - type: AdditiveRobotCentered
        distributions:
          - type: Normal
            mean: [0.0, 0.0]
            covariance: [1.0, 0.0, 0.0, 1.0]
```

### 5. Deterministic vs. Random Experiments

For **reproducible** results, use a fixed seed:

```yaml
random_seed: 42  # Always produces same behavior
```

For **statistical testing**, run multiple trials with `null`:

```yaml
random_seed: null  # Different each time
```

### 6. Performance Monitoring

Add time analysis to track performance:

```yaml
time_analysis:
  exporter:
    type: TraceEventExporter
  keep_last: true
  output_path: time_performance
  analysis_unit: ms                   # Milliseconds for precision
```

Then analyze results in Python:
```python
import json
with open('time_performance.json') as f:
    timing = json.load(f)
```

### 7. Logging for Debugging

Start with minimal logging, then enable more as needed:

```yaml
# Minimal logging (production)
log:
  log_level:
    type: Warn

# Debug logging (for your Rust custom code)
# Note: Debug logs from simulator only appear if log_level is Internal
log:
  log_level:
    type: Debug

# Internal debugging (simulator internals)
# Shows detailed simulator operation, network messages, setup steps
log:
  log_level:
    type: Internal
    options:
      - SensorManager
      - NavigatorDetailed
```

**Important**: 
- `Debug` level: For your own Rust custom code logging (via `log::debug!()`)
- `Internal` level: For SiMBA's internal simulator logs (network messages, component lifecycle, etc.)

The simulator does NOT emit Debug-level logs unless you explicitly use `Internal` logging option.

### 8. Gain Tuning

Start with moderate PID gains, adjust based on performance:

```yaml
controller:
  type: PID
  # Start conservative
  proportional_gains: [0.5, 0.5, 0.5]
  derivative_gains: [0.25, 0.25, 0.25]
  integral_gains: [0.05, 0.05, 0.05]
```

Then tune:
- Increase P if response is too slow
- Increase D if oscillating
- Increase I if steady-state error

The number of gain values depends on robot model:
- Unicycle: 2 values (longitudinal velocity and angular velocity)
- Holonomic: 3 values (longitudinal, lateral, angular velocities)

---

## Troubleshooting

### Issue: "Unknown field in robots"

**Cause**: Typo in field name (YAML is case-sensitive)

**Solution**: 
- Check exact field name in reference
- Enable schema validation
- Compare with working examples

```yaml
# ❌ Wrong
navigator:
  type: trajectory_follower  # Wrong case

# ✅ Correct
navigator:
  type: TrajectoryFollower
```

### Issue: "Invalid enum value"

**Cause**: Using invalid option for `type` field

**Solution**: Check allowed values in [Configuration Reference](config_reference.md)

```yaml
# ❌ Wrong
navigator:
  type: RandomWalk  # Not supported

# ✅ Correct
navigator:
  type: GoTo  # or TrajectoryFollower, Python, External
```

### Issue: "File not found" for trajectory/map

**Cause**: Incorrect relative path or file doesn't exist

**Solution**:
- Use relative paths from config file location
- Verify file exists
- Check spelling

```yaml
# If config is in config/my_sim.yaml
navigator:
  trajectory_path: paths/path.yaml  # Looks for config/paths/path.yaml
  
# NOT
navigator:
  trajectory_path: /absolute/path/path.yaml  # Avoid absolute paths
```

### Issue: Robot doesn't move

**Possible causes**:
1. `max_time` is too short
2. PID gains are too low
3. Target is at robot's starting position or no target set (default for GoTo)

**Debug steps**:
```yaml
# 1. Increase max_time
max_time: 100.0

# 2. Increase gains
controller:
  proportional_gains: [2.0, 2.0, 2.0]

# 3. Check command and error in result output:
results:
  result_path: results.json
```

### Issue: Sensors not detecting anything

**Possible causes**:
1. Detection distance too small
2. Sensor `period` too large (updates too slowly)
3. Target outside detection range

**Solution**:
```yaml
sensor_manager:
  sensors:
    - name: RobotDetector
      config:
        type: RobotSensor
        detection_distance: 50.0  # Increase range
        period: 0.05              # Increase update rate
```

### Issue: Simulation runs very slowly

**Solutions**:

1. **Reduce sensor update rates**:
```yaml
period: 0.5  # 2 Hz instead of 10 Hz
```

2. **Disable time analysis** (set to `null` instead of removing):
```yaml
time_analysis: null  # Disable performance analysis
```

3. **Use faster state estimators**:
```yaml
state_estimator:
  type: Perfect
  prediction_period: 0.5  # Slower updates
```

4. **Reduce logging**:
```yaml
log:
  log_level:
    type: Info  # Not Debug
```

5. **Check time analysis output** for time taken by each component to identify bottlenecks. You can check directly the `time_performance.csv` file containing time statistics:
```yaml
time_analysis:
  output_path: time_performance
  analysis_unit: ms
```

### Issue: Unrealistic physics behavior

**Check** robot model parameters:

```yaml
# Unicycle: wheel_distance should match real robot
physics:
  model:
    type: Unicycle
      wheel_distance: 0.25  # Typical for TurtleBot: 0.16-0.4m

# Holonomic: max velocities should be realistic
physics:
  model:
    type: Holonomic
    max_longitudinal_velocity: 1.0   # 1 m/s is reasonable
    max_lateral_velocity: 1.0
    max_angular_velocity: 3.14       # ~180°/s
```

---

## Performance Optimization

### For Detailed Analysis

```yaml
version: 1.4.5
max_time: 10.0  # Shorter simulation

log:
  log_level:
    type: Debug  # Detailed logs

time_analysis:
  keep_last: true
  output_path: timing_analysis
  analysis_unit: ms

results:
  show_figures: true
  analyse_script: post_analysis.py
```

### For Large Fleet Simulations

```yaml
robots:
  # All robots use same base config with minimal sensing
  - name: robot_1
    sensor_manager:
      sensors: []  # No sensors = faster
    network:
      range: 0.0   # No communication = faster
```

---

## Advanced Techniques

### Using Python Components

For custom algorithms not built-in, use Python:

```yaml
navigator:
  type: Python
  file: my_navigator.py
  class_name: CustomNavigator

controller:
  type: Python
  file: my_controller.py
  class_name: CustomController

physics:
  type: Python
  file: my_physics.py
  class_name: CustomPhysics

state_estimator:
  type: Python
  file: my_estimator.py
  class_name: CustomEstimator
```

### Scenario-Based Testing

Create complex test scenarios:

```yaml
scenario:
  events:
    # Robot 2 appears after 5 seconds
    - trigger:
        type: Time
        time: 5.0
      event_type:
        type: Spawn
        model_name: robot2
        node_name: robot2
    
    # Robot 1 killed if it gets too close to robot 2
    - trigger:
        type: Proximity
        distance: 1.0
        inside: true
      event_type:
        type: Kill
        target: robot1
```

### Multiple Test Runs

Create configuration templates for parameter sweeps:

```bash
# Generate configs with different parameters
for speed in 0.3 0.5 1.0; do
  sed "s/TARGET_SPEED/$speed/g" template.yaml > config_speed_${speed}.yaml
done
```

In template.yaml:
```yaml
navigator:
  type: GoTo
  target_point: [10.0, 0.0]
  target_speed: TARGET_SPEED  # Replaced by script
```

### Result Analysis Scripts
Use custom Python scripts for post-simulation analysis:

```yaml
results:
  result_path: simulation_results
  analyse_script: analyze_results.py  # Your analysis script
```

The analysis script should contain a function with signature:
```python
def analyse(records: list, config: dict, figure_path: str, figure_type: str, additionnal_param: dict|None) -> None:
  # Your analysis code here
```

Where:
- `records`: Simulation results data, with the same structure as the `records` list of `results.json`
- `config`: The configuration dictionary used for the simulation
- `figure_path`: Path to save generated figures (from `results.result_path` config)
- `figure_type`: File type for figures (e.g., "png", "pdf"), should be supported by matplotlib
- `python_params`: Any additional parameters you may want to pass (can be `None`). These can be set in the `results` configuration as:
```yaml
results:
  analyse_script: analyze_results.py
  python_params:
    param1: value1
    param2: value2
```

See `analyse_results.py` in the `python_scripts/` folder for an example implementation.

---

## More Information

- **[Getting Started Guide](config_getting_started.md)**: Quick introduction to configuration
- **[Complete Reference](config_reference.md)**: Detailed documentation of all options
- **[Auto-generated Documentation](config_documentation.md)**: Field-level details
- **[Rust API Docs](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/)**: Technical implementation details
