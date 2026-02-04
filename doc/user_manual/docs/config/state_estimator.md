# State Estimator Configuration

!!! tip "Quick Navigation"
    - ← [Back to Configuration Reference](../config_reference.md)
    - See also: [Sensor Configuration](sensors.md), [Computation Units](computation_units.md)

The `state_estimator` component processes sensor data to estimate the robot's state (position, orientation, velocity) and optionally other robots' states and landmarks.

## `state_estimator` Configuration

**Type**: `StateEstimatorConfig` (Enum)  
**Required**: Yes

Determines how the robot estimates its state based on sensor inputs.

## State Estimator Type: `Perfect`

The robot knows its exact state from the simulator. Useful for developing and testing algorithms without estimation uncertainty.

```yaml
state_estimator:
  type: Perfect
  prediction_period: 0.1             # Update period (seconds)
  targets:                           # What to estimate
    - self                           # Own state
  map_path: null                     # Optional landmark map
```

**Parameters**:
- `prediction_period`: How often state estimates are updated (e.g., `0.1` = 10 Hz)
- `targets`: Which states to estimate, including:
  - `self`: The robot's own state
  - Other robot names: e.g., `robot1`, `robot2` to get their states perfectly
- `map_path`: Optional path to landmark map for landmark-aware estimation

**Use cases**:
- Algorithm development (removing estimation noise)
- Testing navigation without localization errors
- Benchmark comparisons
- Debugging controller issues

**When to use**:
- You're developing pure navigation/control algorithms
- You want to isolate algorithm performance from sensor noise
- Testing in simulation before adding realistic sensors
- You want a perfect control loop to evaluate state estimation algorithms with the `bench` setup (`state_estimator_bench` in robots).

## State Estimator Type: `Python`

Implement custom state estimation in Python.

```yaml
state_estimator:
  type: Python
  file: my_estimator.py
  class_name: MyStateEstimator
  prediction_period: 0.1
  # Additional parameters
  filter_type: kalman                # Your custom parameters
  noise_level: 0.1
```

For Python integration details, see the [Python Integration Guide](../python/using_python.md).

## State Estimator Type: `External`

Implement state estimation via external plugin. Use for maximum performance or proprietary algorithms.

```yaml
state_estimator:
  type: External
  config:
    # Configuration for your plugin
    prediction_period: 0.1
```

The `config` dict is passed to your plugin. Consult your plugin documentation for required fields.

### Perfect Estimator with Landmark Map

Use landmark map to provide landmark identity information:

```yaml
state_estimator:
  type: Perfect
  prediction_period: 0.1
  map_path: maps/landmarks.yaml      # Map for landmark observation linking
```

## State Estimation in the Simulation Loop

The state estimator is a critical component in the simulation. It processes sensor measurements to estimate the robot's state (position, orientation, velocity). The estimated state is then used by the navigator and controller to make decisions.

The `prediction_period` controls how often the state estimator updates its estimates. It also triggers the control loop.

## Common Patterns

### Simple simulation (no estimation noise)
```yaml
state_estimator:
  type: Perfect
  prediction_period: 0.1
```

### Realistic single-robot SLAM
```yaml
state_estimator:
  type: Python
  file: ekf_slam.py
  class_name: EKF_SLAM
  prediction_period: 0.1
  landmarks_max: 50
  innovation_threshold: 5.0
```

### Multi-robot centralized estimation
```yaml
state_estimator:
  type: Perfect
  prediction_period: 0.1
  targets:
    - self
    - robot1
    - robot2
  map_path: maps/landmarks.yaml
```

### Decentralized with custom filter
```yaml
state_estimator:
  type: Python
  file: particle_filter.py
  class_name: ParticleFilter
  prediction_period: 0.05  # 20 Hz
  num_particles: 1000
  resampling_threshold: 0.5
```

## Integration with Sensors

**Important**: Your state estimator receives sensor data based on:

1. **Robot's own sensors**: Always available (directly from `sensor_manager`)
2. **Other robots' sensors**: Only if they sent it via `send_to` (requires network communication)

Example: If robot1 has a landmark sensor and sends it to robot2 via `send_to: [robot2]`:
```yaml
# robot1 config
sensor_manager:
  sensors:
    - name: landmark_detector
      send_to: [robot2]              # Send to robot2
      config:
        type: OrientedLandmarkSensor
        map_path: maps/landmarks.yaml

# robot2 config
state_estimator:
  type: Python
  file: cooperative_slam.py
  class_name: CooperativeSLAM
```

---

## See Also

- [Sensor Configuration](sensors.md) - Data source for state estimators
- [Navigation Configuration](navigation.md) - Uses state estimates
- [Computation Units](computation_units.md) - Centralized state estimation
- [Configuration Tips & Tricks](../config_tips.md#state-estimator-tips) - Estimation setup examples
- [Python Integration Guide](../python/using_python.md) - Implementing custom estimators
