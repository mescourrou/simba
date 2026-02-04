# Robots Configuration

!!! tip "Quick Navigation"
    - ← [Back to Configuration Reference](../config_reference.md)
    - See also: [Physics Configuration](physics.md), [Sensor Configuration](sensors.md)

The `robots` section defines all robots in the simulation, their kinematics models, sensors, controllers, and initial conditions.

## Robot Structure

Each robot in the `robots` list contains:

```yaml
robots:
  - name: robot1                     # Unique robot identifier
    navigator: { ... }               # Navigation component
    controller: { ... }              # Control component
    physics: { ... }                 # Physics simulation
    state_estimator: { ... }         # State estimation
    sensor_manager: { ... }          # Sensors
    network: { ... }                 # Communication
    autospawn: true                  # Auto-start this robot
```

## Robot Models

SiMBA supports two kinematic models for robots:

### Unicycle (Differential Drive)

Two-wheeled differential drive robot. Standard ground robot model.

```yaml
physics:
  type: Internal
  model:
    type: Unicycle
    wheel_distance: 0.25             # Distance between wheels (meters)
  initial_state:
    pose: [0.0, 0.0, 0.0]          # [x, y, theta]
    velocity: [0.0, 0.0]            # [v_x, v_theta]
```

**Characteristics**:
- Non-holonomic (cannot move sideways)
- 2 input velocities: forward and angular
- `wheel_distance`: Physical distance between drive wheels

**Use cases**:
- Ground robots with differential drive
- Wheeled mobile robots
- Most standard ground platforms

### Holonomic

Omnidirectional robot that can move in any direction.

```yaml
physics:
  type: Internal
  model:
    type: Holonomic
  initial_state:
    pose: [0.0, 0.0, 0.0]          # [x, y, theta]
    velocity: [0.0, 0.0, 0.0]      # [v_x, v_y, v_theta]
```

**Characteristics**:
- Holonomic (can move in any direction)
- 3 input velocities: forward, lateral, and angular
- No wheel parameters needed

**Use cases**:
- Omnidirectional mobile bases
- Aerial vehicles (warning: no altitude and attitude dynamics)
- Robots with mecanum wheels


## Complete Robot Configuration Example

```yaml
max_time: 30.0

robots:
  - name: robot1
    navigator:
      type: TrajectoryFollower
      trajectory_path: paths/robot1.yaml
      forward_distance: 0.2
      target_speed: 0.5
      stop_distance: 0.2
      stop_ramp_coefficient: 0.5
    controller:
      type: PID
      proportional_gains: [1.0, 1.0]
      derivative_gains: [0.0, 0.5]
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
      type: Perfect
      prediction_period: 0.1
      targets: [self, robot2]
    sensor_manager:
      sensors:
        - name: RobotDetector
          config:
            type: RobotSensor
            detection_distance: 20.0
            period: 0.1
            faults: []
    network:
      range: 25.0
      reception_delay: 0.01
    autospawn: true
```

---

## See Also

- [Configuration Reference](../config_reference.md#robots-configuration) - All robot configuration options
- [Physics Configuration](physics.md) - Detailed physics and model setup
- [Sensor Configuration](sensors.md) - Sensor setup for robots
- [Configuration Tips & Tricks](../config_tips.md#robot-tips) - Robot configuration patterns