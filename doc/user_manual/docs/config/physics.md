# Physics Configuration

!!! tip "Quick Navigation"
    - ← [Back to Configuration Reference](../config_reference.md)
    - See also: [Controller Configuration](controller.md), [Sensor Configuration](sensors.md)

The `physics` component defines how the robot moves and interacts with the world. Choose from built-in physics engines or implement your own.

## `physics` Configuration

**Type**: `PhysicsConfig` (Enum)  
**Required**: Yes

Defines how the robot moves and interacts with the world.

## Physics Type: `Internal`

Use SiMBA's built-in kinematic/dynamic physics engine. This is the standard choice for most simulations.

```yaml
physics:
  type: Internal
  model:                             # Robot kinematic model
    type: Unicycle
    wheel_distance: 0.25
  initial_state:                     # Starting position and velocity
    pose: [0.0, 0.0, 0.0]           # [x, y, theta] in meters and radians
    velocity: [0.0, 0.0]            # [v_x, v_y] in m/s
    random: []                       # Random initialization
  faults: []                         # Actuator faults
```

**Parameters**:
- `model`: Robot kinematic constraints (see Robot Models below)
- `initial_state`: Starting position, orientation, and velocity
- `faults`: Simulated actuator failures/noise

### Robot Models in Physics

Define the kinematic/dynamic constraints of your robot.

#### Unicycle Model

For differential-drive robots (skid-steer vehicles, car-like robots):

```yaml
model:
  type: Unicycle
  wheel_distance: 0.25  # Distance between wheels (meters)
```

**Characteristics**:
- Two independently controlled wheels
- Non-holonomic: cannot move sideways
- Examples: TurtleBot, differential-drive robots

**When to use**:
- Wheeled mobile robots
- Car-like vehicles
- Most terrestrial robots

#### Holonomic Model

For omnidirectional robots (mecanum wheels, full-body movement):

```yaml
model:
  type: Holonomic
  max_longitudinal_velocity: 1.0  # Max forward speed (m/s)
  max_lateral_velocity: 1.0        # Max strafe speed (m/s)
  max_angular_velocity: 3.14       # Max rotation speed (rad/s)
```

**Characteristics**:
- Can move in any direction
- Instantaneous movement possible
- Examples: Mecanum-wheeled robots, omnidirectional platforms

**When to use**:
- Omnidirectional robots
- Robots with mecanum/Omni wheels
- Simplified ideal robot models

### Initial State Configuration

Define the robot's starting position and velocity:

```yaml
initial_state:
  pose: [1.0, 1.0, 0.0]     # [x (m), y (m), theta (rad)]
  velocity: [0.0, 0.0]      # [v_x (m/s), v_y (m/s)]
  random:                   # Optional: random variations
    - type: Normal
      mean: [0.0, 0.0, 0.0]
      covariance: [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
      variable_order: [x, y, theta]
```

Here, `pose` sets the initial position and orientation, while `velocity` sets the starting speed. The optional `random` field allows adding Gaussian noise to the initial state. The initial position will then be sampled on a normal distribution with a mean of `[1.0, 1.0, 0.0]` and the specified covariance matrix.

### Physics Faults

Simulate actuator failures and noise:

```yaml
faults:
  - type: AdditiveRobotCentered
    distributions:
      - type: Normal
        mean: [0.0, 0.0]
        covariance: [0.01, 0.0, 0.0, 0.01]
        variable_order: [x, y]
    proportionnal_to_velocity: 0.5  # Fault scales with speed
```

**Use cases**:
- Simulating motor noise
- Testing robustness to actuation errors
- Modeling friction/slip

More physics faults will be added in future releases.

## Physics Type: `Python`

Implement custom physics simulation in Python. Use this for custom dynamics, constraints, or physics not covered by built-in models.

```yaml
physics:
  type: Python
  file: my_physics.py
  class_name: MyPhysics
  # Additional parameters
  custom_param: value
```

**How it works**:
1. SiMBA loads `my_physics.py` and instantiates `MyPhysics`
2. Your class receives initial state and config
3. When motor commands are sent, your `apply_command(command, time)` method is called
4. Each time step, `update_state(time)` is called
5. `state(time)` returns current robot state

For Python integration details, see the [Python Integration Guide](../python/using_python.md).

**Typical use cases**:
- More complex dynamics (e.g., aerial robots, underwater vehicles)
- Hardware-in-the-loop simulations

## Physics Type: `External`

Implement physics via external plugin (Rust ou Python). Use for specific physics engines or hardware in the loop.

```yaml
physics:
  type: External
  config: {}  # Configuration for your plugin
```

The `config` dict is passed to your plugin. Consult your plugin documentation for required fields.

## Common Patterns

### Differential-drive robot
```yaml
physics:
  type: Internal
  model:
    type: Unicycle
    wheel_distance: 0.2
  initial_state:
    pose: [0.0, 0.0, 0.0]
    velocity: [0.0, 0.0]
```

### Omnidirectional robot
```yaml
physics:
  type: Internal
  model:
    type: Holonomic
    max_longitudinal_velocity: 1.5
    max_lateral_velocity: 1.5
    max_angular_velocity: 6.28
  initial_state:
    pose: [0.0, 0.0, 0.0]
    velocity: [0.0, 0.0]
```

### Robot with initial uncertainty
```yaml
physics:
  type: Internal
  model:
    type: Unicycle
    wheel_distance: 0.2
  initial_state:
    pose: [5.0, 5.0, 0.0]
    velocity: [0.0, 0.0]
    random:
      - type: Normal
        mean: [0.0, 0.0, 0.0]
        covariance: [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.01]
        variable_order: [x, y, theta]
```

### Robot with motor noise
```yaml
physics:
  type: Internal
  model:
    type: Unicycle
    wheel_distance: 0.2
  initial_state:
    pose: [0.0, 0.0, 0.0]
    velocity: [0.0, 0.0]
  faults:
    - type: AdditiveRobotCentered
      distributions:
        - type: Normal
          mean: [0.0, 0.0]
          covariance: [0.001, 0.0, 0.0, 0.001]
          variable_order: [x, y]
      proportionnal_to_velocity: 0.05  # 5% velocity-dependent noise
```

### Robot with environment disturbances
Such as wind for aerial robots or water currents for underwater vehicles:
```yaml
physics:
  type: Internal
  model:
    type: Unicycle
    wheel_distance: 0.2
  initial_state:
    pose: [0.0, 0.0, 0.0]
    velocity: [0.0, 0.0]
  faults:
    - type: AdditiveRobotCentered
      distributions:
        - type: Normal
          mean: [0.0, 0.0]
          covariance: [0.001, 0.0, 0.0, 0.001]
          variable_order: [x, y]
      proportionnal_to_velocity: 0. # Not dependent on velocity, distrurbance apply event in stationary mode
```

---
## See Also

- [Controller Configuration](controller.md) - Generates motor commands for physics
- [Navigation Configuration](navigation.md) - Provides targets for controller
- [Configuration Tips & Tricks](../config_tips.md#physics-tips) - Physics setup examples
