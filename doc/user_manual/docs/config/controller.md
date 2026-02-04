# Controller Configuration

!!! tip "Quick Navigation"
    - ← [Back to Configuration Reference](../config_reference.md)
    - See also: [Navigation Configuration](navigation.md), [Physics Configuration](physics.md)

The `controller` component converts navigation target commands into actual motor commands that the physics engine executes.

## `controller` Configuration

**Type**: `ControllerConfig` (Enum)  
**Required**: Yes  
**Description**: Converts navigation commands into motor commands.

## Controller Type: `PID`

Proportional-Integral-Derivative controller for trajectory tracking. This is the standard control approach for following paths.

```yaml
controller:
  type: PID
  robot_model:                      # Optional: specify robot constraints.
    type: Unicycle                  # If not specified, uses physics model.
    wheel_distance: 0.25
  proportional_gains: [1.0, 1.0]
  derivative_gains: [0.5, 0.5]
  integral_gains: [0.1, 0.1]
```

**Parameters**:
- `robot_model`: Optional. Kinematic constraints for the robot (see Robot Models below)
- `proportional_gains`, `derivative_gains`, `integral_gains`: PID coefficients as arrays. The exact values depend on your robot model, control objectives, and tuning. See [Configuration Tips & Tricks](../config_tips.md) for tuning guidance.

**How it works**:
- The navigator provides a target position/velocity
- The controller computes error (target - current state)
- PID feedback adjusts motor commands to minimize error

**Tuning gains**:
- Higher `proportional_gains` = faster response (can oscillate if too high)
- Higher `derivative_gains` = damps oscillations (reduces overshoot)
- Higher `integral_gains` = corrects steady-state errors

### Robot Models in Controller

Optionally specify robot kinematic constraints to improve control quality.

#### Unicycle Model

For differential-drive robots (skid-steer):

```yaml
robot_model:
  type: Unicycle
  wheel_distance: 0.25  # Distance between wheels (meters)
```

**When to use**: Car-like or differential-drive robots

#### Holonomic Model

For omnidirectional robots (mecanum wheels, full-body movement):

```yaml
robot_model:
  type: Holonomic
  max_longitudinal_velocity: 1.0  # Max forward speed (m/s)
  max_lateral_velocity: 1.0       # Max strafe speed (m/s)
  max_angular_velocity: 3.14      # Max rotation speed (rad/s)
```

**When to use**: Omnidirectional robots, mecanum-wheeled robots

## Controller Type: `Python`

Custom controller implemented in a single Python file and class. Use this for quick custom implementations or algorithms you want to script.

```yaml
controller:
  type: Python
  file: my_controller.py
  class_name: MyController
  # Additional user-specific parameters here
```

**How it works**:
1. SiMBA loads `my_controller.py` and instantiates `MyController`
2. Your class receives the config dict with all parameters
3. Each simulation step, your methods are called to compute control
4. You return motor commands

For Python integration details, see `examples/python/` directory.

## Controller Type: `External`

Implement custom control via the plugin API. Plugins can be written in Rust or Python (via the plugin API). Use this to implement your own control algorithms.

```yaml
controller:
  type: External
  config: {}  # Configuration passed to your plugin
```

The `config` dict is passed to your plugin. Consult the plugin API documentation and your plugin's README for required fields.

## Common Patterns

### Basic PID control

```yaml
controller:
  type: PID
  proportional_gains: [1.0, 1.0, 1.0]
  derivative_gains: [0.5, 0.5, 0.5]
  integral_gains: [0.1, 0.1, 0.1]
```

### PID with unicycle constraints

```yaml
controller:
  type: PID
  robot_model:
    type: Unicycle
    wheel_distance: 0.2
  proportional_gains: [0.8, 0.8]
  derivative_gains: [0.4, 0.4]
  integral_gains: [0.05, 0.05]
```

## See Also

- [Navigation Configuration](navigation.md) - Where control targets come from
- [Physics Configuration](physics.md) - How motor commands translate to movement
- [Configuration Tips & Tricks](../config_tips.md#controller-tips) - Control tuning examples
