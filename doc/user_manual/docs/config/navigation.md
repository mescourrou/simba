# Navigation Configuration

!!! tip "Quick Navigation"
    - ← [Back to Configuration Reference](../config_reference.md)
    - See also: [Controller Configuration](controller.md), [Physics Configuration](physics.md)

The `navigator` component determines how a robot chooses its path and target goal.

## `navigator` Configuration

**Type**: `NavigatorConfig` (Enum)  
**Required**: Yes  
**Description**: Determines how a robot chooses its path and target.

## Navigator Type: `GoTo`

Simple point-to-point navigation to a fixed goal.

```yaml
navigator:
  type: GoTo
  target_point: [5.0, 5.0]           # Goal coordinates [x, y]
  target_speed: 0.5                  # Speed toward goal (m/s)
  stop_distance: 0.1                 # Stopping tolerance (meters)
  stop_ramp_coefficient: 0.5         # Speed reduction near goal
```

**Parameters**:
- `target_point`: Target goal in meters `[x, y]`. Optional - can be set to `null` and updated via commands.
- `target_speed`: Desired speed toward the goal (m/s)
- `stop_distance`: Distance tolerance to consider goal reached
- `stop_ramp_coefficient`: How quickly speed reduces when approaching goal (0-1)

**Remarks**:
- If `target_point` is set to `null`, the robot will wait for external commands to set the goal during simulation (through `GoTo` messages).
- The robot will slow down as it approaches the goal based on the `stop_ramp_coefficient`.

## Navigator Type: `TrajectoryFollower`

Follow a pre-defined path from a file. Perfect for predefined patrol routes, racing lines, or exploration paths.

```yaml
navigator:
  type: TrajectoryFollower
  trajectory_path: paths/path1.yaml  # Path to trajectory file
  forward_distance: 0.2              # Look-ahead distance (meters)
  target_speed: 0.5                  # Desired speed (m/s)
  stop_distance: 0.2                 # Final stopping tolerance
  stop_ramp_coefficient: 0.5         # Deceleration rate
```

**Parameters**:
- `trajectory_path`: Path to YAML file containing waypoints (relative to config file location)
- `forward_distance`: Look-ahead distance for path following (meters)
- `target_speed`: Speed along the path (m/s)
- `stop_distance`: Tolerance when reaching the end
- `stop_ramp_coefficient`: Speed reduction rate (0-1)

### Trajectory File Format

Create a YAML file (e.g., `paths/path1.yaml`) with this structure:

```yaml
point_list:
  - [0.0, 0.0]       # Starting point
  - [1.0, 0.0]       # Waypoint 2
  - [2.0, 1.0]       # Waypoint 3
  - [3.0, 1.0]       # Ending waypoint
do_loop: false       # Return to start? true/false
```

**Fields**:
- `point_list`: Array of coordinates `[x, y]` in meters, in order of travel
- `do_loop`: Boolean. If `true`, robot returns to start after reaching last point and restarts the path. Default is `true`.

**Example with looping**:
```yaml
point_list:
  - [0.0, 0.0]
  - [0.0, 15.0]
  - [15.0, 15.0]
  - [15.0, 0.0]
do_loop: true        # Forms a square patrol pattern
```

See the `config_example/paths/path1.yaml` file in the repository for a complete example.

## Navigator Type: `Python`

Implement custom navigation logic in Python. Use this for complex decision-making, learning algorithms, or behaviors not covered by built-in types.

```yaml
navigator:
  type: Python
  file: my_navigator.py              # Python file with navigator class
  class_name: MyNavigator            # Class name in the file
  # Additional user-specific parameters here
  custom_param1: value1              # Your custom configuration
```

**How it works**:
1. SiMBA loads `my_navigator.py` and instantiates the `MyNavigator` class
2. Your class receives the `config` dict with all parameters (including `custom_param1`)
3. Each simulation step, your navigator's `get_navigation_command()` method is called
4. You return target position/velocity for the controller to track

For Python integration details, see `examples/python/` directory.

## Navigator Type: `External`

Implement custom navigation via external plugin (Rust or Python). Use this for maximum performance on compute-intensive navigation algorithms.

```yaml
navigator:
  type: External
  config: {}  # Provide configuration for your plugin
```

The `config` dict is passed to your plugin at initialization. Consult your plugin's documentation for required fields.

## Common Patterns

### Multi-waypoint patrol
Use `TrajectoryFollower` with `do_loop: true`:
```yaml
navigator:
  type: TrajectoryFollower
  trajectory_path: paths/patrol.yaml
  target_speed: 0.3
```

### Dynamic target
Use `GoTo` with `target_point: null`:
```yaml
navigator:
  type: GoTo
  target_point: null                 # Set via message
  target_speed: 1.0
  stop_distance: 0.05
```

### Custom decision making
Use `Python` type:
```yaml
navigator:
  type: Python
  file: smart_navigator.py
  class_name: SmartNavigator
  decision_threshold: 0.8
  explore_mode: true
```

---

## See Also

- [Controller Configuration](controller.md) - How control commands are generated
- [State Estimator Configuration](state_estimator.md) - Inputs for navigation (robot state)
- [Configuration Tips & Tricks](../config_tips.md#navigation-tips) - Best practices for navigation setup
