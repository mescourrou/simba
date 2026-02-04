# Scenario Configuration

!!! tip "Quick Navigation"
    - ← [Back to Configuration Reference](../config_reference.md)
    - See also: [Robots Configuration](../config_reference.md#robots-configuration)

The `scenario` component defines dynamic events that occur during simulation - robot spawning, collisions, area-based events, etc.

## `scenario` Configuration

**Type**: `ScenarioConfig`  
**Optional**

Define events that happen at specific conditions during the simulation.

```yaml
scenario:
  events:
  - trigger:
      type: Time
      value:
        time:
          type: Num
          value: 5.0
        occurences: 
          type: Num
          value: 3 # If != 1, event repeats this many times, with the period defined by 'time'
    event_type:
      type: Spawn
      model_name: robot_template
      node_name: robot3
  - trigger:
      type: Proximity
      distance: 1.0
      inside: true
    event_type:
      type: Kill
      target: $0 # Kill robot when it gets too close to another
```

## Event Structure

Each event has two parts:

```yaml
- trigger:       # When does the event happen?
    type: ...
    ...
  event_type:    # What happens?
    type: ...
    ...
```

## Event Triggers

Define conditions that cause events.

### Trigger Type: `Time`

Trigger at specific simulation time.

```yaml
trigger:
  type: Time
  value:
    time:
      type: Num
      value: 5.0     # Trigger at 5 seconds
    occurences: 
      type: Num
      value: 1         # Number of times to trigger (1 = once, default value)
```

**Parameters**:
- `time`: Simulation time (seconds)
- `occurences`: Three cases:
-- if `time` is a fixed number, it turns into a period and `occurences` is how many times the event will be triggered. Use 0 for infinite (uses the simulation max time given in the configuration).
-- if `time` is a random variable, `occurences` is how many samples will be drawn from it to schedule the event
-- if `time` is a random variable with multiple dimensions, `occurences` is the number of repetitions of the full set of samples: if `time` draws N samples, and `occurences` is M, then M*N event times will be scheduled

**Variable settings**:
Sets the `$0` variable to the occurence index (starting at 0).

**Examples**:

Trigger every 10 seconds throughout simulation:
```yaml
trigger:
  type: Time
  value:
    time:
      type: Num
      value: 10.0
  occurences: 0                   # Repeats every 10 seconds
```

Trigger multiple times:
```yaml
trigger:
  type: Time
  value:
    time:
      type: Num
      value: 5.0                     # Every 5 seconds
  occurences: 3                      # At 5s, 10s, 15s
```

Time can be set with a probabilistic distribution:
```yaml
trigger:
  type: Time
  value:
    time:
      type: Rand
      value:
        type: Exponential
        rate: 0.1                        # Average every 10 seconds
    occurences: 3                      # Draw 3 samples from distribution, around 10 seconds.
```

### Trigger Type: `Proximity`

Trigger when robots are close (distance-based).

```yaml
trigger:
  type: Proximity
  protected_target: robot1           # Robot to be compared against
  distance: 2.0                      # Distance threshold
  inside: true                       # true = within distance, false = outside
```

**Parameters**:
- `protected_target`: Check proximity to specific robot (optional; if omitted, checks all robots). Will trigger the event (might be killed)
- `distance`: Distance threshold
- `inside`: 
  - `true`: Trigger when robots are closer than distance
  - `false`: Trigger when robots are farther than distance

**Variable settings:**
Sets `$0` to the name of the robot involved in the trigger. Multiple robots are involved in the case of proximity. Each robot trigger an event with `$0` sets to its name.

**Examples**:

Trigger when any two robots get within 1 meter:
```yaml
trigger:
  type: Proximity
  value:
    distance: 1.0
    inside: true
```

Trigger when robots get further than 10 meters than `robot1`:
```yaml
trigger:
  type: Proximity
  value:
    protected_target: robot1
    distance: 10.0
    inside: false
```

### Trigger Type: `Area`

Trigger when robot enters/exits an area.

#### Rectangular Area
```yaml
trigger:
  type: Area
  value:
    type: Rect
    bottom_left: [0.0, 0.0]
    top_right: [10.0, 10.0]
    inside: true                       # true = enter area, false = leave area
```

#### Circular Area
```yaml
trigger:
  type: Area
  value:
    type: Circle
    center: [5.0, 5.0]
    radius: 2.0
    inside: true
```

**Parameters**:
- For `Rect`: `bottom_left` and `top_right` coordinates
- For `Circle`: `center` point and `radius`
- `inside`: 
  - `true`: Trigger when entering area
  - `false`: Trigger when leaving area

**Variable settings**:
Sets `$0` to the name of the robot that triggered the event by entering or leaving the area.

**Examples**:

Trigger when any robot enters 5x5 square at origin:
```yaml
trigger:
  type: Area
  value:
    type: Rect
    bottom_left: [0.0, 0.0]
    top_right: [5.0, 5.0]
    inside: true
```

Trigger when robot leaves circular zone:
```yaml
trigger:
  type: Area
  value:
    type: Circle
    center: [50.0, 50.0]
    radius: 20.0
    inside: false
```

## Event Types

Define what happens when a trigger activates.

### Event Type: `Spawn`

Create a new robot during simulation.

```yaml
event_type:
  type: Spawn
  model_name: robot_template         # Name of robot config to clone
  node_name: new_robot               # Name for spawned robot.
```

**Requirements**:
1. `model_name` must reference a robot defined in `robots` list
2. `node_name` must be unique.  You can use variables like `$0`, etc. for dynamic naming when spawning multiple robots (time with occurences > 1)

**How it works**:
- SiMBA copies the specified robot configuration
- New robot starts at the same initial position as the template (redraw if random)
- New robot name is `node_name`

**Example**:

Define robot template:
```yaml
robots:
  - name: robot_template
    navigator: { type: GoTo, target_point: [10.0, 10.0], target_speed: 0.5 }
    controller: { type: PID, proportional_gains: [1.0, 1.0, 1.0], ... }
    physics: { type: Internal, model: { type: Unicycle, wheel_distance: 0.2 }, ... }
    state_estimator: { type: Perfect, prediction_period: 0.1, targets: [self] }
    sensor_manager: { sensors: [] }
    network: { range: 0.0, reception_delay: 0.0 }
    autospawn: false                 # Don't spawn at start
```

Spawn it dynamically:
```yaml
scenario:
  events:
    - trigger:
        type: Time
        value:
          time:
            type: Num
            value: 10.0
          occurences: 1
      event_type:
        type: Spawn
        model_name: robot_template
        node_name: robot1
```

### Event Type: `Kill`

Remove a robot from simulation.

```yaml
event_type:
  type: Kill
  value: robot_name                 # Robot to remove
```

**Parameters**:
- `value`: Name of robot to remove. Use `$0` to refer to robots involved in the trigger (e.g., proximity).

**Example**:

Remove robot after 20 seconds:
```yaml
scenario:
  events:
    - trigger:
        type: Time
        time: 20.0
        occurences: 1
      event_type:
        type: Kill
        target: robot1
```

## Common Scenario Patterns

### Robots enter one-by-one
```yaml
scenario:
  events:
    - trigger:
        type: Time
        value:
          time:
            type: Num
            value: 5.0
          occurences: 3
      event_type:
        type: Spawn
        model_name: robot_template
        node_name: robot$0  # Dynamic naming with occurence index
```

### Robots spawn periodically
```yaml
scenario:
  events:
    - trigger:
        type: Time
        value:
          time:
            type: Num
            value: 10.0
        occurences: 0             # Repeat
      event_type:
        type: Spawn
        model_name: robot_template
        node_name: robot_$0  # Dynamic naming
```

### Collision detection
```yaml
scenario:
  events:
    - trigger:
        type: Proximity
        value:
          distance: 0.5
          inside: true
      event_type:
        type: Kill
        value: $0
```

### Zone entry/exit
```yaml
scenario:
  events:
    - trigger:
        type: Area
        value:
          type: Rect
          bottom_left: [0.0, 0.0]
          top_right: [10.0, 10.0]
          inside: false
      event_type:
        type: Kill
        target: $0
```

---

## See Also

- [Robots Configuration](../config_reference.md#robots-configuration) - Define robot templates
- [Configuration Tips & Tricks](../config_tips.md#scenario-tips) - Scenario setup examples
