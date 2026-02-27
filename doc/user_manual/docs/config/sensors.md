# Sensor Manager Configuration

!!! tip "Quick Navigation"
    - ← [Back to Configuration Reference](../config_reference.md)
    - See also: [State Estimator Configuration](state_estimator.md), [Network Configuration](network.md)

The `sensor_manager` component manages all sensors attached to a robot and controls where their measurements are sent.

## `sensor_manager` Configuration

**Type**: `SensorManagerConfig`  
**Required**: Yes

Manages all sensors attached to the robot and routes their measurements.

```yaml
sensor_manager:
  sensors:                           # List of sensors
    - name: sensor1
      send_to: []                    # Where data goes
      triggered: false               # Triggered vs periodic
      config:                        # Sensor-specific config
        type: RobotSensor
        detection_distance: 10.0
        activation_time:
          period: {type: Num, value: 0.1}
        faults: []
```

## Common Sensor Fields

### `name`
**Type**: String  
**Required**: Yes  
**Description**: Unique identifier for the sensor. Used in logging and results or to reference the origin of measurements.

```yaml
- name: my_lidar
```

### `send_to` (Optional)
**Type**: List of Strings containing node names
**Default**: Empty (observations automatically go to local state estimator)  
**Description**: List of additional nodes to share sensor data with.

By default, sensor observations are automatically sent to the local robot's state estimator. Use `send_to` to also share data with other robots or computation units.

**Valid values**:
- Name of another robot: `robot1`, `robot2`
- Name of a computation unit: `CentralUnit` (or your custom unit name)

**Note**: The local robot's state estimator always receives data automatically. Only list additional recipients.

Example sharing sensor data with a central unit:
```yaml
sensor_manager:
  sensors:
    - name: RobotDetector
      send_to: 
        - CentralUnit        # Also send to central processor
        - robot2              # Share with another robot
      config:
        type: RobotSensor
        detection_distance: 20.0
```

### `triggered`
**Type**: Boolean  
**Default**: `false`

- `true`: Sensor sends data on-demand (event-driven), based on `SensorTrigger` message reception
- `false`: Sensor sends data periodically (use sensor's `activation_time` parameter)

## Sensor Type: `RobotSensor`

Detects other robots within range. Returns position, and orientation of detected robots.

```yaml
config:
  type: RobotSensor
  detection_distance: 10.0           # Detection range (meters)
  activation_time:
    period: {type: Num, value: 0.1}  # Update period (seconds)
  faults: []                         # Sensor faults (see Sensor Faults)
  filters: []                        # Measurement filters (see Sensor Filters)
```

**Use cases**:
- Robot-to-robot collision avoidance
- Formation control
- Distributed coordination

## Sensor Type: `OrientedLandmarkSensor`

Detects landmarks (beacons, markers) with pose information. Perfect for landmark-based navigation and SLAM. Landmarks are defined in the `environment` configuration of the simulator (see [Environment Configuration](environment.md)).

```yaml
config:
  type: OrientedLandmarkSensor
  detection_distance: 10.0           # Detection range (meters)
  activation_time:
    period: {type: Num, value: 0.1}  # Update period (seconds)
  xray: false                        # Can see through walls?
  faults: []                         # Sensor faults
  filters: []                        # Measurement filters
```

**Parameters**:
- `detection_distance`: Maximum range to detect landmarks (meters)
- `activation_time`: How often measurements are updated. Use `period` for periodic updates. You can set a `table` of activation times for more complex patterns.
- `xray`: If `true`, sensor can detect landmarks even if obstructed. If `false`, height of landmarks are considered to determine visibility (or partial visibility).


**Use cases**:
- GPS-denied localization
- Marker-based navigation
- Visual landmark tracking

## Sensor Type: `SpeedSensor`

Measures robot velocity (odometry). Returns `[linear_velocity, angular_velocity]`.

```yaml
config:
  type: SpeedSensor
  activation_time:
    period: {type: Num, value: 0.1}  # Update period (seconds)
  faults: []
  filters: []
```

**Use cases**:
- Velocity estimation
- Dead reckoning

## Sensor Type: `DisplacementSensor`

Measures robot displacement (dead reckoning). Returns position changes `[dx, dy, dtheta]`.

```yaml
config:
  type: DisplacementSensor
  activation_time:
    period: {type: Num, value: 0.1}  # Update frequency (seconds)
  faults: []
  filters: []
```

**Use cases**:
- Odometry-based localization
- Dead reckoning algorithms

## Sensor Type: `GNSSSensor`

GNSS-like absolute position and velocity measurement. Returns global coordinates.

```yaml
config:
  type: GNSSSensor
  activation_time:
    period: {type: Num, value: 0.1}  # Update period (seconds)
  faults: []
  filters: []
```

**Use cases**:
- Simulating GPS receivers
- Global localization
- Testing GPS-based algorithms



## Sensor Faults

Add realistic sensor noise and failures. The faults depend on the sensor used.

```yaml
faults:
- type: AdditiveRobotCentered
  apparition:
    probability: [1.0]             # Always present (default)
  distributions:
    - type: Normal
      mean: [0.0, 0.0]
      covariance: [0.01, 0.0, 0.0, 0.01]
      variable_order: [x, y]
- type: Misdetection
  apparition:
    probability: [0.05]            # 5% chance to miss detection
```

**Fault types**:
- `AdditiveRobotCentered[Polar]`: Gaussian noise in robot frame (polar/cartesian)
- `AdditiveObservationCenteredPolar`: Gaussian noise in observation frame (polar)
- `Misdetection`: Randomly miss detections
- `Misassociation`: Swap landmark/robots IDs randomly
- `Clutter`: Add false positive detections
- `Python`: Custom fault defined in Python

## Sensor Filters

Filter or validate measurement data:

```yaml
filters:
  - type: RangeFilter
    variables: [x, y]
    min_range: [-5.0, -5.0]
    max_range: [5.0, 5.0]
    inside: true                     # Keep measurements inside range
  - type: IdFilter
    accepted: [landmark_.*]          # Regex for accepted IDs
    rejected: []
    priority_accept: true            # Accept list takes priority
```

**Common filter types**:
- `RangeFilter`: Keep measurements within bounds
- `IdFilter`: Accept/reject by landmark ID

**Filter parameters**:
- `variables`: List of variable names to filter on (in the list `[x, y, orientation, theta, position_x, position_y, velocity_x, velocity_y, w, v, self_velocity, target_velocity, width, height]` depending on the sensor type)
- `min_range`, `max_range`: Min/max bounds for each variable
- `inside`: If `true`, keep measurements inside the defined range; if `false`, keep those outside
- `accepted`, `rejected`: Lists of regex patterns for IDs to accept/reject
- `priority_accept`: How to manage intersection between accepted and rejected sets. If `true`, when an identifier is in the accepted and rejected sets, it is accepted.

## Common Sensor Configurations

### Robot with landmark detection
```yaml
sensor_manager:
  sensors:
    - name: landmark_sensor
      config:
        type: OrientedLandmarkSensor
        detection_distance: 15.0
        activation_time:
          period: {type: Num, value: 0.1}
```

### Robot sharing observations with central unit
```yaml
sensor_manager:
  sensors:
    - name: robot_detector
      send_to:
        - Central Unit
      config:
        type: RobotSensor
        detection_distance: 20.0
        activation_time:
          period: {type: Num, value: 0.05}
```

### Sensor with realistic noise
```yaml
sensor_manager:
  sensors:
    - name: gps
      config:
        type: GNSSSensor
        activation_time:
          period: {type: Num, value: 1.0}
        faults:
          - type: AdditiveRobotCentered
            apparition:
              probability: [1.0]
            distributions:
              - type: Normal
                mean: [0.0, 0.0]
                covariance: [1.0, 0.0, 0.0, 1.0]  # 1 meter std dev
                variable_order: [x, y]
```

---

## See Also

- [State Estimator Configuration](state_estimator.md) - How sensors are used to estimate state
- [Network Configuration](network.md) - How sensor data is shared between robots
- [Configuration Tips & Tricks](../config_tips.md#sensor-tips) - Sensor setup patterns and troubleshooting
