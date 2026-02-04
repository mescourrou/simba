# Computation Units Configuration

!!! tip "Quick Navigation"
    - ← [Back to Configuration Reference](../config_reference.md)
    - See also: [State Estimator Configuration](state_estimator.md), [Network Configuration](network.md)

Computation units are centralized computing nodes that run algorithms on data from multiple robots. Use them for centralized control, global state estimation, or resource-intensive algorithms.

## `computation_units` Configuration

**Type**: List of `ComputationUnitConfig`  
**Optional**

Define one or more computation units for global algorithms.

```yaml
computation_units:
  - name: Central Unit
    state_estimators:
      - name: central_estimator
        config:
          type: Perfect
          prediction_period: 0.1
          targets:
            - robot1
            - robot2
```

## Computation Unit Fields

### `name`
Unique identifier for the computation unit. Used in logging, results, and `send_to` references.

```yaml
name: Central Unit
```

### `state_estimators`
List of state estimation algorithms running on this unit.

```yaml
state_estimators:
  - name: central_estimator
    config:
      type: Perfect
      prediction_period: 0.1
      targets:
        - robot1
        - robot2
```

Each estimator has:
- `name`: Identifier for this estimator
- `config`: Full state estimator configuration (see [State Estimator Configuration](state_estimator.md))

A computation unit can run multiple estimators if needed (e.g., one for localization, one for obstacle detection).

## Typical Use Cases

### Centralized State Estimation

One unit estimates all robots' states:

```yaml
computation_units:
  - name: Central Localization
    state_estimators:
      - name: global_slam
        config:
          type: Python
          file: central_slam.py
          class_name: CentralSLAM
          prediction_period: 0.1
          targets:
            - robot1
            - robot2
            - robot3
          map_path: maps/global_map.yaml
```

Each robot must send its sensor data to the central unit:
```yaml
  # In robot config
  sensor_manager:
    sensors:
      - name: lidar
        send_to: [Central Localization]
        config: { type: OrientedLandmarkSensor, map_path: maps/landmarks.yaml }
```

### Multiple Specialized Units

Different units handle different tasks:

```yaml
computation_units:
  - name: Localization Unit
    state_estimators:
      - name: slam
        config:
          type: Python
          file: slam_estimator.py
          class_name: SLAMEstimator
          targets: [robot1, robot2]
          prediction_period: 0.1

  - name: Planning Unit
    state_estimators:
      - name: monitor
        config:
          type: Perfect
          targets: [robot1, robot2]
          prediction_period: 0.1
```

## Data Flow with Computation Units

```
Robot 1 Sensor → send_to: [Central Unit]
Robot 2 Sensor → send_to: [Central Unit]
                     ↓
         Central Unit State Estimator
```

---

## See Also

- [State Estimator Configuration](state_estimator.md) - Full estimator reference
- [Network Configuration](network.md) - Communication settings
- [Sensor Configuration](sensors.md) - How data reaches computation units
- [Configuration Tips & Tricks](../config_tips.md#computation-unit-tips) - Setup examples
