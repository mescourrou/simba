# Network Configuration

!!! tip "Quick Navigation"
    - ← [Back to Configuration Reference](../config_reference.md)
    - See also: [Sensor Configuration](sensors.md), [Computation Units](computation_units.md)

The `network` component configures inter-robot and robot-to-unit communication.

## `network` Configuration

**Type**: `NetworkConfig`  
**Required**: Yes

Configures communication between robots and computation units.

```yaml
network:
  range: 10.0                        # Communication range (meters)
  reception_delay: 0.01              # Message delay (seconds)
```

**Parameters**:
- `range`: Maximum distance for direct communication
  - `0.0` = Unlimited range (all robots can communicate)
  - `> 0` = Line-of-sight range in meters
- `reception_delay`: Network latency added to all messages (seconds). No-latency messages are possible with `0.0`.

## Communication Range

### Unlimited communication
```yaml
network:
  range: 0.0                         # All robots can communicate
  reception_delay: 0.0               # No delay
```

Use this when:
- Network is not the focus of your study
- You want to test pure algorithm behavior

### Limited communication
```yaml
network:
  range: 50.0                        # 50-meter range
  reception_delay: 0.01              # 10 ms delay
```

Use this when:
- Modeling WiFi or wireless communication
- Range is critical to your application
- Testing network-degraded scenarios

## Reception Delay

Adds latency to all messages:

```yaml
network:
  range: 0.0
  reception_delay: 0.1               # All messages delayed 100 ms
```

**Realistic values**:
- Wired network: `0.001` - `0.01` (1-10 ms)
- WiFi LAN: `0.01` - `0.05` (10-50 ms)
- Cellular: `0.05` - `0.5` (50-500 ms)
- Satellite: `0.5` - `2.0` (500+ ms)

!!! warning "Important"
    Adding reception delay increases the number of time step to compute, slowing down the simulation.

## How Networks Work in SiMBA

Network configuration controls how sensor data and state estimates propagate:

### Without explicit send_to

Sensor measurements always go to local robot's state estimator:

```
Sensor → Local State Estimator
```

### With send_to

Sensor data is shared with specified nodes:

```
Sensor → Local State Estimator
      → send_to: [robot2, Central Unit]
        (subject to network range/delay)
```

### State Estimate Sharing

State estimates from one robot can be used by another if:
1. Sensor data is sent via `send_to`
2. Network allows communication (within range)

## Multi-Robot Communication Pattern

```yaml
robots:
  - name: robot1
    network:
      range: 20.0
      reception_delay: 0.01
    sensor_manager:
      sensors:
        - name: lidar
          send_to: [robot2]          # Share with robot2
          config: { type: RobotSensor, detection_distance: 10.0 }

  - name: robot2
    network:
      range: 20.0
      reception_delay: 0.01
    state_estimator:
      type: Perfect
      targets:
        - self
        - robot1                      # Can estimate robot1
      prediction_period: 0.1
```

## Computation Unit Communication

Computation units (e.g., central control stations) also have network configuration:

```yaml
computation_units:
  - name: Central Unit
    network:
      range: 0.0                     # Unlimited range for central unit
      reception_delay: 0.0
    state_estimators: [...]
```

**Warning**: Computation unit behavior is not completely defined yet regarding network constraints. Use unlimited range for now.

## Common Network Patterns

### No network constraints (all robots communicate)
```yaml
robots:
  - name: robot1
    network: { range: 0.0, reception_delay: 0.0 }
  - name: robot2
    network: { range: 0.0, reception_delay: 0.0 }
```

### Limited WiFi range
```yaml
robots:
  - name: robot1
    network: { range: 50.0, reception_delay: 0.02 }
  - name: robot2
    network: { range: 50.0, reception_delay: 0.02 }
```

### Realistic multi-hop network
```yaml
robots:
  - name: relay
    network: { range: 100.0, reception_delay: 0.05 }
  - name: robot1
    network: { range: 100.0, reception_delay: 0.05 }
  - name: robot2
    network: { range: 100.0, reception_delay: 0.05 }
```

### Central unit with limited robot range
```yaml
robots:
  - name: robot1
    network: { range: 30.0, reception_delay: 0.01 }
  - name: robot2
    network: { range: 30.0, reception_delay: 0.01 }

computation_units:
  - name: Central Unit
    network: { range: 0.0, reception_delay: 0.0 }
```

---

## See Also

- [Sensor Configuration](sensors.md) - How to route sensor data
- [State Estimator Configuration](state_estimator.md) - How to estimate remote states
- [Computation Units](computation_units.md) - Centralized processing
- [Configuration Tips & Tricks](../config_tips.md#network-tips) - Network setup examples
