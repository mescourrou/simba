# Environment Configuration

## Overview

The `environment` section defines the static world in which your robots operate. This includes maps, landmarks, and other environmental features that can be sensed by your robots (only landmarks for now). 

The environment is used by sensors to detect obstructions.

## `environment` Section

```yaml
  map_path: "path/to/map.yaml" # Path to a YAML file defining landmarks (relative to config file). Relative to the config file location.
```

## Map
The map is defined as a YAML file containing a list of landmarks. Each landmark has a unique ID, position, and orientation. You can also define planar landmarks with width and height for occlusion handling.

### Landmark Map Format

Create a YAML file (e.g., `maps/landmarks.yaml`):

```yaml
landmarks:
  - id: 1                    # Unique landmark identifier
    x: 5.0                   # X coordinate (meters)
    y: 5.0                   # Y coordinate (meters)
    theta: 0.0               # Orientation (radians)
  - id: 2
    x: 10.0
    y: 10.0
    theta: 1.57
```

See the `config_example/maps/landmarks_square.yaml` file in the repository for a complete example.

You can define planar landmarks, by the addition of a `width` field. `x`, `y`, and `theta` define the center pose, while `width` defines the size of the landmark.
```yaml
  - id: 3
    x: 15.0
    y: 5.0
    theta: 0.0
    width: 1.0               # Landmark width (meters)
```

This landmark can be partially visible depending on the height. Even with 2D world, occlusions are considered based on landmark height: a higher landmark can occlude a lower ones. 0 height means the landmark is always visible.

```yaml
  - id: 4
    x: 20.0
    y: 5.0
    theta: 0.0
    width: 1.0
    height: 2.0              # Landmark height (meters)
  - id: 5
    x: 20.0
    y: 7.0
    theta: 0.0
    width: 1.0
    height: 1.0              # Lower height, can be occluded by landmark 4
```

Please note that ponctual landmarks (without width) cannot occlude other landmarks but can be occluded by planar landmarks if they are lower in height.

**Use cases**:
- GPS-denied localization
- Marker-based navigation
- Visual landmark tracking