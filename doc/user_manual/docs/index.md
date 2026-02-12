# SiMBA: Multi-Robot Backend Simulator 

## User Documentation

SiMBA is a simulator designed to abstract sensor measurements to facilitate the implementation of backend fusion algorithms or visual servoing. The simulator supports multiple robots, and the time rate is dynamic, waiting for the algorithms to finish before jumping to the next time step. It is written in Rust, and Python bindings are available, with a loss of performance. Its modular architecture allows for extension of different components while using the default version of the others.

---

## Quick Navigation

### New to SiMBA? Start here:

1. **[Getting Started Guide](config_getting_started.md)** ← Start here!
   - Minimal working example
   - Key concepts explained
   - Simple configurations

2. **[Complete Configuration Reference](config_reference.md)**
   - All configuration options documented
   - Organized by sections (robots, sensors, navigation, etc.)
   - Examples for each feature

3. **[Tips & Tricks](config_tips.md)**
   - Common patterns (multi-robot, faults, coordination)
   - Best practices
   - Troubleshooting guide

### Advanced Documentation

- **[Auto-generated API Reference](config_documentation.md)** - Complete field listing (for reference)
- **[Python Integration](python/using_python.md)** - Custom Python components
- **[Plugin Development](plugin/index.md)** - Extending SiMBA
- **[Rust API Documentation](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/)** - Technical implementation details

---

## What You'll Configure

Most of the simulator behavior is defined in a **YAML configuration file**. You can specify:

- **Simulation parameters**: duration, randomness, logging, output
- **Robots**: navigation, control, physics, sensors
- **Computation units**: centralized algorithms
- **Scenarios**: dynamic events during simulation

The simulator provides sensible defaults, so you only need to configure what you want to customize.

---

## Node Types

The simulator manages multiple nodes in a network:

### Robots
Connected nodes that can move, sense and compute. Each robot has:
- Navigation strategy (how to reach goals)
- Controller (how to generate motor commands)
- Physics model (how it moves)
- Sensors (what it perceives)
- State estimator (what it knows about itself)

### Computation Units
Connected nodes without physical presence. They can:
- Run state estimation algorithms
- Process sensor data from robots
- Run coordination algorithms
- Not move or sense directly

---

## Documentation Structure

| Guide | Best For | Topics |
|-------|----------|--------|
| [Getting Started](config_getting_started.md) | First-time users | Minimal examples, basic concepts, structure |
| [Configuration Reference](config_reference.md) | Learning all options | Every field documented, organized by component |
| [Tips & Tricks](config_tips.md) | Solving problems | Patterns, best practices, debugging |
| [Auto-generated Docs](config_documentation.md) | Quick lookup | Field types, links to Rust API |
| [Detailed Reference (Old)](config/0_introduction.md) | Legacy information | Older documentation format |

---

## Example Configurations

Working examples are available in the repository:
- `config_example/config.yaml` - Complete real-world configuration
- `config_example/config_2.yaml` - Alternative scenario
- `config_example/config_scenario.yaml` - With scenario events
- `examples/*/config_example/` - Various use cases

Use these as templates for your own simulations!

---

## Common Tasks

### Setting up your first simulation
👉 Follow the [Getting Started Guide](config_getting_started.md)

### Understanding all available features
👉 See the [Complete Configuration Reference](config_reference.md)

### Configuring sensors
👉 Search for "Sensor" in the [Reference](config_reference.md) or see examples in Tips

### Debugging your configuration
👉 Check [Tips & Tricks - Troubleshooting](config_tips.md#troubleshooting)

### Adding custom Python algorithms
👉 See [Python Integration](python/using_python.md)

### Creating fault models
👉 See "Fault Injection Testing" in [Tips & Tricks](config_tips.md#pattern-2-fault-injection-testing)

### Sending messages between nodes
👉 See [Network System](network.md)

---

## Schema Validation

Enable auto-completion in your editor by adding this line to your config file:
```yaml
# yaml-language-server: $schema=/path/to/config.schema.json
```

This provides real-time validation and suggestions! The `config.schema.json` file is included with each release and available at the root of the repository.

---

## Getting Help

1. **Check the troubleshooting guide**: [Tips & Tricks - Troubleshooting](config_tips.md#troubleshooting)
2. **Look at examples**: Browse `config_example/` directory
3. **Search the reference**: [Configuration Reference](config_reference.md)
4. **Check the Rust docs**: [Full API docs](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/)

---

## Key Concepts

### Units & Coordinate System
- **Distance**: meters
- **Time**: seconds
- **Angle**: radians
- **Coordinates**: Cartesian (X, Y), with Z as height for 3D
- **Pose**: [x, y, theta] for 2D (position + orientation)

### Frequency/Period
- When you see `period: 0.1`, that's 0.1 seconds = 10 Hz
- Larger period = fewer updates = faster simulation

### Robot Models
- **Unicycle**: 2D differential drive (ground robots)
- **Holonomic**: Can move in any direction (drones, omnidirectional bases)