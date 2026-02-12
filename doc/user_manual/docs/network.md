# Network System

Since version 1.5.0, the communication network is based on a publisher-subscriber, with channels identified with paths, such as ROS topics.

## Available channels:
- `/simba/scenario`: publish scenario events, such as node creation or destruction, with the event type and node name as data.
- `/simba/nodes/<node_name>/sensors/<sensor_name>`: allow to trigger the sensor with a `SensorTrigger` message.
- `/simba/nodes/<node_name>/sensors/observations`: allow to send observation to `node_name`.
- `/simba/nodes/<node_name>/navigator/goto`: if using a `GoTo` navigator, allow to change the target point, target speed and stop distance with a `GoTo` message.
- `/simba/command/<node_name>`: allow to send a command to `node_name` using message Flags. For now, only `Kill` is available.

By default, the node network is configured to send messages with the base `/simba/nodes/<node_name>` path. If you want to send a message elsewhere, use absolute paths (starting with `/`) instead of relative paths.