# Simple Planner

The Simple Planner is a basic path planning component of the AROLA (Autonomous Racing Open Layered Architecture) planning layer. It reads waypoints from a CSV file and publishes them as a ROS 2 Path message.

## Overview

This planner serves as a simple reference implementation that:
- Reads waypoints from a CSV file containing x, y coordinates and desired velocities
- Publishes these waypoints as a `nav_msgs/Path` message
- Encodes velocity information in the angular z (w) field of each pose
- Supports waypoint order inversion for reverse path traversal

## Node: simple_path_node

### Functionality
- Reads waypoints from a specified CSV file
- Publishes a path at a configurable rate
- Handles CSV files with comment headers (lines starting with #)
- Validates waypoint data and reports parsing errors

### Topics

#### Publishers
- `~/planned_path` (`nav_msgs/Path`): The planned path with waypoints

### Parameters
- `csv_path` (string): Path to CSV file containing waypoints (format: x,y,v)
- `invert` (bool): Whether to invert the order of waypoints (default: false)
- `frame_id` (string): Frame ID for the waypoints (default: "map")
- `publish_rate` (double): Rate at which to publish path messages in Hz (default: 1.0)

### CSV Format
The expected CSV format is:
```
# x_m, y_m, v_m/s
-4.7125862,3.4176980,2.6816582
-4.6185579,3.2392036,2.6847125
...
```

Where:
- Lines starting with `#` are treated as comments and ignored
- Each data line contains: x_coordinate, y_coordinate, desired_velocity
- Coordinates are in meters, velocity in m/s

### Velocity Encoding
The velocity is encoded in the `angular.z` field of each `PoseStamped` in the path. This is a non-standard usage but allows passing velocity information through the standard Path message format.

## Usage

### Standalone Launch
```bash
ros2 launch simple_planner simple_planner.launch.py
```

### With Custom Parameters
```bash
ros2 launch simple_planner simple_planner.launch.py csv_path:=/path/to/your/waypoints.csv publish_rate:=2.0
```

### As Part of AROLA
The simple planner is automatically included when launching the full AROLA system:
```bash
ros2 launch core core.launch.py
ros2 launch core simulation.core.launch.py  # For simulation
```

## Configuration

Edit `config/params.yaml` to modify default parameters:
```yaml
simple_planner:
  ros__parameters:
    csv_path: "src/control/pure_pursuit/path/example_path.csv"
    invert: false
    frame_id: "map"
    publish_rate: 1.0
```

## Example Waypoint File

An example waypoint file is provided at `src/control/pure_pursuit/path/example_path.csv` which contains a sample racing trajectory with corresponding velocities.

This planner provides a foundation for more sophisticated planning algorithms while maintaining simplicity and reliability for basic autonomous racing scenarios.
