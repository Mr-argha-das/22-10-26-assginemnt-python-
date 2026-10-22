# F1TENTH Watchdog

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)

A comprehensive system monitoring and safety watchdog for F1TENTH autonomous racing vehicles. This ROS2 package provides real-time monitoring of critical system components to ensure safe vehicle operation.

## Features

-   **Battery Monitoring**: Continuous voltage monitoring with configurable thresholds
-   **Motor Temperature Monitoring**: Real-time temperature tracking with thermal protection
-   **LiDAR Health Monitoring**: Timeout detection and status reporting
-   **Camera Status Monitoring**: Live feed verification and timeout detection
-   **Motor Status Tracking**: Velocity and angular velocity monitoring
-   **Critical System Alerts**: Emergency stop signals for unsafe conditions
-   **Comprehensive Status Reporting**: Detailed system health information

## System Requirements

-   ROS2 Humble or later
-   Python 3.8+
-   VESC interface (vesc_msgs)
-   Standard ROS2 sensor message packages

## Installation

1. Clone this repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone <repository-url> watchdog
```

2. Install dependencies:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:

```bash
colcon build --packages-select watchdog
```

4. Source the workspace:

```bash
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch watchdog watchdog.launch.py
```

### Custom Configuration

```bash
ros2 launch watchdog watchdog.launch.py config_file:=/path/to/custom/config.yaml
```

### Direct Node Execution

```bash
ros2 run watchdog watchdog_node
```

## Configuration

The watchdog node is highly configurable through parameters. The default configuration is located in `config/watchdog_params.yaml`.

### Key Parameters

-   `critical_voltage`: Battery voltage threshold for critical alerts (default: 12.0V)
-   `temp_warning_start`: Temperature threshold for warnings (default: 80.0°C)
-   `lidar_timeout`: Maximum time between LiDAR messages (default: 0.5s)
-   `camera_timeout`: Maximum time between camera frames (default: 0.5s)

See `config/watchdog_params.yaml` for complete parameter documentation.

## Topics

### Subscribed Topics

-   `/sensors/core` (vesc_msgs/VescStateStamped): VESC sensor data
-   `/camera/camera/color/image_raw` (sensor_msgs/Image): Camera feed
-   `/scan` (sensor_msgs/LaserScan): LiDAR data
-   `/odom` (nav_msgs/Odometry): Vehicle odometry

### Published Topics

-   `/tmp/watchdog/critical` (std_msgs/Bool): Critical system alert
-   `/watchdog/system/status` (std_msgs/String): Comprehensive system status
-   `/tmp/watchdog/camera_is_live` (std_msgs/Bool): Camera status

## Safety Features

The watchdog monitors the following critical conditions:

1. **Low Battery Voltage**: Triggers critical alert when voltage drops below threshold
2. **High Motor Temperature**: Monitors for thermal overload conditions
3. **LiDAR Timeout**: Detects sensor communication failures
4. **Camera Timeout**: Monitors camera feed availability

When critical conditions are detected, the node publishes emergency stop signals to prevent unsafe operation.

## Development

### Running Tests

```bash
cd ~/ros2_ws
colcon test --packages-select watchdog
```

### Code Quality

This package follows PEP 8 style guidelines and includes:

-   Type hints for better code documentation
-   Comprehensive docstrings
-   Error handling and logging
-   Modular design for maintainability

## Authors

-   **Fam Shihata** - _Maintainer_ - fam@awadlouis.com
-   **Mohammed Azab** - _Original co-author_ - mo7ammed3zab@outlook.com
-   **Nada Mahmoud** - _Original co-author_

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Support

For questions and support, please contact the maintainers or open an issue on the repository.
