# F1TENTH AROLA Architecture - Minimal Software Example

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![CI/CD](https://img.shields.io/badge/CI%2FCD-passing-brightgreen.svg)](https://github.com/GIU-F1Tenth/minimal_sofware_example/actions)

## Table of Contents

- [Overview](#overview)
- [AROLA Layered Architecture](#arola-layered-architecture)
  - [Layer Descriptions](#layer-descriptions)
- [Repository Structure](#repository-structure)
- [AROLA Components](#arola-components)
  - [Sensor Layer Components](#sensor-layer-components)
  - [Preprocessing Layer Components](#preprocessing-layer-components)
  - [Perception Layer Components](#perception-layer-components)
  - [Mapping & Localization Layer Components](#mapping--localization-layer-components)
  - [Planning Layer Components](#planning-layer-components)
  - [Behavior Layer Components](#behavior-layer-components)
  - [Control Layer Components](#control-layer-components)
  - [Actuator Layer Components](#actuator-layer-components)
- [AROLA Architecture Benefits](#arola-architecture-benefits)
- [Quick Start Guide](#quick-start-guide)
  - [Prerequisites](#prerequisites)
  - [1. Install dependencies](#1-install-dependencies)
  - [2. Build the AROLA System](#2-build-the-arola-system)
  - [3. Launch the AROLA System](#3-launch-the-arola-system)
- [AROLA Configuration Philosophy](#arola-configuration-philosophy)
  - [Configuration Layer Mapping](#configuration-layer-mapping)
- [CSV Waypoint Format](#csv-waypoint-format)
- [AROLA Startup Sequence](#arola-startup-sequence)
- [AROLA Development & Debugging](#arola-development--debugging)
  - [Layer-by-Layer Debugging](#layer-by-layer-debugging)
  - [AROLA System Status Commands](#arola-system-status-commands)
  - [Common AROLA Issues & Solutions](#common-arola-issues--solutions)
- [AROLA Dependencies](#arola-dependencies)
  - [Core Dependencies](#core-dependencies)
  - [Layer-Specific Dependencies](#layer-specific-dependencies)
- [Getting Involved](#getting-involved)
  - [Contributing to AROLA](#contributing-to-arola)
  - [Development Guidelines](#development-guidelines)
- [Community & Support](#community--support)
- [License](#license)
- [About F1TENTH AROLA](#about-f1tenth-arola)

## Overview

The **F1TENTH AROLA (Autonomous Racing Open Layered Architecture)** represents a revolutionary layered software framework designed for building scaled autonomous vehicles. This minimal software example demonstrates the complete autonomous racing stack using our modular, scalable architecture.

AROLA Architecture provides separation of concerns through distinct layers, enabling rapid development, testing, and deployment of autonomous vehicle systems at any scale - from 1/10th scale racing cars to full-size autonomous vehicles.

## AROLA Layered Architecture

Our architecture follows a hierarchical design with clear separation between different system responsibilities:

```
┌─────────────────────────────────────────────────────────────┐
│                        SENSOR LAYER                         │
│               (Sensor drivers & Data collection)            │
├─────────────────────────────────────────────────────────────┤
│                     PREPROCESSING LAYER                     │
│                (Data manipulation & altering)               │
├─────────────────────────────────────────────────────────────┤
│                     PERCEPTION LAYER                        │
│                (Object detection & perception)              │
├─────────────────────────────────────────────────────────────┤
│               MAPPING & LOCALIZATION LAYER                  │
│             (Perception & State Estimation)                 │
├─────────────────────────────────────────────────────────────┤
│                    PLANNING LAYER                           │
│               (Path Planning & Navigation)                  │
├─────────────────────────────────────────────────────────────┤
│                     BEHAVIOR LAYER                          │
│                (Complex Thinking & Behavior)                |
├─────────────────────────────────────────────────────────────┤
│                    CONTROL LAYER                            │
│                (Vehicle Control System)                     │
├─────────────────────────────────────────────────────────────┤
│                    ACTUATOR LAYER                           │
│                   (Physical layer)                          │
└─────────────────────────────────────────────────────────────┘
```

### Layer Descriptions

- **Sensor Layer**: Raw sensor data collection and hardware interface management
- **Preprocessing Layer**: Data filtering, transformation, and preparation for higher-level processing
- **Perception Layer**: Object detection, classification, and environmental understanding
- **Mapping & Localization Layer**: Simultaneous localization and mapping (SLAM) and state estimation
- **Planning Layer**: Path planning algorithms, route optimization, and navigation strategy
- **Behavior Layer**: High-level decision making, racing strategy, and complex autonomous behaviors
- **Control Layer**: Low-level vehicle control algorithms and trajectory following
- **Actuator Layer**: Physical hardware control and motor/servo command execution

## Repository Structure

```
minimal_software_example/
├── src/
│   ├── core/                    # CORE LAYER
│   │   ├── launch/             # System orchestration and launch files
│   │   └── config/             # Centralized configuration management
│   │
│   ├── control/                # CONTROL LAYER
│   │   └── pure_pursuit/       # Trajectory following controller
│   │       ├── launch/
│   │       ├── config/
│   │       └── pure_pursuit/
│   │
│   ├── planning/               # PLANNING LAYER
│   │   └── simple_planner/    # Waypoint-based path planning
│   │       ├── config/
│   │       └── simple_planner/
│   │
│   ├── drivers/                # DRIVERS LAYER
│   │   └── urg_node2/         # LiDAR driver for Hokuyo sensors
│   │       ├── launch/
│   │       ├── config/
│   │       └── src/
│   │
│   └── watchdog/               # WATCHDOG LAYER
│       └── watchdog/          # System health monitoring
│           ├── launch/
│           ├── config/
│           └── watchdog/
│
├── LICENSE
└── README.md
```

## AROLA Components

### Sensor Layer Components
- **URG Node2 LiDAR Driver**
  - Package: `urg_node2` 
  - Function: Hardware abstraction for Hokuyo LiDAR sensors
  - Key Topics: Publishes `/scan`
  - Configuration: IP address, port, and scan frequency settings

### Preprocessing Layer Components
- **Data Filtering & Transformation**
  - Function: Raw sensor data filtering, noise reduction, and format standardization
  - Integration: Embedded within Nav2 sensor processing pipeline

### Perception Layer Components
- **Environmental Understanding**
  - Function: Object detection, classification, and environmental perception
  - Integration: Built into Nav2 costmap and obstacle detection systems

### Mapping & Localization Layer Components
- **Nav2 AMCL Localization**
  - Package: `nav2_amcl`
  - Function: Adaptive Monte Carlo Localization with particle filtering
  - Key Topics: Subscribes to `/scan`, `/tf`; Publishes `/tf`, `/pose`
- **Nav2 Ecosystem**: Complete navigation framework with 25+ specialized packages

### Planning Layer Components
- **Simple Planner**
  - Package: `simple_planner`
  - Function: CSV-based waypoint path planning and publishing
  - Key Topics: Publishes `/pp_path`
  - Configuration: Waypoint file path and publishing rate

### Behavior Layer Components
- **Racing Strategy & Decision Making**
  - Function: High-level autonomous behaviors and racing strategy
  - Integration: Embedded within control and planning layer decision logic

### Control Layer Components
- **Pure Pursuit Controller**
  - Package: `pure_pursuit`
  - Function: Trajectory following and velocity control
  - Key Topics: Subscribes to `/pp_path`, `/odom`; Publishes `/drive`
  - Configuration: Lookahead distance, maximum velocity, and control frequency

### Actuator Layer Components
- **Drive Command Interface**
  - Function: Physical hardware control and motor/servo command execution
  - Key Topics: Receives `/drive` commands for steering and throttle control 
## AROLA Architecture Benefits

### 🎯 **Modularity & Scalability**
- Each layer operates independently with well-defined interfaces
- Easy to swap components without affecting other layers
- Scales from 1/10th models to full-size vehicles

### 🔧 **Rapid Development**
- Clear separation of concerns accelerates development
- Parallel development across different layers
- Standardized interfaces reduce integration complexity

### 🛡️ **Built-in Safety**
- Dedicated watchdog layer monitors all system components
- Fail-safe mechanisms at every layer
- Real-time health monitoring and alerting

### 📈 **Production Ready**
- Battle-tested components from Nav2 ecosystem
- Proven in competitive F1TENTH racing environments
- Enterprise-grade software practices

## Quick Start Guide

### Prerequisites
- ROS 2 Humble or later
- Ubuntu 22.04 (recommended)
- Python 3.8 or higher
- Colcon build tools
- Docker 

### 1. Install dependencies

If you do not have them already installed, install the dependencies with:
```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  libnanoflann-dev \
  ros-humble-ompl \
  libceres-dev \
  libxtensor-dev \
  ros-humble-behaviortree-cpp-v3 \
  ros-humble-test-msgs \
  ros-humble-laser-proc \
  ros-humble-diagnostic-updater \
  ros-humble-bondcpp \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-ackermann-msgs \
  ros-humble-tf-transformations

pip install -r requirements.txt
```

If you wish to run the stack on the actual RoboRacer vehicle, additionally run
```bash
sudo apt install ros-humble-urg-node
```

### 2. Build the AROLA System
```bash
# Clone the repository
git clone https://github.com/GIU-F1Tenth/minimal_sofware_example.git
cd minimal_software_example

# Update submodules 
git submodule update --init --recursive

# Build all AROLA layers
colcon build

# Source the workspace
source install/setup.bash
```

### 3. Launch the AROLA System

For the actual vehicle, run:
```bash
ros2 launch core core.launch.py
```

For the simulation, run: 
```bash
# Launch the complete AROLA stack
ros2 launch core simulation.core.launch.py
```

Additionally, to open the simulation:
```bash 
cd f1tenth_gym_ros
sudo docker compose up -d
docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
```

Inside the simulation terminal, run:
```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

The simulation will appear at http://localhost:8080/vnc.html

## AROLA Configuration Philosophy

The AROLA architecture employs **centralized configuration management** through `src/core/config/params.yaml`, eliminating configuration sprawl and ensuring consistency across all layers.

### Configuration Layer Mapping

```yaml
# CORE LAYER - System-wide settings
core:
  use_sim_time: false
  robot_base_frame: "base_link"
  global_frame: "map"

# CONTROL LAYER - Motion control parameters
pure_pursuit:
  lookahead_distance: 1.5
  velocity_max: 3.0
  control_frequency: 20.0

# LOCALIZATION LAYER - AMCL and perception
nav2_amcl:
  max_particles: 2000
  min_particles: 500
  recovery_alpha_slow: 0.001
  recovery_alpha_fast: 0.1

# PLANNING LAYER - Path planning configuration
simple_planner:
  csv_path: "/path/to/waypoints.csv"
  frame_id: "map"
  publish_rate: 10.0

# DRIVERS LAYER - Hardware interfaces
urg_node2:
  ip_address: "192.168.1.10"
  port: 10940
  frame_id: "laser"
  scan_frequency: 40.0

# WATCHDOG LAYER - Safety monitoring
watchdog:
  monitor_frequency: 10.0
  timeout_threshold: 2.0
  critical_topics: ["/scan", "/odom", "/drive"]
```

## CSV Waypoint Format

The simple planner expects CSV files with the following format:
```
x,y,v
1.0,0.0,2.5
2.0,1.0,3.0
3.0,2.0,2.8
...
```

Where:
- `x`: X coordinate in map frame (meters)
- `y`: Y coordinate in map frame (meters)  
- `v`: Desired velocity at waypoint (m/s)

## AROLA Startup Sequence

The AROLA architecture follows a carefully orchestrated startup sequence to ensure proper system initialization:

```
1. DRIVERS LAYER     → Hardware initialization and sensor startup
   └── URG Node2     → LiDAR driver establishes sensor foundation

2. LOCALIZATION LAYER → Perception and state estimation
   └── Nav2 AMCL     → Particle filter localization initialization

3. PLANNING LAYER    → Path planning and navigation
   └── Simple Planner → Waypoint loading and path publishing

4. CONTROL LAYER     → Motion control and trajectory following
   └── Pure Pursuit  → Controller activation for vehicle motion

5. WATCHDOG          → Safety monitoring (started last)
   └── System Monitor → Comprehensive health monitoring activation
```

This startup sequence ensures that:
- Hardware interfaces are established before dependent systems
- Localization is available before path planning
- All core systems are operational before safety monitoring begins
- Each layer can gracefully handle dependencies on lower layers

## AROLA Development & Debugging

### Layer-by-Layer Debugging

#### **Drivers Layer Debugging**
```bash
# Check LiDAR connectivity and data
ros2 topic echo /scan
ros2 topic hz /scan
ros2 run tf2_tools view_frames.py
```

#### **Localization Layer Debugging**
```bash
# Monitor AMCL performance
ros2 topic echo /amcl_pose
ros2 topic echo /particlecloud
rviz2  # Visualize particle filter and pose estimates
```

#### **Planning Layer Debugging**
```bash
# Verify path planning output
ros2 topic echo /pp_path
ros2 topic hz /pp_path
# Check waypoint file format and accessibility
```

#### **Control Layer Debugging**
```bash
# Monitor control commands and odometry
ros2 topic echo /drive
ros2 topic echo /odom
# Verify trajectory following performance
```

### AROLA System Status Commands

```bash
# Overall system health check
ros2 node list | grep -E "(pure_pursuit|amcl|simple_planner|urg_node|watchdog)"

# Topic activity monitoring
ros2 topic list | head -20
ros2 topic hz /scan /odom /pp_path /drive

# Transform tree verification (critical for multi-layer coordination)
ros2 run tf2_tools view_frames.py
ros2 run tf2_ros tf2_echo map base_link

# Real-time system performance
ros2 topic bw /scan  # Bandwidth monitoring
top | grep -E "(pure_pursuit|amcl|simple_planner)"  # CPU usage
```

### Common AROLA Issues & Solutions

| Layer | Issue | Solution |
|-------|-------|----------|
| **Drivers** | LiDAR not connecting | Verify IP in params.yaml, check network connectivity |
| **Localization** | AMCL not localizing | Check map loading, set initial pose, verify scan data |
| **Planning** | No path output | Verify CSV file path/format in centralized config |
| **Control** | Poor trajectory following | Tune lookahead distance and velocity parameters |
| **Watchdog** | False alarms | Adjust timeout thresholds in watchdog configuration |

## AROLA Dependencies

### Core Dependencies
- **ROS 2 Humble** or later (Ubuntu 22.04 recommended)
- **Nav2 Navigation Stack** (complete ecosystem)
- **TF2** for coordinate frame management
- **Colcon** build system

### Layer-Specific Dependencies

#### Drivers Layer
- `urg_node2` - Hokuyo LiDAR driver
- Hardware-specific sensor drivers

#### Localization Layer  
- `nav2_amcl` - Adaptive Monte Carlo Localization
- `nav2_costmap_2d` - Costmap management
- `nav2_lifecycle_manager` - Node lifecycle management

#### Control Layer
- Custom `pure_pursuit` package
- `geometry_msgs` - ROS message types
- `nav_msgs` - Navigation message types

#### Planning Layer
- Custom `simple_planner` package
- CSV file parsing libraries

#### Watchdog Layer
- Custom `watchdog` package
- System monitoring utilities

## Getting Involved

### Contributing to AROLA
We welcome contributions to the F1TENTH AROLA Architecture:

1. **Fork** the repository
2. **Create** a feature branch following layer conventions
3. **Implement** your changes with proper layer isolation
4. **Test** across all affected layers
5. **Submit** a pull request with detailed layer impact analysis

### Development Guidelines
- Follow the layered architecture principles
- Maintain clear interfaces between layers
- Update centralized configuration appropriately
- Include comprehensive testing for safety-critical components

## Community & Support

- **Documentation**: [F1TENTH AROLA Architecture Docs](https://docs.f1tenth.org/arola)
- **Community Forum**: [F1TENTH Discourse](https://discourse.f1tenth.org)
- **GitHub Issues**: Report bugs and request features
- **Workshops**: Regular AROLA architecture training sessions

## License

**MIT License** - Open source software enabling innovation in autonomous vehicle research and education.

See individual package LICENSE files for component-specific licensing details.

---

## About F1TENTH AROLA

The **Autonomous Racing Open Layered Architecture (AROLA)** represents the next generation of modular, scalable software frameworks for autonomous vehicles. Born from competitive F1TENTH racing environments, AROLA enables rapid development and deployment of autonomous systems from research prototypes to production vehicles.

**Building the future of autonomous racing, one layer at a time.**
