# F1TENTH AROLA Core Module

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)

## Overview

The **Core Module** serves as the central orchestration layer of the F1TENTH AROLA (Autonomous Racing Open Layered Architecture). It provides system integration, centralized configuration management, and launch orchestration for all AROLA layers.

As the top-level layer in the AROLA architecture, the Core Module ensures seamless coordination between all subsystems while maintaining a unified configuration approach and proper startup sequencing.



## Core Module Structure

```
core/
├── launch/                    # Launch orchestration
│   ├── core.launch.py        # Main system launch file
│   └── simulation.core.launch.py  # Simulation variant
├── config/                   # Centralized configuration
│   └── params.yaml          # Unified parameter file
├── core/                    # Core package source
│   └── __init__.py
├── resource/               # Package resources
├── test/                   # Unit tests
├── package.xml            # ROS2 package definition
├── setup.py              # Python package setup
├── setup.cfg             # Setup configuration
├── LICENSE               # Package license
└── README.md            # This file
```

## Key Features

### 🚀 **Launch Orchestration**
- **Centralized Launch Management**: Single entry point for entire AROLA system
- **Proper Startup Sequencing**: Ensures correct initialization order across all layers
- **Configuration Flexibility**: Runtime parameter override and environment adaptation
- **Simulation Support**: Dedicated simulation launch configurations

### ⚙️ **Centralized Configuration**
- **Unified Parameter Management**: Single `params.yaml` for all system components
- **Layer-Aware Configuration**: Organized by AROLA architecture layers
- **Runtime Reconfiguration**: Support for dynamic parameter updates
- **Environment Profiles**: Different configurations for various deployment scenarios

### 🔗 **System Integration**
- **Inter-Layer Communication**: Manages topic routing and message flow
- **Lifecycle Management**: Coordinates node activation and deactivation
- **Dependency Resolution**: Ensures proper service dependencies
- **Health Monitoring**: Integration with watchdog layer for system health

## Launch Files

### `core.launch.py` - Main System Launch

The primary launch file that orchestrates the complete F1TENTH AROLA system:

**Key Features:**
- Launches all AROLA layers in proper sequence
- Manages centralized configuration distribution
- Provides runtime parameter override capabilities
- Includes comprehensive system logging

**Launch Arguments:**
```bash
ros2 launch core core.launch.py --help
```

| Argument | Default | Description |
|----------|---------|-------------|
| `config_file` | `config/params.yaml` | Path to configuration file |
| `use_sim_time` | `false` | Enable simulation time |
| `log_level` | `info` | System-wide logging level |

**Usage Examples:**
```bash
# Standard launch
ros2 launch core core.launch.py

# Custom configuration
ros2 launch core core.launch.py config_file:=/path/to/custom.yaml

# Simulation mode
ros2 launch core core.launch.py use_sim_time:=true

# Debug mode
ros2 launch core core.launch.py log_level:=debug
```

### `simulation.core.launch.py` - Simulation Launch

Specialized launch configuration optimized for simulation environments:

**Key Features:**
- Pre-configured for Gazebo/simulation time
- Adjusted parameters for simulation performance
- Mock hardware interfaces where appropriate
- Development-friendly logging levels

## Configuration Management

### Centralized Parameters (`config/params.yaml`)

The Core Module maintains a unified configuration file that serves all AROLA layers:

```yaml
# ==============================================================================
# CORE LAYER CONFIGURATION
# ==============================================================================
core:
  use_sim_time: false
  robot_base_frame: "base_link"
  global_frame: "map"
  system_frequency: 50.0

# ==============================================================================
# CONTROL LAYER CONFIGURATION  
# ==============================================================================
pure_pursuit:
  lookahead_distance: 1.5
  max_velocity: 3.0
  control_frequency: 20.0
  waypoint_tolerance: 0.5

# ==============================================================================
# LOCALIZATION LAYER CONFIGURATION
# ==============================================================================
nav2_amcl:
  max_particles: 2000
  min_particles: 500
  recovery_alpha_slow: 0.001
  recovery_alpha_fast: 0.1
  laser_max_range: 12.0

# ==============================================================================
# PLANNING LAYER CONFIGURATION
# ==============================================================================
simple_planner:
  csv_path: "/path/to/waypoints.csv"
  frame_id: "map"
  publish_rate: 10.0
  path_timeout: 5.0

# ==============================================================================
# DRIVERS LAYER CONFIGURATION
# ==============================================================================
urg_node2:
  ip_address: "192.168.1.10"
  port: 10940
  frame_id: "laser"
  scan_frequency: 40.0
  angle_min: -2.094
  angle_max: 2.094

# ==============================================================================
# WATCHDOG LAYER CONFIGURATION
# ==============================================================================
watchdog:
  monitor_frequency: 10.0
  timeout_threshold: 2.0
  critical_topics: 
    - "/scan"
    - "/odom" 
    - "/drive"
  system_health_topic: "/watchdog/system/status"
```

### Configuration Philosophy

The Core Module implements **Configuration as Code** principles:

1. **Single Source of Truth**: One configuration file for entire system
2. **Layer Organization**: Clear separation by APEX architecture layers
3. **Environment Agnostic**: Same config works across different environments
4. **Version Controlled**: Configuration changes tracked with code
5. **Validation**: Built-in parameter validation and defaults

## System Startup Sequence

The Core Module orchestrates a carefully designed startup sequence:

```
┌─────────────────────────────────────────────────────────────┐
│                   AROLA STARTUP SEQUENCE                   │
└─────────────────────────────────────────────────────────────┘

1. 🔧 CORE INITIALIZATION
   ├── Load centralized configuration
   ├── Initialize logging system  
   ├── Validate system dependencies
   └── Setup inter-layer communication

2. 🔌 DRIVERS LAYER STARTUP
   ├── URG Node2 (LiDAR Driver)
   ├── Hardware interface initialization
   └── Sensor calibration and validation

3. 📍 LOCALIZATION LAYER STARTUP  
   ├── Nav2 AMCL (Localization)
   ├── Particle filter initialization
   └── Map and transform setup

4. 🗺️ PLANNING LAYER STARTUP
   ├── Simple Planner (Path Planning)
   ├── Waypoint loading and validation
   └── Path publishing initialization

5. 🎮 CONTROL LAYER STARTUP
   ├── Pure Pursuit Controller
   ├── Control loop initialization
   └── Safety limit configuration

6. 🛡️ WATCHDOG LAYER STARTUP
   ├── System Watchdog
   ├── Health monitoring activation
   └── Safety system validation

7. ✅ SYSTEM READY
   └── All layers operational and monitored
```

## Development Guide

### Adding New Components

To integrate a new component into the AROLA system via the Core Module:

1. **Update Configuration**
   ```yaml
   # Add to config/params.yaml
   new_component:
     parameter1: value1
     parameter2: value2
   ```

2. **Modify Launch File**
   ```python
   # Add to core.launch.py
   new_component_node = Node(
       package='new_package',
       executable='new_executable',
       name='new_component',
       parameters=[config_file_path],
       # ... other configuration
   )
   ```

3. **Update Dependencies**
   ```xml
   <!-- Add to package.xml -->
   <depend>new_package</depend>
   ```

### Testing Core Module

```bash
# Unit tests
colcon test --packages-select core

# Integration tests
ros2 launch core core.launch.py
ros2 topic list  # Verify all expected topics are active
ros2 node list   # Verify all nodes are running

# Configuration validation
ros2 param list  # Check all parameters are loaded correctly
```

## Monitoring & Debugging

### Core System Health

```bash
# Check all AROLA nodes
ros2 node list | grep -E "(pure_pursuit|amcl|simple_planner|urg_node|watchdog)"

# Monitor system startup
ros2 launch core core.launch.py log_level:=debug

# Configuration debugging  
ros2 param dump /pure_pursuit_node
ros2 param dump /amcl
```

### Performance Monitoring

```bash
# System resource usage
top | grep -E "(pure_pursuit|amcl|simple_planner)"

# Network traffic
ros2 topic bw /scan /odom /pp_path

# System latency
ros2 topic hz /scan /odom /drive
```

## Integration Points

### Upstream Dependencies
- **ROS2 Core**: Base ROS2 functionality
- **Launch System**: ROS2 launch framework
- **Parameter Server**: ROS2 parameter management

### Downstream Dependencies
- **Control Layer**: Provides configuration and orchestration
- **Localization Layer**: Manages Nav2 stack integration
- **Planning Layer**: Coordinates path planning systems
- **Drivers Layer**: Manages hardware abstraction
- **Watchdog Layer**: Integrates safety monitoring

## Best Practices

### 🎯 **Configuration Management**
- Keep all parameters in centralized `params.yaml`
- Use descriptive parameter names with layer prefixes
- Provide sensible defaults for all parameters
- Document parameter units and valid ranges

### 🚀 **Launch Orchestration**
- Maintain proper startup sequence dependencies
- Use descriptive node names and namespaces
- Include comprehensive logging for debugging
- Provide flexible runtime configuration options

### 🔧 **System Integration**
- Ensure clean separation between layers
- Use standard ROS2 interfaces between components
- Implement proper error handling and recovery
- Monitor system health continuously

## Contributing

When contributing to the Core Module:

1. **Maintain Layer Separation**: Keep clear boundaries between AROLA layers
2. **Configuration First**: Add all new parameters to centralized config
3. **Documentation**: Update README and inline documentation
4. **Testing**: Ensure integration tests pass with new changes
5. **Backwards Compatibility**: Maintain compatibility with existing deployments

## License

MIT License - See [LICENSE](LICENSE) file for details.

---

**The Core Module - Orchestrating Excellence in Autonomous Racing**

*Part of the F1TENTH AROLA Architecture ecosystem*
