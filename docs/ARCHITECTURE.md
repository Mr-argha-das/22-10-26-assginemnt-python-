# System Architecture — `AME-RoboRacer-Project-YourStudentID`

- **Course / Competition:** AME RoboRacer Competition — GIU Berlin (Winter Semester 2026)
- **Author:** `YourName` (`YourStudentID`)
- **Upstream Baseline:** `https://github.com/GIU-F1Tenth/minimal_sofware_example` (`4239040`)

---

## 1. Layered ROS 2 Package Architecture

The project preserves and hardens the 6-package **AROLA (Autonomous Racing Open Layered Architecture)** layout inside a standard `colcon` workspace:

| Layer | Package (`ament_python`) | Workspace Path | Node Executable(s) | Primary Responsibility |
| :--- | :--- | :--- | :--- | :--- |
| **Simulation & Physics** | `f1tenth_gym_ros` | `f1tenth_gym_ros/` | `gym_bridge` | Steps `f110_gym:f110-v0` RK4 single-track dynamics at `100 Hz`, simulates 1080-beam 2D LiDAR `/scan`, publishes `/ego_racecar/odom` & `map -> ego_racecar/base_link` TF at `250 Hz`. |
| **Planning** | `simple_planner` | `src/planning/simple_planner/` | `simple_path_node` | Loads/generates GIU Berlin paths (`csv`, `harmonic_centerline`, `min_curvature`), validates map bounds & vehicle clearance (`>= 0.605 m`), publishes `/pp_path` & `/planned_path_marker`. |
| **Control** | `pure_pursuit` | `src/control/pure_pursuit/` | `pure_pursuit_node` | 100 Hz Ackermann geometric Pure Pursuit with speed-adaptive lookahead, forward curvature preview speed regulation, steering low-pass/rate saturation, and stale-odom / `/pause` fail-safes; **sole publisher on `/drive`**. |
| **Core & Orchestration** | `core` | `src/core/` | `odom_to_base_link_node` | Centralized YAML profiles (`params.yaml`, `berlin_conservative.yaml`, `berlin_race.yaml`, `hardware.params.yaml`), launch orchestration, and conflict-free `odom -> ego_racecar/base_link` TF ownership. |
| **Safety Supervision** | `watchdog` | `src/watchdog/watchdog/` | `watchdog_node` | Periodic `/scan` & `/ego_racecar/odom` liveliness monitoring (plus optional hardware `vesc_msgs` battery/temperature supervision); publishes emergency `Bool(True)` to `/pause` on sensor dropout. |
| **Telemetry & Evaluation** | `race_monitor` | `src/monitor/race_monitor/` | `race_monitor` | Start/finish line crossing detection on GIU Berlin (`[0.80, -17.25] -> [3.80, -17.25]`), lap timing, CSV/JSON telemetry export, and `evo` (`PosePath3D`) APE/RPE error evaluation. |

---

## 2. Node, Topic & Frame Dataflow Diagram

```text
+---------------------------------------------------------------------------------------------------+
|                           GIU BERLIN OCCUPANCY GRID (600x600, 0.05 m/px)                          |
|                    origin = [-11.606540, -26.520793, 0.0], start = (2.2933, -17.2435)             |
+-----------------------------------+-------------------------------------------+-------------------+
                                    |                                           |
                                    v                                           v
                    +-------------------------------+           +-------------------------------+
                    |      simple_planner           |           |   f1tenth_gym_ros             |
                    |     (simple_path_node)        |           |     (gym_bridge)              |
                    |  - Clearance EDT validation   |           |  - RK4 single-track (100 Hz)  |
                    |  - Curvature speed profiling  |           |  - 1080-beam 2D LiDAR         |
                    +---------------+---------------+           +---+-----------+-----------+---+
                                    |                               |           |           |
                /pp_path (nav_msgs/Path, frame="map")               | /scan     | /tf       | /ego_racecar/odom
                pose.orientation.w = v_ref [m/s]                    |           |           | (nav_msgs/Odometry)
                                    |                               |           |           |
                                    v                               v           |           +----------------+
                    +-----------------------------------------------+---+       |           |                |
                    |                 pure_pursuit                      |<------+-----------+                |
                    |               (pure_pursuit_node)                 |                                    |
                    |  - Speed-scaled lookahead Ld(v)                   |<---- /pause (std_msgs/Bool)        |
                    |  - Ackermann steer atan(2*L*sin(alpha)/Ld)        |            ^                       |
                    |  - Curvature preview speed regulation             |            |                       |
                    |  - Rate & angle saturation + stale-odom stop      |    +-------+---------------+       |
                    +-------------------------+-------------------------+    |       watchdog        |<------+
                                              |                              |    (watchdog_node)    |
                              /drive (ackermann_msgs/AckermannDriveStamped)  | - /scan timeout guard |
                              [SINGLE PUBLISHER ACROSS ENTIRE GRAPH]         | - VESC optional guard |
                                              |                              +-----------------------+
                                              +-----------------------------> back to gym_bridge / VESC
                                                                             + race_monitor (/ego_racecar/odom)
```

---

## 3. ROS 2 Topic & Message Contract Table

| Topic | Message Type | Publisher(s) | Subscriber(s) | Rate | Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/pp_path` | `nav_msgs/msg/Path` | `simple_planner` | `pure_pursuit_node`, `race_monitor` | `2 Hz` | `frame_id: "map"`; `pose.orientation.w` carries $v_{\text{ref},i}$ [m/s]. |
| `/planned_path_marker` | `visualization_msgs/msg/Marker` | `simple_planner` | RViz2 | `2 Hz` | Speed-colored `LINE_STRIP` of validated raceline. |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | `gym_bridge` | `pure_pursuit_node`, `watchdog_node`, `race_monitor`, `odom_to_base_link_node` | `250 Hz` | Exact vehicle pose and body twist $(v_x, v_y, \dot{\psi})$. |
| `/scan` | `sensor_msgs/msg/LaserScan` | `gym_bridge` | `watchdog_node`, `amcl` (optional) | `250 Hz` | 1080 beams over $270^\circ$ FOV (`[-2.35, +2.35] rad`). |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | `pure_pursuit_node` (**exclusive**) | `gym_bridge` (sim) / `ackermann_mux` (hardware) | `100 Hz` | Verified single publisher (`len(drive_publishers) == 1`). |
| `/pause` | `std_msgs/msg/Bool` | `watchdog_node` / operator CLI | `pure_pursuit_node` | Event / `2 Hz` | `True` immediately commands `speed = 0.0, steer = 0.0`. |
| `/watchdog/system/status` | `std_msgs/msg/String` | `watchdog_node` | Operator / logging | `1 Hz` | Human-readable health telemetry. |

---

## 4. Strict Single-Parent TF2 Ownership

A critical bug in the unmodified upstream repository (`Bug #4`) was **three simultaneous nodes broadcasting conflicting parent frames for `ego_racecar/base_link`**. Our architecture enforces strict single-parent authority in every mode:

1. **Ground-Truth Simulation (`berlin_full_sim.launch.py`, default `use_amcl:=false`):**
   - `gym_bridge` owns `map -> ego_racecar/base_link -> ego_racecar/laser`.
   - `odom_to_base_link_node` sets `publish_tf: false` (verified `tf_parent_conflicts == []`).
2. **AMCL Localization Mode (`simulation.core.launch.py` / `core.launch.py` with `use_amcl:=true`):**
   - `amcl` owns `map -> odom`.
   - `odom_to_base_link_node` sets `publish_tf: true, publish_map_to_odom: false` and exclusively owns `odom -> ego_racecar/base_link`.

---

## 5. Summary of 10 Upstream Starter Bugs Diagnosed & Fixed

| # | Package / File | Upstream Defect | Fix Implemented |
| :--- | :--- | :--- | :--- |
| 1 | `GIU-F1Tenth/f1tenth_maps` (`berlin.png`) | Stored as 1-bit `mode='1'` PNG; `ScanSimulator2D` reads raw `0/1` instead of `0/255`, treating **100% of pixels as solid wall**. | Converted `berlin.png` & `berlin.pgm` to 8-bit `mode='L'` (`0`/`255`), verified `360,000/360,000` pixel match with `f1tenth_gym`, plus automatic runtime conversion guard in `resolve_gym_map_path()`. |
| 2 | `pure_pursuit_node.py` | Required joystick deadman button in headless sim, used small-angle approximation `kp * gamma` instead of $\arctan(L \gamma)$, asymmetric velocity clamp, stale-odom blindness, and hardcoded `base_link` frame. | Added `autonomous_vel_enabled: true`, exact Ackermann $\arctan(L \gamma)$, symmetric rate & angle clamping, `odom_timeout` fail-safe stop, `/pause` handling, and configurable `ego_racecar/base_link`. |
| 3 | `core/odom_to_base_link_node.py` | Broadcast BOTH `map -> odom` AND `odom -> ego_racecar/base_link` simultaneously while `gym_bridge` broadcast `map -> ego_racecar/base_link`. | Added `publish_tf` & `publish_map_to_odom` flags guaranteeing single-parent TF2 authority in both ground-truth and AMCL modes. |
| 4 | `race_monitor/race_monitor.py` | Hardcoded subscription `'/car_state/odom'` (ignoring YAML parameter `odometry_topics`) and non-Berlin start line `[-2, 2] -> [2, 2]`; hard dependency on `tf_transformations`. | Subscribes to configured `odometry_topics[0]` (`'/ego_racecar/odom'`), sets GIU Berlin start line `[0.80, -17.25] -> [3.80, -17.25]`, and adds clean `transforms3d` fallback. |
| 5 | `watchdog/watchdog_node.py` | Unconditional `from vesc_msgs.msg import VescStateStamped` crashed node startup in simulation without VESC hardware driver. | Wrapped `vesc_msgs` in `try ... except ImportError` (`VESC_AVAILABLE`), added `/ego_racecar/odom` monitoring, and enabled simulation operation. |
| 6 | `watchdog/watchdog_node.py` | Checked LiDAR timeout only inside `_lidar_callback`, so total `/scan` dropout never set `is_critical = True`! | Added `_evaluate_lidar_silence()` inside periodic timer `_publish_critical_status()` and published `Bool(True)` to `/pause`. |
| 7 | `simple_planner` (`package.xml`, `setup.py`, `params.yaml`) | Missing `visualization_msgs` dependency, mismatched YAML node header, unnecessary `pandas` CSV dependency, and non-Berlin default map/path. | Added `visualization_msgs`, pure-Python `csv` parser + occupancy-grid planner/validator, aligned `simple_planner` YAML header, and shipped 4 validated Berlin paths. |
| 8 | `f1tenth_gym_ros` (`setup.py`, `sim.yaml`, `docker-compose.yml`) | Hardcoded host path `/home/username/...`, uninstalled `maps/` share directory, and isolated Docker bridge network. | Installed `maps/berlin.{png,pgm,yaml}` via `setup.py`, added `resolve_gym_map_path()`, `Dockerfile.humble`, and unified `docker-compose.yml`. |
| 9 | `pure_pursuit/setup.py` & `race_monitor/setup.py` | Missing `tests_require=['pytest']` (`0` tests collected under `colcon test`), `numpy<2.0` cap, and `122` `flake8` violations. | Added `tests_require=['pytest']`, built **30 passing automated tests** across all 6 packages, and achieved **0 default-79-col `flake8` errors**. |
| 10 | `Raceline-Optimization` (`traj_race_cl-2025-08-03.csv`) | Exact `0.0000 m` duplicate closing waypoint (`row[0] == row[-1]`) causing degenerate zero-length segment tangents. | Implemented `deduplicate_waypoints()` and validated all 4 shipped Berlin CSV trajectories (`spacing ~ 0.20 m`). |
