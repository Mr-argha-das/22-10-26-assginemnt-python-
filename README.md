# AME-RoboRacer-Project-YourStudentID

**Autonomous Path Planning, Pure Pursuit Tracking Control & Closed-Loop Simulation on the Official GIU Berlin Track**

- **Competition / Course:** AME RoboRacer (F1TENTH) Competition — German International University (GIU) Berlin, Winter Semester 2026
- **Author:** `YourName` | **Student ID:** `YourStudentID`
- **Mandatory Starter Baseline:** [`GIU-F1Tenth/minimal_sofware_example`](https://github.com/GIU-F1Tenth/minimal_sofware_example) (`commit 4239040`, AROLA Architecture — see [`UPSTREAM_ATTRIBUTION.md`](UPSTREAM_ATTRIBUTION.md))
- **Scope:** Full ROS 2 software stack, automated test suite, and genuine closed-loop multi-lap simulation on the official `600 x 600` GIU Berlin occupancy grid (`f110_gym` single-track RK4 physics + 1080-beam 2D LiDAR). Physical chassis assembly and physical vehicle track testing are out of scope (hardware-transfer configuration and considerations are provided in `src/core/config/hardware.params.yaml` and Section 7 of [`docs/FINAL_REPORT.md`](docs/FINAL_REPORT.md)).

---

## Key Verified Results Summary

| Metric / Deliverable | Unmodified Upstream Baseline (`4239040`) | Completed Project (`AME-RoboRacer-Project-YourStudentID`) |
| :--- | :--- | :--- |
| **`colcon build` (6 packages)** | `6 / 6` built (`3.37 s`) | **`6 / 6` built cleanly (`3.38 s`)** |
| **`colcon test` & `pytest`** | `4 failed` packages, `12 errors`, `0` functional tests | **`30 / 30 passed`, `0 errors`, `0 failures`, `0 skipped`** |
| **Strict PEP 8 (`flake8`, 79 cols)** | `122` style/lint violations | **`0` warnings / `0` errors across all packages & tools** |
| **Berlin Map Loading (`600 x 600`)** | Broken (`1-bit PNG` -> 100% solid wall collision) | **Triple-verified `8-bit` map (`269.885 m^2` free, `0` collisions)** |
| **5-Lap Closed-Loop Completion** | Fails to drive headlessly / TF conflicts | **`5 / 5` laps across Conservative, Balanced & Fast-Race (`0` collisions)** |
| **Best Flying Lap Time (Berlin)** | N/A | **`16.176 s`** (`berlin_race.yaml`) / **`14.976 s`** (`Tune-UltraAggr`) |
| **Cross-Track Error (CTE RMSE)** | N/A | **`4.84 cm`** (`Conservative`), **`7.89 cm`** (`Balanced`), **`9.23 cm`** (`Fast-Race`) |
| **Perturbed Spawn Reliability** | N/A | **`15 / 15` (`100.0%`) zero-collision completion across random pose offsets** |

![GIU Berlin Clearance Field and Driven Speed Profile](results/plots/berlin_map_and_paths.png)

---

## Repository Structure

```text
AME-RoboRacer-Project-YourStudentID/
├── src/
│   ├── core/                         # Central YAML profiles, single-owner TF node & full launch files
│   │   ├── config/
│   │   │   ├── berlin_conservative.yaml  # Safe baseline profile (40.70 s/lap, 4.84 cm CTE RMSE)
│   │   │   ├── params.yaml               # Default balanced profile (21.41 s/lap, 7.89 cm CTE RMSE)
│   │   │   ├── berlin_race.yaml          # Tuned fast race profile (16.18 s/lap, 9.23 cm CTE RMSE)
│   │   │   └── hardware.params.yaml      # Physical RoboRacer / VESC hardware transfer profile
│   │   └── launch/
│   │       ├── berlin_full_sim.launch.py # One-command GIU Berlin full-stack simulation launch
│   │       ├── simulation.core.launch.py # AROLA core simulation stack launch
│   │       └── core.launch.py            # Physical robot AROLA core launch
│   ├── planning/simple_planner/      # Occupancy-grid EDT validator, harmonic/min-curv planner & 4 Berlin CSVs
│   ├── control/pure_pursuit/         # Exact Ackermann Pure Pursuit, preview speed control & fail-safes
│   ├── watchdog/watchdog/            # Periodic /scan & /ego_racecar/odom health monitor -> /pause stop
│   └── monitor/race_monitor/         # Berlin finish-line lap timer + EVO PosePath3D APE/RPE analyzer
├── f1tenth_gym_ros/                  # F1TENTH OpenAI Gym ROS 2 bridge, 8-bit Berlin map & Dockerfile.humble
├── tools/
│   ├── ros2_sim_harness/             # Deterministic in-process ROS 2 shim when /opt/ros/humble is absent
│   └── run_berlin_simulation.py      # Reproducible closed-loop benchmark, tuning, rosbag2 & plot runner
├── results/                          # Genuine closed-loop CSV telemetry, JSON summary, .db3 bag & PNG plots
├── docs/                             # Complete engineering documentation & ~4.5-page Final Report
│   ├── FINAL_REPORT.md
│   ├── ARCHITECTURE.md
│   ├── PATH_PLANNING.md
│   ├── CONTROLLER.md
│   ├── SIMULATION_RESULTS.md
│   └── MILESTONES.md
└── UPSTREAM_ATTRIBUTION.md           # Full upstream GIU-F1Tenth provenance and MIT license attribution
```

---

## Reproducible Setup & Execution Instructions

### Option A — Full Native Ubuntu 22.04 LTS + ROS 2 Humble Workflow (Target Competition Environment)

```bash
# 1. Install ROS 2 Humble & F1TENTH dependencies on Ubuntu 22.04
sudo apt update && sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-pip \
  ros-humble-ackermann-msgs ros-humble-nav2-map-server \
  ros-humble-nav2-amcl ros-humble-nav2-lifecycle-manager \
  ros-humble-xacro ros-humble-rviz2

pip install -r requirements.txt
pip install f110-gym==0.2.1 evo rosbags pytest flake8

# 2. Build all 6 ROS 2 packages cleanly
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 3. Run all 30 automated unit, lint & closed-loop integration tests
colcon test
colcon test-result --verbose

# 4. Launch complete closed-loop autonomous driving on GIU Berlin!
#    Choose profile:=conservative | balanced | race
ros2 launch core berlin_full_sim.launch.py profile:=balanced

# Optional emergency stop / resume from another terminal:
ros2 topic pub --once /pause std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /pause std_msgs/msg/Bool "{data: false}"
```

### Option B — Browser NoVNC Docker Simulation (`f1tenth_gym_ros/docker-compose.yml`)

```bash
cd f1tenth_gym_ros
docker compose up --build -d
# Open http://localhost:8080/vnc.html in your browser, then inside container:
docker compose exec sim bash -c "
  cd /arola_ws && source /opt/ros/humble/setup.bash &&
  colcon build && source install/setup.bash &&
  ros2 launch core berlin_full_sim.launch.py profile:=race
"
```

### Option C — Headless Python 3.10–3.12 Container / CI Execution (No Root or `/opt/ros/humble` Needed)

When `/opt/ros/humble` is not installed on the host OS (e.g., Debian 12 / CI sandboxes), `tools/ros2_sim_harness` provides an automatic zero-overhead deterministic `rclpy` + `tf2_ros` runtime executing the **exact ROS 2 node classes** directly against `f110_gym`:

```bash
# 1. Build all 6 ROS 2 packages & execute all 30 automated tests
colcon build
colcon test && colcon test-result --verbose
pytest -v
flake8 f1tenth_gym_ros src/core src/planning/simple_planner \
  src/control/pure_pursuit src/watchdog/watchdog tools

# 2. Re-run the complete 5-lap profile benchmark, 4 path modes,
#    6-point tuning sweep, 15-run Monte Carlo test, rosbag2 & plots:
python3 tools/run_berlin_simulation.py
```

---

## Documentation Index

| Document | Contents |
| :--- | :--- |
| [**`docs/FINAL_REPORT.md`**](docs/FINAL_REPORT.md) | Complete competition technical report (~4.5 pages): architecture, math, tuning, results, limitations & sim-to-real transfer. |
| [**`docs/ARCHITECTURE.md`**](docs/ARCHITECTURE.md) | ROS 2 package graph, topic contracts, single-owner TF2 design, and all **10 upstream bugs fixed**. |
| [**`docs/PATH_PLANNING.md`**](docs/PATH_PLANNING.md) | Ground-truth `600 x 600` GIU Berlin occupancy analysis, harmonic + minimum-curvature planner math & 4 validated paths. |
| [**`docs/CONTROLLER.md`**](docs/CONTROLLER.md) | Exact Ackermann Pure Pursuit equations, forward curvature preview speed regulation & fail-safe state transitions. |
| [**`docs/SIMULATION_RESULTS.md`**](docs/SIMULATION_RESULTS.md) | Full multi-lap tables, parameter tuning Pareto trade-offs, Monte Carlo spawn robustness & fault-injection metrics. |
| [**`docs/MILESTONES.md`**](docs/MILESTONES.md) | Chronological Phase 1–9 engineering execution log and verified Git commits. |
| [**`UPSTREAM_ATTRIBUTION.md`**](UPSTREAM_ATTRIBUTION.md) | Full attribution to `GIU-F1Tenth/minimal_sofware_example` and upstream MIT submodules. |

---

## Troubleshooting Common Issues

| Symptom | Root Cause | Solution Implemented |
| :--- | :--- | :--- |
| Immediate wall collision at spawn `(2.2933, -17.2435)` | `berlin.png` saved as 1-bit `mode='1'` in upstream `f1tenth_maps` | Converted all Berlin map images to 8-bit `mode='L'` + auto-conversion guard in `resolve_gym_map_path()`. |
| Vehicle sits stationary with `0.0 m/s` on `/drive` | Upstream `pure_pursuit_node` waited for physical `/joy` button press | Set `autonomous_vel_enabled: true` in simulation profiles; `/pause` provides clean software e-stop. |
| TF tree jump / warning on `ego_racecar/base_link` | Both `gym_bridge` and `odom_to_base_link_node` published parent TFs | Added `publish_tf: false` default in ground-truth sim (`tf_parent_conflicts == []`). |
| `ModuleNotFoundError: vesc_msgs` on `watchdog_node` | Physical VESC driver absent in pure simulation containers | Graceful `VESC_AVAILABLE` import guard while keeping active `/scan` & `/ego_racecar/odom` monitoring. |
| `race_monitor` shows `0` laps completed | Subscribed to hardcoded `'/car_state/odom'` & non-Berlin finish line | Connected to `odometry_topics[0]` (`'/ego_racecar/odom'`) and Berlin line `[0.80, -17.25] -> [3.80, -17.25]`. |
