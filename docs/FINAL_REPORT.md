# Technical Final Report: Autonomous Path Planning and Pure Pursuit Tracking on the GIU Berlin RoboRacer Circuit

- **Project Title:** `AME-RoboRacer-Project-YourStudentID`
- **Author:** `YourName` (`YourStudentID`)
- **Programme / Module:** Autonomous Mobile Engineering — AME RoboRacer (F1TENTH) Competition, Winter Semester 2026
- **Institution:** German International University (GIU), Berlin, Germany
- **Starter Repository:** [`GIU-F1Tenth/minimal_sofware_example`](https://github.com/GIU-F1Tenth/minimal_sofware_example) (`commit 4239040`)

---

## Abstract

This report presents the design, defect remediation, mathematical formulation, ROS 2 integration, and closed-loop dynamic validation of a complete autonomous racing software stack for the GIU Berlin RoboRacer competition track. Building strictly on the mandatory AROLA starter repository (`GIU-F1Tenth/minimal_sofware_example`) and its four ROS 2 submodules (`pure_pursuit`, `race_monitor`, `watchdog`, and `f1tenth_gym_ros`), ten critical upstream defects—including a 1-bit occupancy-grid encoding bug that caused 100% immediate spawn collisions in `f110_gym`, conflicting multi-parent TF2 broadcasters, a joystick-deadlock guard blocking headless autonomy, an undetected total-silence LiDAR failure mode in `watchdog_node`, and missing test configurations—were systematically diagnosed and resolved. We implement a two-stage map-based path planner combining Laplace harmonic boundary potentials with Euclidean-distance-constrained minimum-curvature smoothing and quasi-steady-state forward-backward velocity profiling across all `107,954` drivable cells (`269.885 m^2`) of the `600 x 600` GIU Berlin occupancy grid. On the control side, we upgrade the Pure Pursuit tracker with exact Ackermann steering kinematics $\delta = \arctan(L \gamma)$, speed-adaptive lookahead scaling, forward curvature preview speed regulation, servo rate and angle saturation, and multi-layered `/pause` and stale-odometry fail-safes. Closed-loop evaluations in `f110_gym` (RK4 single-track dynamics at `100 Hz` with 1080-beam 2D LiDAR and GJK footprint collision checking) demonstrate **100% zero-collision, zero-departure completion** across 5-lap endurance benchmarks in Conservative (`40.695 s`, `4.84 cm` CTE RMSE), Balanced (`21.405 s`, `7.89 cm` CTE RMSE), and Fast-Race (`16.176 s`, `9.23 cm` CTE RMSE) profiles—as well as `14.976 s` in high-aggression tuning and **`15 / 15` (`100.0%`) success** under random initial pose perturbations—backed by **30 passing automated unit, lint, rosbag2, and closed-loop integration tests**.

---

## 1. Introduction & Competition Scope

High-speed autonomous racing on 1:10-scale Ackermann vehicles (RoboRacer / F1TENTH) demands tight coordination between occupancy-grid trajectory planning, nonlinear lateral-longitudinal vehicle control, and deterministic fault supervision. In narrow indoor SLAM circuits such as the GIU Berlin track—where corridor widths narrow to `1.97 m` through compound chicanes—small errors in coordinate conventions, lookahead selection, or corner-entry braking rapidly lead to lateral tire saturation or wall impact.

In accordance with the Winter Semester 2026 competition specification:
1. **Strict Starter Provenance:** All software evolves directly from `GIU-F1Tenth/minimal_sofware_example` without replacing its 6-package ROS 2 architecture.
2. **Software & Simulation Scope:** Physical chassis assembly and physical vehicle track testing are outside scope; engineering effort focuses on reproducible ROS 2 Humble software, authentic closed-loop GIU Berlin simulation, repeatable parameter tuning, automated verification, and explicit sim-to-real transfer readiness.

---

## 2. Starter Repository Audit & System Architecture

### 2.1 Audit of Unmodified Upstream Baseline
Before modifying a single line of code, the upstream workspace was built and benchmarked (`commit b992c14`). While `colcon build` compiled all 6 `ament_python` packages in `3.37 s`, `colcon test` failed on 4 of 6 packages (`12 errors`, `0` functional unit tests executed), default `flake8` flagged `122` style violations, and closed-loop execution on GIU Berlin failed immediately due to **ten distinct defects** documented in [`docs/ARCHITECTURE.md`](ARCHITECTURE.md):
- **Map & Simulator Defects:** `GIU-F1Tenth/f1tenth_maps` stores `berlin.png` as a 1-bit `PIL` image (`mode='1'`), which `ScanSimulator2D` casts directly to `{0.0, 1.0}` instead of `{0.0, 255.0}`, classifying every pixel on the track as occupied wall; `f1tenth_gym_ros/setup.py` also omitted installing `maps/`, and `sim.yaml` hardcoded `/home/username/...`.
- **Control & Autonomy Defects:** `pure_pursuit_node.py` gated all motion behind physical `/joy` button presses, approximated steering via scalar multiplication instead of $\arctan(L \gamma)$, applied an asymmetric velocity clamp, lacked stale-odometry timeout protection, and referenced `"base_link"` rather than `"ego_racecar/base_link"`.
- **TF2, Watchdog & Monitor Defects:** `gym_bridge` and `odom_to_base_link_node` published conflicting parent transforms for `ego_racecar/base_link`; `watchdog_node.py` crashed without hardware `vesc_msgs` and checked LiDAR age only inside `_lidar_callback` (missing complete sensor silence); `race_monitor.py` subscribed to hardcoded `'/car_state/odom'` instead of `'/ego_racecar/odom'`.

### 2.2 Hardened AROLA Dataflow & Single-Authority Invariants
The integrated ROS 2 graph enforces two safety invariants verified continuously in `test_integration_and_configs.py`:
1. **Single `/drive` Publisher Invariant:** Only `pure_pursuit_node` publishes `ackermann_msgs/AckermannDriveStamped` on `/drive` (`100 Hz`), preventing actuator command contention.
2. **Single-Parent TF2 Tree Invariant:** In ground-truth simulation (`berlin_full_sim.launch.py`), `gym_bridge` exclusively broadcasts `map -> ego_racecar/base_link -> ego_racecar/laser` (`publish_tf: false` on `odom_to_base_link_node`). When `amcl` localization is enabled (`use_amcl:=true`), `amcl` broadcasts `map -> odom` while `odom_to_base_link_node` broadcasts `odom -> ego_racecar/base_link` (`tf_parent_conflicts == []`).

---

## 3. GIU Berlin Map Verification & Path Planning

### 3.1 Exact Occupancy-Grid Geometry
Inspection of `f1tenth_gym_ros/maps/berlin.yaml` and pixel-level comparison across `f1tenth/f1tenth_gym`, `GIU-F1Tenth/f1tenth_maps` (converted to 8-bit `L`), and `GIU-F1Tenth/racing_playground` confirmed exact `360,000 / 360,000` cell identity:
- **Dimensions & Scale:** $600 \times 600$ cells at resolution $r = 0.05\text{ m/px}$ ($30.0\text{ m} \times 30.0\text{ m}$), origin $(-11.606540, -26.520793, 0.0)$.
- **Topology:** `107,954` free pixels (`269.885 m^2`) bounded by exactly **2 occupied wall components** (outer boundary `227,977 px`, inner island `24,069 px`, `0` interior obstacles), with corridor half-widths spanning `[0.985 m, 2.680 m]`.
- **Spawn & Finish Geometry:** Spawn pose $(x_0, y_0, \theta_0) = (2.2933\text{ m}, -17.2435\text{ m}, -1.4965\text{ rad})$ sits centered on the western straight (`1.00 m` wall clearance), spanned by start/finish line $[0.80, -17.25] \rightarrow [3.80, -17.25]\text{ m}$.

### 3.2 Harmonic Centerline, Minimum-Curvature Smoothing & Speed Profiling
Implemented in `simple_planner` (`map_utils.py`, `path_planner.py`, `path_validator.py`):
1. **Topological Initialization (`harmonic_centerline`):** Solving $\nabla^2 \phi = 0$ on $\Omega_{\text{free}}$ with Dirichlet conditions $\phi_{\text{inner}} = 0, \phi_{\text{outer}} = 1$ yields a guaranteed singularity-free closed level set $\phi = 0.50$ through all chicanes.
2. **Clearance-Constrained Smoothing (`min_curvature`):** Discrete fourth-difference smoothing minimizes peak curvature $|\kappa_i|$ (reducing max curvature from `2.578 m^-1` on the raw harmonic centerline down to **`0.995 m^-1`** on `berlin_apex_raceline.csv`) subject to the Euclidean distance barrier:
   $$d_{\text{wall}}(x_i, y_i) \ge \frac{W_{\text{car}}}{2} + m_{\text{safe}} = 0.155 + 0.450 = 0.605\text{ m}$$
   Every generated path achieves **`>= 0.948 m` minimum center wall clearance** (`>= 0.79 m` body edge clearance at rest).
3. **Quasi-Steady-State Speed Profile:** Forward-backward longitudinal passes enforce $v_i \le \sqrt{a_{y,\max} / |\kappa_i|}$, $v_{i+1}^2 - v_i^2 \le 2 a_{x,\text{acc}}\Delta s_i$, and $v_i^2 - v_{i+1}^2 \le 2 a_{x,\text{dec}}\Delta s_i$ across cyclic index wrap-around.

---

## 4. Pure Pursuit Controller Upgrade & Safety Supervision

Implemented in `pure_pursuit/controller_core.py` and `pure_pursuit/pure_pursuit_node.py` at `100 Hz`:
1. **Exact Ackermann Geometry:** Transforming the speed-adaptive lookahead point $(x_{\text{la}}, y_{\text{la}})$ into the vehicle frame gives lateral offset $y_v$ and curvature $\gamma = 2 y_v / L_d^2$, mapped through exact bicycle kinematics $\delta_{\text{geom}} = \arctan(L \gamma)$ with $L = 0.3302\text{ m}$.
2. **Damping & Physical Servo Constraints:** Derivative curvature damping ($k_d \dot{\gamma}$), first-order low-pass filtering ($\alpha_{\text{lp}} \in [0.72, 0.76]$), servo rate limiting ($|\dot{\delta}| \le 3.2\text{ rad/s}$), and symmetric angle clamping ($|\delta| \le 0.4189\text{ rad}$) eliminate high-speed chatter on straights.
3. **Forward Curvature Preview Speed Regulation:** Scanning peak path curvature $\kappa_{\text{preview}}$ ahead over $S_{\text{prev}} \in [2.4, 3.1]\text{ m}$ enforces $v_{\text{cmd}} \le \sqrt{a_{y,\max}/\max(|\gamma|, \kappa_{\text{preview}})}$ **prior** to hairpin entry.
4. **Defense-in-Depth Safety:** Immediate `(0.0, 0.0)` stop commands trigger upon `/pause == True`, odometry age exceeding `odom_timeout` (`0.5 s`), invalid/short paths (`< 3` waypoints), non-finite poses, or periodic LiDAR silence detected by `watchdog_node` (`age > 1.0 s`).

---

## 5. Closed-Loop Simulation Methodology & Experimental Results

Closed-loop experiments execute the actual ROS 2 nodes (`GymBridge`, `SimplePathNode`, `OdomToBaseLinkNode`, `PurePursuit`, `WatchdogNode`, and `LapDetector`) against `f110_gym:f110-v0` single-track RK4 physics (`m = 3.74 kg`, $\mu = 1.0489$, $C_{Sf} = 4.718$, $C_{Sr} = 5.4562$, `dt = 0.01 s`). Across **all 5-lap profile endurance tests**, the vehicle achieved **100% completion with zero wall collisions and zero track departures**:

| Profile | Laps | Standing Lap 1 [s] | Flying Mean $\pm$ Std [s] | Best Flying Lap [s] | Min Body Clearance [m] | CTE RMSE [cm] | EVO APE RMSE [cm] | Steer Rate RMS [rad/s] | Mean / Max Speed [m/s] |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **`Conservative`** | `5 / 5` | `41.978` | `40.700 +/- 0.005` | **`40.695`** | **`0.816`** | **`4.84`** | `7.57` | **`0.218`** | `1.69 / 2.68` |
| **`Balanced`** | `5 / 5` | `23.212` | `21.408 +/- 0.003` | **`21.405`** | **`0.794`** | **`7.89`** | `9.83` | **`0.236`** | `3.18 / 4.80` |
| **`Fast-Race`** | `5 / 5` | `17.827` | `16.190 +/- 0.011` | **`16.176`** | **`0.774`** | **`9.23`** | `10.90` | **`0.315`** | `4.18 / 6.47` |

- **Path Mode Comparison (`params.yaml` controller):** Switching from `berlin_safe_centerline.csv` (`33.463 s`, `6.49 cm` CTE) to `berlin_optimal_raceline.csv` (`21.408 s`), `berlin_apex_raceline.csv` (`19.568 s`), and `berlin_giu_reference.csv` (`19.316 s`) reduced lap times by **up to 42.3%** solely through curvature reduction while keeping body wall clearance `>= 0.70 m`.
- **Monte Carlo Perturbed Spawn Campaign (`N = 15`):** Random perturbations $\Delta x \in [-0.28, +0.28]\text{ m}$, $\Delta y \in [-0.25, +0.25]\text{ m}$, $\Delta \psi \in [-12^\circ, +12^\circ]$ achieved **`15 / 15` (`100.0%`) zero-collision completion**, converging from varied Lap 1 recovery times (`23.14–23.26 s`) to **`21.408 +/- 0.004 s`** on Flying Lap 2.
- **Fault Injection Response:** Publishing `/pause = True` at `3.237 m/s` dropped `/drive` to `0.000 m/s` within `10 ms` and brought the vehicle to rest in **`0.390 s` (`0.559 m`)**; simulated `/scan` failure triggered `watchdog_node.is_critical = True` and an automatic `/pause` stop.

---

## 6. Parameter Tuning Trade-Offs

Our controlled 6-configuration tuning sweep (`results/plots/parameter_tuning_comparison.png`) empirically confirmed two fundamental Pure Pursuit trade-offs:
1. **Lookahead Instability vs. Corner-Cutting:** Overly short lookahead (`Tune-ShortLD`, $L_d \in [0.35, 0.85]\text{ m}$) achieved tight `5.98 cm` CTE RMSE but increased steering rate chatter **3.2x** (`0.677 rad/s` vs. `0.210 rad/s`). Conversely, excessive lookahead (`Tune-LongLD`, $L_d \in [1.35, 2.50]\text{ m}$) smoothed steering to `0.210 rad/s` but cut chicane apexes (`16.13 cm` CTE RMSE, reducing minimum wall clearance by `16.0 cm`).
2. **Speed Scaling vs. Tire Friction Margin:** Pairing moderate speed-adaptive lookahead (`[0.72, 1.85] m`) with minimum-curvature `berlin_apex_raceline.csv` pushed clean zero-collision flying laps to **`16.176 s`** (`Fast-Race`, peak `6.47 m/s`) and **`14.976 s`** (`Tune-UltraAggr`, peak `7.30 m/s`). Because `Tune-UltraAggr` peaks at `14.47 m/s^2` transient yaw-rate acceleration during quick chicane transitions, `Tune-OptimalRace` (`16.18 s`) was selected as the shipped `berlin_race.yaml` profile for stronger stability margin.

---

## 7. Limitations & Physical RoboRacer Hardware Transfer Considerations

While the software stack is fully verified in `f110_gym` and ships a dedicated physical configuration (`src/core/config/hardware.params.yaml` + `src/core/launch/core.launch.py`), three sim-to-real factors govern deployment on physical GIU RoboRacer vehicles:
1. **Localization Noise & Latency (`AMCL` / Particle Filter vs. Sim Ground Truth):** Physical Hokuyo UST-10LX / RPLiDAR scans exhibit range noise, glass reflections, and `15–30 ms` pose estimation lag. On hardware, `hardware.params.yaml` enables `odom_to_base_link_node` (`publish_tf: true, publish_map_to_odom: false`) alongside `amcl`, increases `min_lookahead_distance` to `0.95 m`, and sets `vel_division_factor: 1.5` during initial track validation.
2. **Tire-Surface Friction Coefficient ($\mu$):** Painted concrete or polished lab floors at GIU Berlin typically yield $\mu \approx 0.65\text{–}0.85$ (lower than `f110_gym`'s default $\mu = 1.0489$). Teams should begin physical runs with `berlin_conservative.yaml` (`a_lat_max = 2.6 m/s^2`) and increment `a_lat_max` in `0.5 m/s^2` steps while monitoring VESC slip current.
3. **VESC Servo Offset & Battery Voltage Sag:** Physical steering linkages require calibrating `steering_angle_to_servo_offset` in `vesc.yaml` on a straight line before enabling autonomy, and installing `vesc_msgs` automatically activates `watchdog_node`'s live `12.0 V` battery cutoff and `80 C` motor thermal protection without code changes.

---

## 8. Conclusion & References

Starting from `GIU-F1Tenth/minimal_sofware_example`, this work delivered a defect-free, PEP 8 compliant, thoroughly tested ROS 2 autonomous racing stack for the GIU Berlin circuit—achieving **`30 / 30` automated test passes**, **`0` collisions across all multi-lap benchmarks**, and a **2.52x clean lap-time progression** from safe baseline (`40.70 s`) to race trim (`16.18 s`).

### References
1. Coulter, R. C. (1992). *Implementation of the Pure Pursuit Path Tracking Algorithm*. Carnegie Mellon University Robotics Institute, CMU-RI-TR-92-01.
2. O'Kelly, M., Zheng, H., Karthik, D., & Mangharam, R. (2020). *F1TENTH: An Open-Source Evaluation Environment for Continuous Control and Reinforcement Learning*. NeurIPS Competition Track (`f110_gym`).
3. Heilmeier, A., Wischnewski, A., Hermansdorfer, L., Betz, J., Lienkamp, M., & Lohmann, B. (2020). *Minimum Curvature Trajectory Planning and Control for an Autonomous Race Car*. Vehicle System Dynamics, 58(10), 1497–1527.
4. GIU-F1Tenth Autonomous Racing Group (2025–2026). *AROLA Minimal Software Example, Berlin Occupancy Maps, and Raceline Optimization Repositories*. `https://github.com/GIU-F1Tenth`.
