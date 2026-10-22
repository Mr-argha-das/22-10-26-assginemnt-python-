# Engineering Milestones & Phased Progress Log — `AME-RoboRacer-Project-YourStudentID`

- **Course / Competition:** AME RoboRacer Competition — GIU Berlin (Winter Semester 2026)
- **Author:** `YourName` (`YourStudentID`)
- **Active Branch:** `arena/cf153760-22-10-26-assginemnt-python`

---

## Phased Execution Trace & Verified Commits

| Phase | Milestone Title | Key Engineering Deliverables | Verification Status & Git Commit |
| :---: | :--- | :--- | :--- |
| **1** | **Repository & Ecosystem Inspection** | Inspected `GIU-F1Tenth/minimal_sofware_example` (`4239040`), all submodules (`pure_pursuit`, `race_monitor`, `watchdog`, `f1tenth_gym_ros`), sibling GIU Berlin repos (`f1tenth_maps`, `racing_playground`, `Raceline-Optimization`), and catalogued **10 upstream functional/build bugs**. | Completed & documented (`b992c14`) |
| **2** | **Baseline Build & Validation** | Merged `upstream/main`, vendored all 4 ROS 2 submodules cleanly with `.gitmodules.upstream`, ran unmodified baseline (`colcon build` 6/6 in `3.37s`, `colcon test` 4 failed / 12 errors / 0 functional tests, `flake8` 122 errors), and documented host environment (`Debian 12`, Python 3.11.2, `f110_gym` 0.2.1). | Baseline ground truth captured (`b992c14`) |
| **3** | **GIU Berlin Map Loading & Simulator Fix** | Cross-verified exact `360,000 / 360,000` pixel match across all 3 Berlin map sources (`600x600`, `0.05 m/px`, `origin = [-11.606540, -26.520793, 0.0]`), fixed the 1-bit `berlin.png` wall-collision bug in `GIU-F1Tenth/f1tenth_maps`, installed `berlin.{png,pgm,yaml}` + `berlin_sim.yaml` + `Dockerfile.humble`. | `colcon build` + `flake8` = 0 (`e64c7bb`) |
| **4** | **GIU Berlin Path Planner & Path Validation** | Implemented `map_utils.py`, `path_planner.py` (harmonic boundary potential centerline + clearance-constrained minimum-curvature raceline + forward-backward velocity profiler), `path_validator.py`, `simple_path_node.py`, and generated **4 validated Berlin CSV paths** (`min_clearance >= 0.948 m > 0.605 m` required) + `berlin_reference.csv` for `evo`. | `colcon build` + `flake8` = 0 (`9f2c9e6`) |
| **5** | **Pure Pursuit Controller Upgrade & Safety** | Implemented `controller_core.py` & `pure_pursuit_node.py` with exact Ackermann $\arctan(L \cdot \gamma)$ geometry, speed-adaptive lookahead, forward curvature preview speed regulation, low-pass + servo-rate saturation (`+-0.4189 rad`, `+-3.2 rad/s`), `/pause` & `odom_timeout` safe stops, and headless autonomous operation. | `colcon build` + `flake8` = 0 (`86942ae`) |
| **6** | **Full AROLA ROS 2 Integration** | Integrated single-authority TF ownership (`odom_to_base_link_node`), single `/drive` publisher guarantee, 4 YAML profiles (`params.yaml`, `berlin_conservative.yaml`, `berlin_race.yaml`, `hardware.params.yaml`), 3 `core` launch files, optional-`vesc_msgs` `watchdog_node` with periodic LiDAR silence detection, and Berlin-configured `race_monitor`. | `colcon build` 6/6 (`3.32s`), `flake8` = 0 (`259b172`) |
| **7 & 8** | **Closed-Loop Berlin Simulation, Tuning & Evidence** | Built deterministic ROS 2 `<->` `f110_gym` closed-loop harness (`tools/run_berlin_simulation.py`), executed 5-lap profile benchmarks (`40.70 s` Conservative -> `21.40 s` Balanced -> **`16.18 s` Fast-Race**, **`14.98 s` UltraAggr**, **`0` collisions**), all 4 path modes, 15-run perturbed spawn Monte Carlo (**`15/15 = 100%`**), fault injections, real ROS 2 `.db3` bag, and 4 publication plots. | Evidence generated & verified (`cb62e61`) |
| **9** | **Automated Test Suite, Static Analysis & Final Report** | Implemented **30 automated unit, lint, launch, rosbag2, and closed-loop integration tests** across all 6 packages (`colcon test` = **`30/30 passed, 0 errors, 0 failures`**, `pytest` = **`30 passed`**, default-79-col `flake8` = **`0 errors`**) and completed full competition documentation + ~4-page `docs/FINAL_REPORT.md`. | **100% Complete & Green** |
