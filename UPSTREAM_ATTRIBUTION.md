# Upstream Attribution & Provenance

This repository (`AME-RoboRacer-Project-YourStudentID`) is strictly derived from and preserves the original package structure, Git commit history, and licensing of the official German International University (GIU Berlin) AROLA starter repository:

- **Primary Starter Repository (Mandatory Baseline):**
  - `https://github.com/GIU-F1Tenth/minimal_sofware_example` (`commit 4239040`)
  - Maintainers: AROLA / GIU-F1Tenth Autonomous Racing Group
  - License: MIT (`LICENSE`)

- **Integrated Upstream Submodules (Vendored at Exact Upstream Revisions in `.gitmodules.upstream`):**
  1. `src/control/pure_pursuit` — `https://github.com/GIU-F1Tenth/pure_pursuit` (MIT License)
  2. `src/monitor/race_monitor` — `https://github.com/GIU-F1Tenth/race_monitor` (MIT License, Maintainer: Mohammed Abdeazim)
  3. `src/watchdog/watchdog` — `https://github.com/GIU-F1Tenth/watchdog` (MIT License, Maintainer: Fam Shihata)
  4. `f1tenth_gym_ros` — `https://github.com/GIU-F1Tenth/f1tenth_gym_ros` / `https://github.com/f1tenth/f1tenth_gym_ros` (MIT License, Copyright (c) 2020 Hongrui Zheng)

- **Official GIU Berlin Map & Reference Trajectory Assets:**
  - `f1tenth_gym_ros/maps/berlin.{png,pgm,yaml}` and `src/planning/simple_planner/maps/berlin.{png,pgm,yaml}` — Verified pixel-for-pixel identical (`360,000 / 360,000` cells) across:
    1. `https://github.com/f1tenth/f1tenth_gym` (`gym/f110_gym/envs/maps/berlin.{png,yaml}`)
    2. `https://github.com/GIU-F1Tenth/f1tenth_maps` (`maps/berlin.{png,yaml}`, converted from 1-bit `mode='1'` to 8-bit `mode='L'`)
    3. `https://github.com/GIU-F1Tenth/racing_playground` (`maps/berlin.{pgm,yaml}`)
  - `src/planning/simple_planner/paths/berlin_giu_reference.csv` — Derived from `https://github.com/GIU-F1Tenth/Raceline-Optimization` (`traj_race_cl-2025-08-03.csv`, deduplicated closing seam).
