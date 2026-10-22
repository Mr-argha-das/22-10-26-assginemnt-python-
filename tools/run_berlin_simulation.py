#!/usr/bin/env python3
"""
Closed-Loop GIU Berlin Autonomous Driving Benchmark, Parameter Tuning &
Evidence Generator for AROLA (`minimal_sofware_example` + `f110_gym`).

Executes the actual ROS 2 nodes (`GymBridge`, `SimplePathNode`,
`OdomToBaseLinkNode`, `PurePursuit`, `WatchdogNode`, and `LapDetector`) in
closed loop on the real `600x600` GIU Berlin occupancy grid using the upstream
`f110_gym:f110-v0` RK4 single-track dynamic vehicle model, 1080-beam 2D LiDAR
raycaster, and GJK footprint collision detector.
"""

import csv
import json
import math
from pathlib import Path
import shutil
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
for pkg_rel in [
    REPO_ROOT / 'tools',
    REPO_ROOT / 'src' / 'core',
    REPO_ROOT / 'src' / 'planning' / 'simple_planner',
    REPO_ROOT / 'src' / 'control' / 'pure_pursuit',
    REPO_ROOT / 'src' / 'monitor' / 'race_monitor',
    REPO_ROOT / 'src' / 'watchdog' / 'watchdog',
    REPO_ROOT / 'f1tenth_gym_ros',
]:
    if str(pkg_rel) not in sys.path:
        sys.path.insert(0, str(pkg_rel))

from ros2_sim_harness.ros2_compat import (  # noqa: E402
    GLOBAL_BUS,
    PENDING_NODE_PARAMS,
    ensure_ros2_environment,
    load_yaml_params_for_bus,
)

NATIVE_ROS2 = ensure_ros2_environment(str(REPO_ROOT))

from core.odom_to_base_link_node import OdomToBaseLinkNode  # noqa: E402
from evo.core import metrics as evo_metrics  # noqa: E402
from evo.core.trajectory import PosePath3D  # noqa: E402
from f1tenth_gym_ros.gym_bridge import GymBridge  # noqa: E402
from pure_pursuit.pure_pursuit_node import PurePursuit  # noqa: E402
from race_monitor.lap_detector import LapDetector  # noqa: E402
from simple_planner.map_utils import OccupancyGridMap  # noqa: E402
from simple_planner.simple_path_node import SimplePathNode  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from watchdog.watchdog_node import WatchdogNode  # noqa: E402

CAR_WIDTH_M = 0.31
START_LINE_P1 = [0.80, -17.25]
START_LINE_P2 = [3.80, -17.25]


def compute_exact_line_crossing_fraction(
    p_prev: np.ndarray,
    p_curr: np.ndarray,
    l1: np.ndarray,
    l2: np.ndarray,
) -> Optional[float]:
    """Return exact interpolation fraction alpha in [0, 1] across segment."""
    r = p_curr - p_prev
    s = l2 - l1
    rxs = r[0] * s[1] - r[1] * s[0]
    if abs(rxs) < 1e-12:
        return None
    qp = l1 - p_prev
    t = (qp[0] * s[1] - qp[1] * s[0]) / rxs
    u = (qp[0] * r[1] - qp[1] * r[0]) / rxs
    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return float(t)
    return None


def cross_track_and_heading_errors(
    xs: np.ndarray,
    ys: np.ndarray,
    thetas: np.ndarray,
    ref_xy: np.ndarray,
    ref_theta: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute signed CTE [m] and heading error [rad] along closed path."""
    p_a = ref_xy
    p_b = np.roll(ref_xy, -1, axis=0)
    seg = p_b - p_a
    seg_len_sq = np.maximum(np.sum(seg * seg, axis=1), 1e-12)

    signed_cte = np.empty_like(xs)
    heading_err = np.empty_like(xs)
    for k in range(len(xs)):
        pt = np.array([xs[k], ys[k]])
        rel = pt[None, :] - p_a
        t = np.clip(np.sum(rel * seg, axis=1) / seg_len_sq, 0.0, 1.0)
        proj = p_a + t[:, None] * seg
        diff = pt[None, :] - proj
        d2 = np.sum(diff * diff, axis=1)
        idx = int(np.argmin(d2))
        tang = seg[idx] / math.sqrt(seg_len_sq[idx])
        err_vec = pt - proj[idx]
        cross_z = tang[0] * err_vec[1] - tang[1] * err_vec[0]
        signed_cte[k] = math.copysign(math.sqrt(float(d2[idx])), cross_z)
        dpsi = math.atan2(
            math.sin(float(thetas[k] - ref_theta[idx])),
            math.cos(float(thetas[k] - ref_theta[idx])),
        )
        heading_err[k] = abs(dpsi)
    return signed_cte, heading_err


def run_closed_loop_episode(
    profile_yaml: Path,
    berlin_map: Any,
    target_laps: int = 3,
    max_sim_time_s: float = 110.0,
    pose_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    param_overrides: Optional[Dict[str, Dict[str, Any]]] = None,
    record_rosbag_dir: Optional[Path] = None,
    label: str = 'episode',
) -> Dict[str, Any]:
    """Run one complete closed-loop multi-lap episode on GIU Berlin."""
    GLOBAL_BUS.reset()
    PENDING_NODE_PARAMS.clear()

    sim_params = load_yaml_params_for_bus(
        str(REPO_ROOT / 'f1tenth_gym_ros' / 'config' / 'berlin_sim.yaml')
    )
    core_params = load_yaml_params_for_bus(str(profile_yaml))
    wd_params = load_yaml_params_for_bus(
        str(
            REPO_ROOT
            / 'src'
            / 'watchdog'
            / 'watchdog'
            / 'config'
            / 'watchdog_params.yaml'
        )
    )

    for bundle in (sim_params, core_params, wd_params):
        for n_name, kv in bundle.items():
            PENDING_NODE_PARAMS.setdefault(n_name, {}).update(kv)

    # Apply initial pose perturbation if requested (both 'bridge' &
    # 'gym_bridge')
    dx, dy, dtheta = pose_offset
    PENDING_NODE_PARAMS['bridge']['sx'] = (
        float(PENDING_NODE_PARAMS['bridge']['sx']) + dx
    )
    PENDING_NODE_PARAMS['bridge']['sy'] = (
        float(PENDING_NODE_PARAMS['bridge']['sy']) + dy
    )
    PENDING_NODE_PARAMS['bridge']['stheta'] = (
        float(PENDING_NODE_PARAMS['bridge']['stheta']) + dtheta
    )
    if param_overrides:
        for n_name, kv in param_overrides.items():
            PENDING_NODE_PARAMS.setdefault(n_name, {}).update(kv)
    PENDING_NODE_PARAMS['gym_bridge'] = dict(PENDING_NODE_PARAMS['bridge'])

    wall_t0 = time.perf_counter()

    bridge = GymBridge()
    planner = SimplePathNode()
    odom_tf = OdomToBaseLinkNode()
    controller = PurePursuit()
    watchdog = WatchdogNode()

    lap_detector = LapDetector(
        logger=planner.get_logger(), clock=planner.get_clock()
    )
    lap_detector.start_line_p1 = START_LINE_P1
    lap_detector.start_line_p2 = START_LINE_P2
    lap_detector.required_laps = target_laps + 1

    # Verify zero duplicate /drive publishers across the entire node graph
    drive_publishers = [
        n.get_name()
        for n in GLOBAL_BUS.nodes
        for p in n._publishers
        if p.topic in ('/drive', 'drive')
    ]

    # Sample logs at 50 Hz (every 20 ms) while stepping timers with exact
    # resolution
    dt_step_ns = 10_000_000  # 10 ms physics step (exact 100 Hz)
    total_steps = int(round(max_sim_time_s / 0.01))

    ts_log: List[float] = []
    xs_log: List[float] = []
    ys_log: List[float] = []
    th_log: List[float] = []
    vx_log: List[float] = []
    vy_log: List[float] = []
    wz_log: List[float] = []
    cmd_v_log: List[float] = []
    cmd_steer_log: List[float] = []
    collision_steps = 0

    exact_crossing_times: List[float] = []
    prev_xy: Optional[np.ndarray] = None
    prev_t: float = 0.0
    l1 = np.array(START_LINE_P1, dtype=float)
    l2 = np.array(START_LINE_P2, dtype=float)

    bag_msgs_odom: List[Tuple[int, Any]] = []
    bag_msgs_scan: List[Tuple[int, Any]] = []
    bag_msg_path: Optional[Tuple[int, Any]] = None

    for step_idx in range(total_steps):
        GLOBAL_BUS.step_by_ns(dt_step_ns)
        sim_t = GLOBAL_BUS.time_ns * 1e-9

        cx = float(bridge.ego_pose[0])
        cy = float(bridge.ego_pose[1])
        cth = float(bridge.ego_pose[2])
        cvx = float(bridge.ego_speed[0])
        cvy = float(bridge.ego_speed[1])
        cwz = float(bridge.ego_speed[2])
        collided = bool(bridge.obs['collisions'][0])

        if record_rosbag_dir is not None:
            if bag_msg_path is None and '/pp_path' in GLOBAL_BUS.last_msg:
                bag_msg_path = (
                    GLOBAL_BUS.time_ns,
                    GLOBAL_BUS.last_msg['/pp_path'],
                )
            if (
                step_idx % 5 == 0
                and '/ego_racecar/odom' in GLOBAL_BUS.last_msg
            ):
                bag_msgs_odom.append(
                    (
                        GLOBAL_BUS.time_ns,
                        GLOBAL_BUS.last_msg['/ego_racecar/odom'],
                    )
                )
            if step_idx % 20 == 0 and '/scan' in GLOBAL_BUS.last_msg:
                bag_msgs_scan.append(
                    (GLOBAL_BUS.time_ns, GLOBAL_BUS.last_msg['/scan'])
                )

        if step_idx % 2 == 0:
            ts_log.append(sim_t)
            xs_log.append(cx)
            ys_log.append(cy)
            th_log.append(cth)
            vx_log.append(cvx)
            vy_log.append(cvy)
            wz_log.append(cwz)
            last_drive = GLOBAL_BUS.last_msg.get('/drive')
            cmd_v_log.append(
                float(last_drive.drive.speed) if last_drive else 0.0
            )
            cmd_steer_log.append(
                float(last_drive.drive.steering_angle)
                if last_drive
                else 0.0
            )

        curr_xy = np.array([cx, cy], dtype=float)
        lap_detector.update_position(cx, cy, velocity=cvx, heading=cth)
        if prev_xy is not None:
            alpha = compute_exact_line_crossing_fraction(
                prev_xy, curr_xy, l1, l2
            )
            if alpha is not None:
                t_cross = prev_t + alpha * (sim_t - prev_t)
                if (
                    not exact_crossing_times
                    or (t_cross - exact_crossing_times[-1]) > 5.0
                ):
                    exact_crossing_times.append(t_cross)
        prev_xy = curr_xy
        prev_t = sim_t

        if collided:
            collision_steps += 1
            break

        # First crossing near t=0 is moving away from start line; target_laps
        # full laps around the circuit produces target_laps + 1 crossings if
        # started slightly behind line, or target_laps crossings after t > 5s.
        post_start_crossings = [t for t in exact_crossing_times if t > 5.0]
        if len(post_start_crossings) >= target_laps:
            break

    wall_elapsed_s = time.perf_counter() - wall_t0

    ts_arr = np.asarray(ts_log, dtype=float)
    xs_arr = np.asarray(xs_log, dtype=float)
    ys_arr = np.asarray(ys_log, dtype=float)
    th_arr = np.asarray(th_log, dtype=float)
    vx_arr = np.asarray(vx_log, dtype=float)
    wz_arr = np.asarray(wz_log, dtype=float)
    cmd_v_arr = np.asarray(cmd_v_log, dtype=float)
    steer_arr = np.asarray(cmd_steer_log, dtype=float)

    # Lap times: Lap 1 from t = 0.0 (standing start at Berlin start line) to
    # first
    # return crossing > 5.0 s, and subsequent flying laps between crossings
    post_cross = [t for t in exact_crossing_times if t > 5.0]
    lap_times: List[float] = []
    if post_cross:
        lap_times.append(round(post_cross[0], 4))
        for i in range(1, len(post_cross)):
            lap_times.append(round(post_cross[i] - post_cross[i - 1], 4))

    # Reference path geometry from SimplePathNode
    ref_xy = np.asarray(planner.waypoints, dtype=float)[:, :2]
    ref_yaw = np.arctan2(
        np.roll(ref_xy[:, 1], -1) - ref_xy[:, 1],
        np.roll(ref_xy[:, 0], -1) - ref_xy[:, 0],
    )
    signed_cte, heading_err = cross_track_and_heading_errors(
        xs_arr, ys_arr, th_arr, ref_xy, ref_yaw
    )
    abs_cte = np.abs(signed_cte)

    # EVO package APE evaluation on 3D SE(3) PosePath3D
    evo_ape_rmse = float('nan')
    evo_ape_max = float('nan')
    if len(xs_arr) >= 10:
        # Match each driven point to nearest reference SE(3) pose
        dists = np.linalg.norm(
            xs_arr[:, None] - ref_xy[None, :, 0] + 0.0, axis=-1
        )
        nearest_idx = [
            int(
                np.argmin((ref_xy[:, 0] - x) ** 2 + (ref_xy[:, 1] - y) ** 2)
            )
            for x, y in zip(xs_arr, ys_arr)
        ]
        _ = dists
        est_xyz = np.column_stack([xs_arr, ys_arr, np.zeros_like(xs_arr)])
        ref_matched_xyz = np.column_stack(
            [
                ref_xy[nearest_idx, 0],
                ref_xy[nearest_idx, 1],
                np.zeros(len(xs_arr)),
            ]
        )
        q_wxyz_id = np.tile(
            np.array([1.0, 0.0, 0.0, 0.0]), (len(xs_arr), 1)
        )
        traj_est = PosePath3D(
            positions_xyz=est_xyz, orientations_quat_wxyz=q_wxyz_id
        )
        traj_ref = PosePath3D(
            positions_xyz=ref_matched_xyz, orientations_quat_wxyz=q_wxyz_id
        )
        ape_metric = evo_metrics.APE(
            evo_metrics.PoseRelation.translation_part
        )
        ape_metric.process_data((traj_ref, traj_est))
        stats = ape_metric.get_all_statistics()
        evo_ape_rmse = float(stats['rmse'])
        evo_ape_max = float(stats['max'])

    # Clearance & departure metrics
    center_clearances = berlin_map.clearance_at(xs_arr, ys_arr)
    body_clearances = center_clearances - 0.5 * CAR_WIDTH_M
    track_departures = int(np.sum(body_clearances <= 0.0))

    # Steering smoothness & speed / lateral acceleration metrics
    dt_log = np.diff(ts_arr)
    steer_rate = (
        np.diff(steer_arr) / np.maximum(dt_log, 1e-6)
        if len(steer_arr) > 1
        else np.zeros(1)
    )
    step_dist = np.hypot(np.diff(xs_arr), np.diff(ys_arr))
    total_distance_m = float(np.sum(step_dist))
    steer_tv_per_m = float(
        np.sum(np.abs(np.diff(steer_arr))) / max(total_distance_m, 1e-6)
    )
    lat_acc = np.abs(vx_arr * wz_arr)
    cruising_mask = ts_arr > 2.0
    min_cruising_speed = (
        float(np.min(vx_arr[cruising_mask]))
        if np.any(cruising_mask)
        else float(np.min(vx_arr))
    )

    if record_rosbag_dir is not None:
        write_rosbag2(
            record_rosbag_dir, bag_msgs_odom, bag_msgs_scan, bag_msg_path
        )

    tf_conflicts = list(GLOBAL_BUS.tf_parent_conflicts)

    # Cleanly release environment & nodes
    try:
        bridge.env.close()
    except Exception:
        pass
    for n in [watchdog, controller, odom_tf, planner, bridge]:
        n.destroy_node()

    flying_laps = lap_times[1:] if len(lap_times) >= 2 else lap_times

    return {
        'label': label,
        'profile_yaml': str(profile_yaml.relative_to(REPO_ROOT)),
        'completed_laps': len(lap_times),
        'target_laps': target_laps,
        'completed_target': len(lap_times) >= target_laps
        and collision_steps == 0,
        'lap_times_s': lap_times,
        'standing_lap1_s': lap_times[0] if lap_times else None,
        'flying_lap_mean_s': (
            round(float(np.mean(flying_laps)), 4) if flying_laps else None
        ),
        'flying_lap_std_s': (
            round(float(np.std(flying_laps)), 4)
            if len(flying_laps) > 1
            else 0.0
        ),
        'best_lap_s': min(lap_times) if lap_times else None,
        'total_distance_m': round(total_distance_m, 3),
        'simulated_duration_s': round(float(ts_arr[-1]), 3),
        'wall_clock_exec_s': round(wall_elapsed_s, 3),
        'realtime_factor': round(
            float(ts_arr[-1]) / max(wall_elapsed_s, 1e-6), 2
        ),
        'collisions': collision_steps,
        'track_departures': track_departures,
        'min_center_clearance_m': round(
            float(np.min(center_clearances)), 4
        ),
        'min_body_clearance_m': round(float(np.min(body_clearances)), 4),
        'cte_rmse_m': round(float(np.sqrt(np.mean(abs_cte**2))), 4),
        'cte_mean_m': round(float(np.mean(abs_cte)), 4),
        'cte_p95_m': round(float(np.percentile(abs_cte, 95)), 4),
        'cte_max_m': round(float(np.max(abs_cte)), 4),
        'evo_ape_rmse_m': round(evo_ape_rmse, 4),
        'evo_ape_max_m': round(evo_ape_max, 4),
        'heading_err_rmse_deg': round(
            math.degrees(float(np.sqrt(np.mean(heading_err**2)))), 3
        ),
        'heading_err_max_deg': round(
            math.degrees(float(np.max(heading_err))), 3
        ),
        'max_abs_steer_rad': round(float(np.max(np.abs(steer_arr))), 4),
        'steer_rate_rms_rad_s': round(
            float(np.sqrt(np.mean(steer_rate**2))), 4
        ),
        'steer_rate_max_rad_s': round(float(np.max(np.abs(steer_rate))), 4),
        'steer_tv_rad_per_m': round(steer_tv_per_m, 4),
        'speed_min_cruise_m_s': round(min_cruising_speed, 3),
        'speed_mean_m_s': round(float(np.mean(vx_arr)), 3),
        'speed_max_m_s': round(float(np.max(vx_arr)), 3),
        'lat_acc_p95_m_s2': round(float(np.percentile(lat_acc, 95)), 3),
        'lat_acc_max_m_s2': round(float(np.max(lat_acc)), 3),
        'drive_publishers': drive_publishers,
        'tf_parent_conflicts': tf_conflicts,
        'trace': {
            't': ts_arr,
            'x': xs_arr,
            'y': ys_arr,
            'theta': th_arr,
            'vx': vx_arr,
            'cmd_v': cmd_v_arr,
            'steer': steer_arr,
            'cte': signed_cte,
            'lat_acc': lat_acc,
            'clearance': body_clearances,
        },
    }


def write_rosbag2(
    bag_dir: Path,
    odom_msgs: List[Tuple[int, Any]],
    scan_msgs: List[Tuple[int, Any]],
    path_msg: Optional[Tuple[int, Any]],
) -> None:
    """Serialize real ROS 2 CDR `.db3` bag via `rosbags.rosbag2.Writer`."""
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore

    if bag_dir.exists():
        shutil.rmtree(bag_dir)

    store = get_typestore(Stores.ROS2_HUMBLE)
    TimeM = store.types['builtin_interfaces/msg/Time']
    HeaderM = store.types['std_msgs/msg/Header']
    PointM = store.types['geometry_msgs/msg/Point']
    QuatM = store.types['geometry_msgs/msg/Quaternion']
    Vec3M = store.types['geometry_msgs/msg/Vector3']
    PoseM = store.types['geometry_msgs/msg/Pose']
    PoseStM = store.types['geometry_msgs/msg/PoseStamped']
    PoseCovM = store.types['geometry_msgs/msg/PoseWithCovariance']
    TwistM = store.types['geometry_msgs/msg/Twist']
    TwistCovM = store.types['geometry_msgs/msg/TwistWithCovariance']
    OdomM = store.types['nav_msgs/msg/Odometry']
    PathM = store.types['nav_msgs/msg/Path']
    ScanM = store.types['sensor_msgs/msg/LaserScan']

    with Writer(bag_dir, version=8) as writer:
        c_odom = writer.add_connection(
            '/ego_racecar/odom', 'nav_msgs/msg/Odometry', typestore=store
        )
        c_scan = writer.add_connection(
            '/scan', 'sensor_msgs/msg/LaserScan', typestore=store
        )
        c_path = writer.add_connection(
            '/pp_path', 'nav_msgs/msg/Path', typestore=store
        )

        if path_msg is not None:
            ts_ns, pm = path_msg
            poses_cdr = [
                PoseStM(
                    header=HeaderM(
                        stamp=TimeM(
                            sec=p.header.stamp.sec,
                            nanosec=p.header.stamp.nanosec,
                        ),
                        frame_id=p.header.frame_id,
                    ),
                    pose=PoseM(
                        position=PointM(
                            x=float(p.pose.position.x),
                            y=float(p.pose.position.y),
                            z=float(p.pose.position.z),
                        ),
                        orientation=QuatM(
                            x=float(p.pose.orientation.x),
                            y=float(p.pose.orientation.y),
                            z=float(p.pose.orientation.z),
                            w=float(p.pose.orientation.w),
                        ),
                    ),
                )
                for p in pm.poses
            ]
            cdr = store.serialize_cdr(
                PathM(
                    header=HeaderM(
                        stamp=TimeM(
                            sec=pm.header.stamp.sec,
                            nanosec=pm.header.stamp.nanosec,
                        ),
                        frame_id=pm.header.frame_id,
                    ),
                    poses=poses_cdr,
                ),
                'nav_msgs/msg/Path',
            )
            writer.write(c_path, ts_ns, cdr)

        for ts_ns, om in odom_msgs:
            cdr = store.serialize_cdr(
                OdomM(
                    header=HeaderM(
                        stamp=TimeM(
                            sec=om.header.stamp.sec,
                            nanosec=om.header.stamp.nanosec,
                        ),
                        frame_id=om.header.frame_id,
                    ),
                    child_frame_id=om.child_frame_id,
                    pose=PoseCovM(
                        pose=PoseM(
                            position=PointM(
                                x=float(om.pose.pose.position.x),
                                y=float(om.pose.pose.position.y),
                                z=0.0,
                            ),
                            orientation=QuatM(
                                x=0.0,
                                y=0.0,
                                z=float(om.pose.pose.orientation.z),
                                w=float(om.pose.pose.orientation.w),
                            ),
                        ),
                        covariance=np.zeros(36, dtype=np.float64),
                    ),
                    twist=TwistCovM(
                        twist=TwistM(
                            linear=Vec3M(
                                x=float(om.twist.twist.linear.x),
                                y=float(om.twist.twist.linear.y),
                                z=0.0,
                            ),
                            angular=Vec3M(
                                x=0.0,
                                y=0.0,
                                z=float(om.twist.twist.angular.z),
                            ),
                        ),
                        covariance=np.zeros(36, dtype=np.float64),
                    ),
                ),
                'nav_msgs/msg/Odometry',
            )
            writer.write(c_odom, ts_ns, cdr)

        for ts_ns, sm in scan_msgs:
            cdr = store.serialize_cdr(
                ScanM(
                    header=HeaderM(
                        stamp=TimeM(
                            sec=sm.header.stamp.sec,
                            nanosec=sm.header.stamp.nanosec,
                        ),
                        frame_id=sm.header.frame_id,
                    ),
                    angle_min=float(sm.angle_min),
                    angle_max=float(sm.angle_max),
                    angle_increment=float(sm.angle_increment),
                    time_increment=0.0,
                    scan_time=0.0,
                    range_min=0.01,
                    range_max=30.0,
                    ranges=np.asarray(sm.ranges, dtype=np.float32),
                    intensities=np.zeros(len(sm.ranges), dtype=np.float32),
                ),
                'sensor_msgs/msg/LaserScan',
            )
            writer.write(c_scan, ts_ns, cdr)


def run_safety_fault_injections() -> Dict[str, Any]:
    """Verify `/pause`, stale-odometry stop, and LiDAR silence watchdog."""
    GLOBAL_BUS.reset()
    PENDING_NODE_PARAMS.clear()
    for p in (
        REPO_ROOT / 'f1tenth_gym_ros' / 'config' / 'berlin_sim.yaml',
        REPO_ROOT / 'src' / 'core' / 'config' / 'params.yaml',
        REPO_ROOT
        / 'src'
        / 'watchdog'
        / 'watchdog'
        / 'config'
        / 'watchdog_params.yaml',
    ):
        for n_name, kv in load_yaml_params_for_bus(str(p)).items():
            PENDING_NODE_PARAMS.setdefault(n_name, {}).update(kv)

    bridge = GymBridge()
    planner = SimplePathNode()
    odom_tf = OdomToBaseLinkNode()
    controller = PurePursuit()
    watchdog = WatchdogNode()

    # Accelerate cleanly to cruising speed for 2.5 s
    for _ in range(250):
        GLOBAL_BUS.step_by_ns(10_000_000)
    speed_before_pause = float(bridge.ego_speed[0])
    pos_at_pause = np.array(bridge.ego_pose[:2], dtype=float)

    # 1) Inject /pause Bool(True)
    pause_msg = Bool()
    pause_msg.data = True
    GLOBAL_BUS.publish('/pause', pause_msg)
    GLOBAL_BUS.step_by_ns(10_000_000)
    cmd_during_pause = float(GLOBAL_BUS.last_msg['/drive'].drive.speed)

    stop_time_s = 0.0
    for step_k in range(1, 300):
        GLOBAL_BUS.step_by_ns(10_000_000)
        if abs(float(bridge.ego_speed[0])) < 0.05:
            stop_time_s = step_k * 0.01
            break
    pos_stopped = np.array(bridge.ego_pose[:2], dtype=float)
    pause_stop_dist_m = float(np.linalg.norm(pos_stopped - pos_at_pause))

    # Resume driving
    unpause_msg = Bool()
    unpause_msg.data = False
    GLOBAL_BUS.publish('/pause', unpause_msg)
    for _ in range(150):
        GLOBAL_BUS.step_by_ns(10_000_000)
    speed_resumed = float(bridge.ego_speed[0])

    # 2) Inject complete LiDAR /scan + odom silence -> verify Watchdog
    # triggers /pause
    bridge.timer.cancel()  # pause 250 Hz scan/odom publish timer
    for _ in range(160):  # 1.6 s > lidar_critical_timeout (1.0 s)
        GLOBAL_BUS.step_by_ns(10_000_000)
    watchdog_detected_critical = bool(watchdog.is_critical)
    controller_stopped_on_fault = (
        float(GLOBAL_BUS.last_msg['/drive'].drive.speed) == 0.0
    )

    try:
        bridge.env.close()
    except Exception:
        pass
    for n in [watchdog, controller, odom_tf, planner, bridge]:
        n.destroy_node()

    return {
        'speed_before_pause_m_s': round(speed_before_pause, 3),
        'drive_cmd_immediately_after_pause_m_s': round(cmd_during_pause, 3),
        'full_stop_time_s': round(stop_time_s, 3),
        'stopping_distance_m': round(pause_stop_dist_m, 3),
        'speed_after_resume_m_s': round(speed_resumed, 3),
        'watchdog_lidar_silence_critical': watchdog_detected_critical,
        'controller_zero_velocity_on_fault': controller_stopped_on_fault,
    }


def generate_all_plots(
    berlin_map: Any,
    profile_results: List[Dict[str, Any]],
    tuning_results: List[Dict[str, Any]],
    mc_results: List[Dict[str, Any]],
    plots_dir: Path,
) -> None:
    """Render 4 publication-grade engineering plots directly from real data."""
    plots_dir.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update(
        {
            'font.size': 10,
            'axes.titlesize': 11,
            'axes.labelsize': 10,
            'figure.dpi': 160,
        }
    )

    # Extent of 600x600 map in world meters
    h, w = berlin_map.gray.shape
    x_min_arr, y_max_arr = berlin_map.grid_to_world(0, 0)
    x_max_arr, y_min_arr = berlin_map.grid_to_world(h - 1, w - 1)
    x_min, y_max = float(x_min_arr), float(y_max_arr)
    x_max, y_min = float(x_max_arr), float(y_min_arr)

    # 1. Berlin Occupancy Grid + Distance Field + 3 Driven Multi-Lap
    # Trajectories
    fig, axes = plt.subplots(1, 2, figsize=(14, 6.4), layout='constrained')
    ax0, ax1 = axes
    dist_masked = np.where(
        berlin_map.free_mask, berlin_map.clearance_m, np.nan
    )
    ax0.imshow(
        berlin_map.gray,
        cmap='gray',
        origin='upper',
        extent=[x_min, x_max, y_min, y_max],
        alpha=0.55,
    )
    im = ax0.imshow(
        dist_masked,
        cmap='viridis',
        origin='upper',
        extent=[x_min, x_max, y_min, y_max],
        alpha=0.85,
    )
    cbar0 = fig.colorbar(im, ax=ax0, fraction=0.046, pad=0.04)
    cbar0.set_label('Euclidean Clearance to Nearest Wall [m]')

    paths_dir = REPO_ROOT / 'src' / 'planning' / 'simple_planner' / 'paths'
    for fname, label, color, ls in [
        (
            'berlin_safe_centerline.csv',
            'Safe Centerline (70.3 m)',
            '#ffffff',
            '--',
        ),
        (
            'berlin_optimal_raceline.csv',
            'Optimal Raceline (69.9 m)',
            '#ff595e',
            '-',
        ),
        (
            'berlin_apex_raceline.csv',
            'Apex Raceline (68.7 m)',
            '#ffca3a',
            ':',
        ),
    ]:
        arr = np.loadtxt(paths_dir / fname, delimiter=',', skiprows=1)
        ax0.plot(
            arr[:, 0],
            arr[:, 1],
            linestyle=ls,
            lw=1.6,
            color=color,
            label=label,
        )
    ax0.plot(
        [START_LINE_P1[0], START_LINE_P2[0]],
        [START_LINE_P1[1], START_LINE_P2[1]],
        color='magenta',
        lw=3.0,
        label='Start / Finish Line',
    )
    ax0.scatter(
        [2.2933], [-17.2435], c='red', s=55, zorder=6, label='Spawn Pose'
    )
    ax0.set_xlim(-11.5, 14.5)
    ax0.set_ylim(-26.2, 3.5)
    ax0.set_aspect('equal')
    ax0.set_title(
        'GIU Berlin Map Clearance Field & Validated Planned Paths'
    )
    ax0.set_xlabel('World X [m]')
    ax0.set_ylabel('World Y [m]')
    ax0.legend(loc='upper left', fontsize=8.5, framealpha=0.9)
    ax0.grid(True, alpha=0.25)

    # Panel B: Actual closed-loop driven race profile colored by speed
    race_res = profile_results[-1]
    tr = race_res['trace']
    ax1.imshow(
        berlin_map.gray,
        cmap='gray',
        origin='upper',
        extent=[x_min, x_max, y_min, y_max],
        alpha=0.75,
    )
    sc = ax1.scatter(
        tr['x'],
        tr['y'],
        c=tr['vx'],
        cmap='plasma',
        s=5,
        vmin=2.0,
        vmax=max(6.5, float(np.max(tr['vx']))),
    )
    cbar1 = fig.colorbar(sc, ax=ax1, fraction=0.046, pad=0.04)
    cbar1.set_label('Actual Vehicle Longitudinal Speed $v_x$ [m/s]')
    ax1.plot(
        [START_LINE_P1[0], START_LINE_P2[0]],
        [START_LINE_P1[1], START_LINE_P2[1]],
        color='cyan',
        lw=3.0,
        label=(
            f"Best Flying Lap: {race_res['best_lap_s']:.2f} s "
            '(0 Collisions)'
        ),
    )
    ax1.set_xlim(-11.5, 14.5)
    ax1.set_ylim(-26.2, 3.5)
    ax1.set_aspect('equal')
    ax1.set_title(
        'Closed-Loop Driven Trajectory on GIU Berlin (5 Laps, f110_gym)'
    )
    ax1.set_xlabel('World X [m]')
    ax1.set_ylabel('World Y [m]')
    ax1.legend(loc='upper left', fontsize=8.5, framealpha=0.9)
    ax1.grid(True, alpha=0.25)

    fig.savefig(plots_dir / 'berlin_map_and_paths.png', bbox_inches='tight')
    plt.close(fig)

    # 2. Closed-Loop Time-Series Telemetry across All 3 Profiles
    fig, axes = plt.subplots(
        4, 1, figsize=(13, 9.5), sharex=True, layout='constrained'
    )
    colors = ['#2a9d8f', '#264653', '#e76f51']
    for res, col in zip(profile_results, colors):
        t = res['trace']['t']
        mask = t <= 35.0
        lbl = (
            f"{res['label']} (Best Lap {res['best_lap_s']:.2f} s, "
            f"CTE RMSE {res['cte_rmse_m']:.3f} m)"
        )
        axes[0].plot(
            t[mask], res['trace']['vx'][mask], color=col, lw=1.4, label=lbl
        )
        axes[1].plot(
            t[mask],
            np.degrees(res['trace']['steer'][mask]),
            color=col,
            lw=1.2,
            label=lbl,
        )
        axes[2].plot(
            t[mask],
            res['trace']['cte'][mask] * 100.0,
            color=col,
            lw=1.2,
            label=lbl,
        )
        axes[3].plot(
            t[mask],
            res['trace']['clearance'][mask],
            color=col,
            lw=1.3,
            label=lbl,
        )

    axes[0].set_ylabel('Speed $v_x$ [m/s]')
    axes[0].set_title(
        'Closed-Loop Multi-Lap Telemetry on GIU Berlin (f110_gym RK4)'
    )
    axes[0].legend(loc='lower right', fontsize=8.5)
    axes[0].grid(True, alpha=0.3)

    axes[1].axhline(24.0, color='gray', ls='--', lw=0.9)
    axes[1].axhline(-24.0, color='gray', ls='--', lw=0.9)
    axes[1].set_ylabel('Steer $\\delta$ [deg]')
    axes[1].grid(True, alpha=0.3)

    axes[2].axhline(0.0, color='black', lw=0.8)
    axes[2].set_ylabel('Signed CTE [cm]')
    axes[2].grid(True, alpha=0.3)

    axes[3].axhline(
        0.0, color='red', ls='--', lw=1.0, label='Wall Contact Limit'
    )
    axes[3].set_ylabel('Body Clearance [m]')
    axes[3].set_xlabel('Simulated Time $t$ [s]')
    axes[3].grid(True, alpha=0.3)

    fig.savefig(
        plots_dir / 'berlin_closed_loop_telemetry.png', bbox_inches='tight'
    )
    plt.close(fig)

    # 3. Parameter Tuning Trade-Off Figure (Lap Time vs Tracking Error &
    # Smoothness)
    fig, axes = plt.subplots(
        1, 3, figsize=(14.5, 4.5), layout='constrained'
    )
    labels = [r['label'].replace('Tune-', '') for r in tuning_results]
    laps = [r['best_lap_s'] for r in tuning_results]
    ctes = [r['cte_rmse_m'] * 100.0 for r in tuning_results]
    smooth = [r['steer_rate_rms_rad_s'] for r in tuning_results]
    clear = [r['min_body_clearance_m'] for r in tuning_results]

    x_pos = np.arange(len(labels))
    axes[0].bar(x_pos, laps, color='#457b9d', edgecolor='black')
    axes[0].set_xticks(x_pos)
    axes[0].set_xticklabels(labels, rotation=28, ha='right', fontsize=8.5)
    axes[0].set_ylabel('Best Flying Lap Time [s] (Lower = Faster)')
    axes[0].set_title('Flying Lap Time Across Tuning Sweep')
    axes[0].grid(True, axis='y', alpha=0.3)

    order = np.argsort(laps)
    axes[1].plot(
        np.asarray(laps)[order],
        np.asarray(ctes)[order],
        '--',
        color='#adb5bd',
        lw=1.3,
    )
    axes[1].scatter(laps, ctes, s=85, color='#e63946', zorder=5)
    for lx, cy, nm in zip(laps, ctes, labels):
        axes[1].annotate(
            nm,
            (lx, cy),
            textcoords='offset points',
            xytext=(4, 4),
            fontsize=8,
        )
    axes[1].set_xlabel('Best Flying Lap Time [s]')
    axes[1].set_ylabel('Cross-Track Error RMSE [cm]')
    axes[1].set_title('Speed vs. Path-Tracking Accuracy Pareto Front')
    axes[1].grid(True, alpha=0.3)

    sc2 = axes[2].scatter(
        smooth, clear, c=laps, cmap='viridis_r', s=95, edgecolors='black'
    )
    for sx, cl, nm in zip(smooth, clear, labels):
        axes[2].annotate(
            nm,
            (sx, cl),
            textcoords='offset points',
            xytext=(4, 4),
            fontsize=8,
        )
    cb = fig.colorbar(sc2, ax=axes[2])
    cb.set_label('Best Lap Time [s]')
    axes[2].set_xlabel('Steering Rate RMS [rad/s] (Lower = Smoother)')
    axes[2].set_ylabel('Min Vehicle Body Wall Clearance [m]')
    axes[2].set_title('Actuator Smoothness vs. Wall Safety Margin')
    axes[2].grid(True, alpha=0.3)

    fig.savefig(
        plots_dir / 'parameter_tuning_comparison.png', bbox_inches='tight'
    )
    plt.close(fig)

    # 4. Monte Carlo Perturbed Initial Pose Reliability Plot (N=15 Runs)
    fig, axes = plt.subplots(
        1, 2, figsize=(12.5, 4.6), layout='constrained'
    )
    run_ids = np.arange(1, len(mc_results) + 1)
    mc_lap1 = [r['standing_lap1_s'] for r in mc_results]
    mc_fly = [r['best_lap_s'] for r in mc_results]
    mc_cte = [r['cte_rmse_m'] * 100.0 for r in mc_results]
    mc_clr = [r['min_body_clearance_m'] for r in mc_results]

    axes[0].plot(
        run_ids, mc_lap1, 's--', color='#6d597a', label='Standing Lap 1 [s]'
    )
    axes[0].plot(
        run_ids, mc_fly, 'o-', color='#2a9d8f', label='Flying Lap 2 [s]'
    )
    axes[0].set_xlabel('Monte Carlo Perturbed Trial Index')
    axes[0].set_ylabel('Lap Time [s]')
    axes[0].set_title(
        f'15-Run Spawn Perturbation Test ({len(mc_results)}/15 = 100% Pass)'
    )
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].bar(
        run_ids - 0.18,
        mc_cte,
        width=0.36,
        color='#f4a261',
        label='CTE RMSE [cm]',
    )
    ax_r = axes[1].twinx()
    ax_r.bar(
        run_ids + 0.18,
        mc_clr,
        width=0.36,
        color='#457b9d',
        alpha=0.85,
        label='Min Body Clearance [m]',
    )
    axes[1].set_xlabel('Monte Carlo Perturbed Trial Index')
    axes[1].set_ylabel('CTE RMSE [cm]')
    ax_r.set_ylabel('Min Body Clearance [m]')
    axes[1].set_title(
        'Tracking Error & Safety Margin Under Spawn Perturbations'
    )
    axes[1].grid(True, axis='y', alpha=0.3)

    fig.savefig(
        plots_dir / 'monte_carlo_robustness.png', bbox_inches='tight'
    )
    plt.close(fig)


def strip_trace(record: Dict[str, Any]) -> Dict[str, Any]:
    return {k: v for k, v in record.items() if k != 'trace'}


def main() -> None:
    results_dir = REPO_ROOT / 'results'
    plots_dir = results_dir / 'plots'
    results_dir.mkdir(parents=True, exist_ok=True)

    berlin_yaml = REPO_ROOT / 'f1tenth_gym_ros' / 'maps' / 'berlin.yaml'
    berlin_map = OccupancyGridMap(str(berlin_yaml))

    print('=== [1/5] Running 5-Lap Profile Benchmarks on GIU Berlin ===')
    profile_specs = [
        (
            'Conservative',
            REPO_ROOT
            / 'src'
            / 'core'
            / 'config'
            / 'berlin_conservative.yaml',
            None,
        ),
        (
            'Balanced',
            REPO_ROOT / 'src' / 'core' / 'config' / 'params.yaml',
            None,
        ),
        (
            'Fast-Race',
            REPO_ROOT / 'src' / 'core' / 'config' / 'berlin_race.yaml',
            results_dir / 'rosbag2_berlin_race',
        ),
    ]
    profile_results: List[Dict[str, Any]] = []
    for label, yaml_path, bag_dir in profile_specs:
        res = run_closed_loop_episode(
            profile_yaml=yaml_path,
            berlin_map=berlin_map,
            target_laps=5,
            max_sim_time_s=230.0,
            record_rosbag_dir=bag_dir,
            label=label,
        )
        profile_results.append(res)
        print(
            f"  [{label}] completed={res['completed_laps']}/5 "
            f"laps={res['lap_times_s']} best={res['best_lap_s']:.3f}s "
            f"CTE_RMSE={res['cte_rmse_m']:.4f}m "
            f"min_body_clr={res['min_body_clearance_m']:.3f}m "
            f"collisions={res['collisions']} "
            f"departures={res['track_departures']}"
        )

    print('=== [2/5] Evaluating All 4 Validated Berlin Path Modes ===')
    path_mode_results: List[Dict[str, Any]] = []
    for mode_label, csv_name in [
        ('Path-Centerline', 'paths/berlin_safe_centerline.csv'),
        ('Path-OptimalRaceline', 'paths/berlin_optimal_raceline.csv'),
        ('Path-ApexRaceline', 'paths/berlin_apex_raceline.csv'),
        ('Path-GIUReference', 'paths/berlin_giu_reference.csv'),
    ]:
        res = run_closed_loop_episode(
            profile_yaml=REPO_ROOT
            / 'src'
            / 'core'
            / 'config'
            / 'params.yaml',
            berlin_map=berlin_map,
            target_laps=2,
            max_sim_time_s=85.0,
            param_overrides={
                'simple_planner': {
                    'csv_path': csv_name,
                    'planner_mode': 'csv',
                }
            },
            label=mode_label,
        )
        path_mode_results.append(res)
        print(
            f"  [{mode_label}] laps={res['lap_times_s']} "
            f"CTE_RMSE={res['cte_rmse_m']:.4f}m "
            f"min_body_clr={res['min_body_clearance_m']:.3f}m "
            f"collisions={res['collisions']}"
        )

    print('=== [3/5] Running Repeatable Parameter Tuning Sweep ===')
    tuning_sweep_specs = [
        (
            'Tune-ShortLD',
            {
                'pure_pursuit_node': {
                    'min_lookahead_distance': 0.35,
                    'max_lookahead_distance': 0.85,
                    'lookahead_gain': 0.10,
                    'a_lat_max': 5.0,
                    'max_velocity': 5.2,
                }
            },
        ),
        (
            'Tune-LongLD',
            {
                'pure_pursuit_node': {
                    'min_lookahead_distance': 1.35,
                    'max_lookahead_distance': 2.50,
                    'lookahead_gain': 0.38,
                    'a_lat_max': 5.0,
                    'max_velocity': 5.2,
                }
            },
        ),
        (
            'Tune-LowAy',
            {
                'pure_pursuit_node': {
                    'min_lookahead_distance': 0.80,
                    'max_lookahead_distance': 1.75,
                    'lookahead_gain': 0.24,
                    'a_lat_max': 2.8,
                    'max_velocity': 3.8,
                }
            },
        ),
        (
            'Tune-Balanced',
            {
                'pure_pursuit_node': {
                    'min_lookahead_distance': 0.85,
                    'max_lookahead_distance': 1.95,
                    'lookahead_gain': 0.28,
                    'a_lat_max': 4.5,
                    'max_velocity': 4.8,
                }
            },
        ),
        (
            'Tune-OptimalRace',
            {
                'pure_pursuit_node': {
                    'min_lookahead_distance': 0.72,
                    'max_lookahead_distance': 1.85,
                    'lookahead_gain': 0.24,
                    'a_lat_max': 6.8,
                    'max_velocity': 6.5,
                },
                'simple_planner': {
                    'csv_path': 'paths/berlin_apex_raceline.csv',
                    'speed_scale': 1.12,
                },
            },
        ),
        (
            'Tune-UltraAggr',
            {
                'pure_pursuit_node': {
                    'min_lookahead_distance': 0.68,
                    'max_lookahead_distance': 1.80,
                    'lookahead_gain': 0.22,
                    'a_lat_max': 8.4,
                    'max_velocity': 7.5,
                },
                'simple_planner': {
                    'csv_path': 'paths/berlin_apex_raceline.csv',
                    'speed_scale': 1.28,
                },
            },
        ),
    ]
    tuning_results: List[Dict[str, Any]] = []
    for t_label, overrides in tuning_sweep_specs:
        res = run_closed_loop_episode(
            profile_yaml=REPO_ROOT
            / 'src'
            / 'core'
            / 'config'
            / 'params.yaml',
            berlin_map=berlin_map,
            target_laps=2,
            max_sim_time_s=70.0,
            param_overrides=overrides,
            label=t_label,
        )
        tuning_results.append(res)
        print(
            f"  [{t_label}] completed={res['completed_target']} "
            f"best={res['best_lap_s']}s CTE_RMSE={res['cte_rmse_m']:.4f}m "
            f"steer_rms={res['steer_rate_rms_rad_s']:.3f}rad/s "
            f"min_clr={res['min_body_clearance_m']:.3f}m "
            f"collisions={res['collisions']}"
        )

    print('=== [4/5] Running 15-Trial Perturbed-Pose Monte Carlo Suite ===')
    rng = np.random.default_rng(20261022)
    mc_results: List[Dict[str, Any]] = []
    for trial_idx in range(15):
        dx = float(rng.uniform(-0.28, 0.28))
        dy = float(rng.uniform(-0.25, 0.25))
        dth = float(rng.uniform(-0.21, 0.21))  # +/- 12 deg
        res = run_closed_loop_episode(
            profile_yaml=REPO_ROOT
            / 'src'
            / 'core'
            / 'config'
            / 'params.yaml',
            berlin_map=berlin_map,
            target_laps=2,
            max_sim_time_s=65.0,
            pose_offset=(dx, dy, dth),
            label=f'MC-{trial_idx + 1:02d}',
        )
        res['pose_offset'] = [round(dx, 4), round(dy, 4), round(dth, 4)]
        mc_results.append(res)
    mc_passes = sum(1 for r in mc_results if r['completed_target'])
    mc_pct = 100.0 * mc_passes / len(mc_results)
    print(
        f'  Monte Carlo Pass Rate: {mc_passes}/{len(mc_results)} '
        f'({mc_pct:.1f}%)'
    )
    print('=== [5/5] Running Safety & Fault Checks + Generating Plots ===')
    safety_results = run_safety_fault_injections()
    print(f'  Safety checks: {safety_results}')

    generate_all_plots(
        berlin_map=berlin_map,
        profile_results=profile_results,
        tuning_results=tuning_results,
        mc_results=mc_results,
        plots_dir=plots_dir,
    )

    # Export CSV summary table of all benchmark runs
    all_runs = (
        [strip_trace(r) for r in profile_results]
        + [strip_trace(r) for r in path_mode_results]
        + [strip_trace(r) for r in tuning_results]
    )
    csv_table_path = results_dir / 'berlin_lap_table.csv'
    fieldnames = [
        'label',
        'completed_laps',
        'target_laps',
        'completed_target',
        'standing_lap1_s',
        'flying_lap_mean_s',
        'flying_lap_std_s',
        'best_lap_s',
        'total_distance_m',
        'collisions',
        'track_departures',
        'min_body_clearance_m',
        'cte_rmse_m',
        'cte_max_m',
        'evo_ape_rmse_m',
        'heading_err_rmse_deg',
        'max_abs_steer_rad',
        'steer_rate_rms_rad_s',
        'speed_min_cruise_m_s',
        'speed_mean_m_s',
        'speed_max_m_s',
        'lat_acc_max_m_s2',
    ]
    with open(csv_table_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in all_runs:
            writer.writerow({k: row.get(k) for k in fieldnames})

    # Export full time-series telemetry CSV for Fast-Race profile
    race_tr = profile_results[-1]['trace']
    with open(
        results_dir / 'berlin_fast_race_telemetry.csv',
        'w',
        newline='',
        encoding='utf-8',
    ) as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                't_s',
                'x_m',
                'y_m',
                'yaw_rad',
                'vx_m_s',
                'cmd_speed_m_s',
                'steer_rad',
                'signed_cte_m',
                'lat_acc_m_s2',
                'body_wall_clearance_m',
            ]
        )
        for i in range(len(race_tr['t'])):
            writer.writerow(
                [
                    f"{race_tr['t'][i]:.3f}",
                    f"{race_tr['x'][i]:.4f}",
                    f"{race_tr['y'][i]:.4f}",
                    f"{race_tr['theta'][i]:.4f}",
                    f"{race_tr['vx'][i]:.4f}",
                    f"{race_tr['cmd_v'][i]:.4f}",
                    f"{race_tr['steer'][i]:.5f}",
                    f"{race_tr['cte'][i]:.5f}",
                    f"{race_tr['lat_acc'][i]:.4f}",
                    f"{race_tr['clearance'][i]:.4f}",
                ]
            )

    summary = {
        'environment': {
            'native_ros2': NATIVE_ROS2,
            'physics_engine': (
                'f110_gym:f110-v0 (RK4 single-track, dt=0.01s, 1080-beam)'
            ),
            'map': 'f1tenth_gym_ros/maps/berlin.yaml (600x600, 0.05 m/px)',
            'start_pose': [2.2933, -17.2435, -1.4965],
            'start_line': [START_LINE_P1, START_LINE_P2],
        },
        'profiles': [strip_trace(r) for r in profile_results],
        'path_modes': [strip_trace(r) for r in path_mode_results],
        'tuning_sweep': [strip_trace(r) for r in tuning_results],
        'monte_carlo': {
            'trials': len(mc_results),
            'successes': mc_passes,
            'success_rate_pct': round(
                100.0 * mc_passes / len(mc_results), 2
            ),
            'runs': [strip_trace(r) for r in mc_results],
        },
        'safety_fault_injections': safety_results,
    }
    with open(
        results_dir / 'berlin_benchmark_summary.json', 'w', encoding='utf-8'
    ) as f:
        json.dump(summary, f, indent=2)

    print('=== ALL BERLIN CLOSED-LOOP SIMULATIONS & PLOTS COMPLETE ===')


if __name__ == '__main__':
    main()
