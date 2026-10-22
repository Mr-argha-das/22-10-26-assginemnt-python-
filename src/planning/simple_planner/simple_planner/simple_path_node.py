#!/usr/bin/env python3
"""
Simple Path Node for AROLA (Autonomous Racing Open Layered Architecture).

Loads or generates validated waypoints for the F1TENTH track (including direct
occupancy-grid planning on the GIU Berlin map) and publishes `nav_msgs/Path`
on `/pp_path` with target speed encoded in `pose.orientation.w` (preserving
full backward compatibility with `pure_pursuit_node`) plus RViz visualization
markers on `/planned_path_marker`.
"""

import csv
import os
from typing import List, Optional, Tuple

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

from .map_utils import OccupancyGridMap
from .path_planner import plan_berlin_path
from .path_validator import deduplicate_waypoints, validate_path

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover
    get_package_share_directory = None


def resolve_resource_path(
    raw_path: str, package_name: str = 'simple_planner'
) -> str:
    """Resolve path across CWD, ROS_WORKSPACE, package share, and source."""
    if not raw_path:
        return ''
    if os.path.isabs(raw_path) and os.path.exists(raw_path):
        return raw_path

    candidates = [os.path.abspath(raw_path)]
    if 'ROS_WORKSPACE' in os.environ:
        candidates.append(os.path.join(os.environ['ROS_WORKSPACE'], raw_path))
    if get_package_share_directory is not None:
        try:
            share_dir = get_package_share_directory(package_name)
            candidates.append(os.path.join(share_dir, raw_path))
            candidates.append(
                os.path.join(share_dir, 'paths', os.path.basename(raw_path))
            )
            candidates.append(
                os.path.join(share_dir, 'maps', os.path.basename(raw_path))
            )
        except Exception:
            pass

    pkg_src_root = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..')
    )
    candidates.append(os.path.join(pkg_src_root, raw_path))
    candidates.append(
        os.path.join(pkg_src_root, 'paths', os.path.basename(raw_path))
    )
    candidates.append(
        os.path.join(pkg_src_root, 'maps', os.path.basename(raw_path))
    )

    for candidate in candidates:
        if os.path.exists(candidate):
            return os.path.abspath(candidate)
    return raw_path


def parse_waypoints_csv_file(
    csv_path: str,
    invert: bool = False,
    speed_scale: float = 1.0,
    closed_loop: bool = True,
) -> List[Tuple[float, float, float]]:
    """Parse `(x, y, v)` CSV rows, skipping `#` comments and header rows."""
    raw_waypoints: List[Tuple[float, float, float]] = []
    with open(csv_path, 'r', encoding='utf-8') as csvfile:
        lines = [
            line.strip()
            for line in csvfile
            if line.strip() and not line.strip().startswith('#')
        ]

    reader = csv.reader(lines)
    for row in reader:
        if len(row) < 3:
            continue
        try:
            x = float(row[0].strip())
            y = float(row[1].strip())
            v = float(row[2].strip()) * float(speed_scale)
            raw_waypoints.append((x, y, v))
        except ValueError:
            # Skip non-numeric header lines cleanly
            continue

    waypoints = deduplicate_waypoints(
        raw_waypoints, min_dist_m=1e-3, closed_loop=closed_loop
    )
    if invert:
        waypoints.reverse()
    return waypoints


class SimplePathNode(Node):
    """AROLA Planning Layer ROS 2 node publishing validated reference paths."""

    def __init__(self):
        super().__init__('simple_planner')

        self.declare_parameter('planner_mode', 'csv')
        self.declare_parameter(
            'csv_path', 'paths/berlin_optimal_raceline.csv'
        )
        self.declare_parameter('map_yaml_path', 'maps/berlin.yaml')
        self.declare_parameter('invert', False)
        self.declare_parameter('closed_loop', True)
        self.declare_parameter('validate_against_map', True)
        self.declare_parameter('vehicle_width_m', 0.31)
        self.declare_parameter('safety_margin_m', 0.45)
        self.declare_parameter('waypoint_spacing_m', 0.20)
        self.declare_parameter('v_min', 1.6)
        self.declare_parameter('v_max', 4.2)
        self.declare_parameter('a_lat_max', 4.0)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('path_topic', '/pp_path')
        self.declare_parameter('marker_topic', '/planned_path_marker')

        self.planner_mode = (
            self.get_parameter('planner_mode')
            .get_parameter_value()
            .string_value
        )
        raw_csv = (
            self.get_parameter('csv_path').get_parameter_value().string_value
        )
        raw_map = (
            self.get_parameter('map_yaml_path')
            .get_parameter_value()
            .string_value
        )
        self.invert = (
            self.get_parameter('invert').get_parameter_value().bool_value
        )
        self.closed_loop = (
            self.get_parameter('closed_loop').get_parameter_value().bool_value
        )
        self.validate_against_map = (
            self.get_parameter('validate_against_map')
            .get_parameter_value()
            .bool_value
        )
        self.vehicle_width_m = (
            self.get_parameter('vehicle_width_m')
            .get_parameter_value()
            .double_value
        )
        self.safety_margin_m = (
            self.get_parameter('safety_margin_m')
            .get_parameter_value()
            .double_value
        )
        self.spacing_m = (
            self.get_parameter('waypoint_spacing_m')
            .get_parameter_value()
            .double_value
        )
        self.v_min = (
            self.get_parameter('v_min').get_parameter_value().double_value
        )
        self.v_max = (
            self.get_parameter('v_max').get_parameter_value().double_value
        )
        self.a_lat_max = (
            self.get_parameter('a_lat_max').get_parameter_value().double_value
        )
        self.speed_scale = (
            self.get_parameter('speed_scale')
            .get_parameter_value()
            .double_value
        )
        self.frame_id = (
            self.get_parameter('frame_id').get_parameter_value().string_value
        )
        self.publish_rate = (
            self.get_parameter('publish_rate')
            .get_parameter_value()
            .double_value
        )
        self.path_topic = (
            self.get_parameter('path_topic')
            .get_parameter_value()
            .string_value
        )
        self.marker_topic = (
            self.get_parameter('marker_topic')
            .get_parameter_value()
            .string_value
        )

        self.csv_path = resolve_resource_path(raw_csv)
        self.map_yaml_path = resolve_resource_path(raw_map)
        self.validation_report = None
        self.waypoints: List[Tuple[float, float, float]] = []

        self.path_publisher = self.create_publisher(Path, self.path_topic, 10)
        self.marker_publisher = self.create_publisher(
            Marker, self.marker_topic, 10
        )

        self._initialize_path()

        if self.waypoints:
            period = 1.0 / max(self.publish_rate, 0.1)
            self.timer = self.create_timer(period, self.publish_path)
            self.publish_path()
            self.get_logger().info(
                f"SimplePathNode active on '{self.path_topic}' "
                f'({len(self.waypoints)} pts, {self.publish_rate:.1f} Hz).'
            )

    def _initialize_path(self) -> None:
        grid_map: Optional[OccupancyGridMap] = None
        if self.validate_against_map and self.map_yaml_path:
            if os.path.exists(self.map_yaml_path):
                grid_map = OccupancyGridMap(self.map_yaml_path)
            else:
                self.get_logger().warn(
                    f'Map YAML not found for validation: {self.map_yaml_path}'
                )

        if self.planner_mode in ('min_curvature', 'harmonic_centerline'):
            if not self.map_yaml_path or not os.path.exists(
                self.map_yaml_path
            ):
                self.get_logger().error(
                    f"Map YAML required for planner_mode='{self.planner_mode}'"
                    f' not found: {self.map_yaml_path}'
                )
                return
            waypoints, grid_map = plan_berlin_path(
                yaml_path=self.map_yaml_path,
                mode=self.planner_mode,
                spacing_m=self.spacing_m,
                vehicle_width_m=self.vehicle_width_m,
                safety_margin_m=self.safety_margin_m,
                v_min=self.v_min,
                v_max=self.v_max,
                a_lat_max=self.a_lat_max,
            )
            if self.speed_scale != 1.0:
                waypoints = [
                    (x, y, v * self.speed_scale) for x, y, v in waypoints
                ]
            if self.invert:
                waypoints.reverse()
        else:
            if not self.csv_path or not os.path.exists(self.csv_path):
                self.get_logger().error(
                    f'CSV file not found: {self.csv_path}'
                )
                return
            waypoints = parse_waypoints_csv_file(
                self.csv_path,
                invert=self.invert,
                speed_scale=self.speed_scale,
                closed_loop=self.closed_loop,
            )

        report = validate_path(
            waypoints,
            grid_map=grid_map if self.validate_against_map else None,
            closed_loop=self.closed_loop,
            vehicle_width_m=self.vehicle_width_m,
            safety_margin_m=self.safety_margin_m,
        )
        self.validation_report = report
        if not report.is_valid:
            self.get_logger().error(
                f'Rejected invalid reference path: {report.errors}'
            )
            return

        self.waypoints = waypoints
        self.get_logger().info(
            f'Validated reference path: {report.num_waypoints} pts, '
            f'length={report.total_length_m:.2f}m, '
            f'min_clearance={report.min_wall_clearance_m:.3f}m, '
            f'v=[{report.min_speed_mps:.2f}, {report.max_speed_mps:.2f}] m/s'
        )

    def publish_path(self) -> None:
        """Publish `nav_msgs/Path` and speed-colored RViz `Marker`."""
        if not self.waypoints:
            return

        stamp = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.frame_id

        marker = Marker()
        marker.header = path_msg.header
        marker.ns = 'berlin_reference_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.pose.orientation.w = 1.0

        v_lo = min(wp[2] for wp in self.waypoints)
        v_hi = max(wp[2] for wp in self.waypoints)
        v_span = max(v_hi - v_lo, 1e-3)

        for x, y, v in self.waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = float(x)
            pose_stamped.pose.position.y = float(y)
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = float(v)
            path_msg.poses.append(pose_stamped)

            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.03
            marker.points.append(pt)

            norm = (v - v_lo) / v_span
            col = ColorRGBA()
            col.r = float(1.0 - norm)
            col.g = float(norm)
            col.b = 0.15
            col.a = 0.9
            marker.colors.append(col)

        if self.closed_loop and marker.points:
            marker.points.append(marker.points[0])
            marker.colors.append(marker.colors[0])

        self.path_publisher.publish(path_msg)
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = SimplePathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
