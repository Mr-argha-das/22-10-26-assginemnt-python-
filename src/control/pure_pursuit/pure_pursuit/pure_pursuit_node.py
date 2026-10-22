#!/usr/bin/env python3
"""
Pure Pursuit Controller ROS 2 Node for F1TENTH AROLA Autonomous Racing.

Preserves all upstream AROLA interfaces (`/pp_path`, `/ego_racecar/odom`,
`/drive`, `/pause`, `/joy`, `/lookahead_marker`, `/lookahead_circle`) while
adding kinematic wheelbase geometry, speed-adaptive continuous lookahead
interpolation, curvature-preview corner speed regulation, steering angle &
rate saturation, stale-odometry safety stop, and path-completion stop.
"""

import math
from typing import List, Optional, Tuple

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from .controller_core import (
    ControlOutput,
    ControllerParams,
    compute_adaptive_lookahead,
    compute_pure_pursuit_step,
    euler_from_quaternion,
    select_lookahead_point,
    sigmoid_velocity_profile,
    transform_to_vehicle_frame,
)


class PurePursuit(Node):
    """AROLA Control Layer Pure Pursuit ROS 2 node."""

    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Upstream AROLA parameters + safety/kinematic extensions
        self.declare_parameter('max_lookahead_distance', 1.95)
        self.declare_parameter('min_lookahead_distance', 0.75)
        self.declare_parameter('lookahead_gain', 0.28)
        self.declare_parameter('use_linear_lad_interp', True)
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('min_velocity', 1.6)
        self.declare_parameter('cmd_vel_topic', '/drive')
        self.declare_parameter('odometry_topic', '/ego_racecar/odom')
        self.declare_parameter('path_topic', '/pp_path')
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('steering_gain', 1.08)
        self.declare_parameter('kp', 0.0)
        self.declare_parameter('kd', 0.02)
        self.declare_parameter('steering_alpha', 0.72)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('max_steering_rate', 3.2)
        self.declare_parameter('a_lat_max', 4.6)
        self.declare_parameter('curvature_preview_m', 2.6)
        self.declare_parameter('k_sigmoid', 8.0)
        self.declare_parameter('skidding_velocity_thresh', 0.35)
        self.declare_parameter('vel_division_factor', 1.0)
        self.declare_parameter('autonomous_vel_enabled', True)
        self.declare_parameter('closed_loop_path', True)
        self.declare_parameter('goal_tolerance_m', 0.35)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('pose_source', 'tf_or_odom')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('pause_topic', '/pause')
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('tf_timeout', 0.05)
        self.declare_parameter('marker_resolution', 60)
        self.declare_parameter('subscriber_queue_size', 10)
        self.declare_parameter('publisher_queue_size', 10)
        self.declare_parameter('tf_target', 'map')
        self.declare_parameter('tf_source', 'ego_racecar/base_link')

        self._sync_params()

        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, self.queue_size
        )
        self.cmd_vel_pub = self.create_publisher(
            AckermannDriveStamped, self.cmd_vel_topic, self.pub_queue_size
        )
        self.path_sub = self.create_subscription(
            Path, self.path_topic, self.path_update_cb, self.queue_size
        )
        self.pause_sub = self.create_subscription(
            Bool, self.pause_topic, self.toggle_stop_cb, self.queue_size
        )
        self.joy_sub = self.create_subscription(
            Joy, self.joy_topic, self.joy_callback, self.queue_size
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 1.0 / max(self.control_frequency, 1.0)
        self.dt = timer_period
        self.timer = self.create_timer(timer_period, self.get_pose)

        self.stop = False
        self.prev_gamma = 0.0
        self.prev_steer = 0.0
        self.last_closest_idx: Optional[int] = None
        self.lookahead_distance = 0.0
        self.odometry = Odometry()
        self.has_received_odom = False
        self.last_odom_time_ns: Optional[int] = None
        self.last_output: Optional[ControlOutput] = None
        self.path: List[Tuple[float, float, float]] = []

        self.lookahead_marker_pub = self.create_publisher(
            Marker, '/lookahead_marker', self.pub_queue_size
        )
        self.lookahead_circle_pub = self.create_publisher(
            Marker, '/lookahead_circle', self.pub_queue_size
        )

        self.get_logger().info(
            f'PurePursuit initialized ({self.control_frequency:.1f} Hz, '
            f'autonomous_vel={self.activate_autonomous_vel}).'
        )

    def _sync_params(self) -> None:
        gp = self.get_parameter
        self.kp = gp('kp').get_parameter_value().double_value
        self.kd = gp('kd').get_parameter_value().double_value
        self.max_velocity = (
            gp('max_velocity').get_parameter_value().double_value
        )
        self.min_velocity = (
            gp('min_velocity').get_parameter_value().double_value
        )
        self.min_lad = (
            gp('min_lookahead_distance').get_parameter_value().double_value
        )
        self.max_lad = (
            gp('max_lookahead_distance').get_parameter_value().double_value
        )
        self.lookahead_gain = (
            gp('lookahead_gain').get_parameter_value().double_value
        )
        self.use_linear_lad = (
            gp('use_linear_lad_interp').get_parameter_value().bool_value
        )
        self.wheelbase = gp('wheelbase').get_parameter_value().double_value
        self.steering_gain = (
            gp('steering_gain').get_parameter_value().double_value
        )
        self.steering_alpha = (
            gp('steering_alpha').get_parameter_value().double_value
        )
        self.max_steering_angle = (
            gp('max_steering_angle').get_parameter_value().double_value
        )
        self.max_steering_rate = (
            gp('max_steering_rate').get_parameter_value().double_value
        )
        self.a_lat_max = gp('a_lat_max').get_parameter_value().double_value
        self.curvature_preview_m = (
            gp('curvature_preview_m').get_parameter_value().double_value
        )
        self.k_sigmoid = gp('k_sigmoid').get_parameter_value().double_value
        self.skidding_velocity_thresh = (
            gp('skidding_velocity_thresh').get_parameter_value().double_value
        )
        self.vel_division_factor = (
            gp('vel_division_factor').get_parameter_value().double_value
        )
        self.activate_autonomous_vel = (
            gp('autonomous_vel_enabled').get_parameter_value().bool_value
        )
        self.closed_loop_path = (
            gp('closed_loop_path').get_parameter_value().bool_value
        )
        self.goal_tolerance_m = (
            gp('goal_tolerance_m').get_parameter_value().double_value
        )
        self.odom_timeout = (
            gp('odom_timeout').get_parameter_value().double_value
        )
        self.pose_source = (
            gp('pose_source').get_parameter_value().string_value
        )
        self.cmd_vel_topic = (
            gp('cmd_vel_topic').get_parameter_value().string_value
        )
        self.odom_topic = (
            gp('odometry_topic').get_parameter_value().string_value
        )
        self.path_topic = gp('path_topic').get_parameter_value().string_value
        self.joy_topic = gp('joy_topic').get_parameter_value().string_value
        self.pause_topic = (
            gp('pause_topic').get_parameter_value().string_value
        )
        self.control_frequency = (
            gp('control_frequency').get_parameter_value().double_value
        )
        self.tf_timeout = gp('tf_timeout').get_parameter_value().double_value
        self.marker_resolution = int(
            gp('marker_resolution').get_parameter_value().integer_value
        )
        self.queue_size = int(
            gp('subscriber_queue_size').get_parameter_value().integer_value
        )
        self.pub_queue_size = int(
            gp('publisher_queue_size').get_parameter_value().integer_value
        )
        self.tf_target = gp('tf_target').get_parameter_value().string_value
        self.tf_source = gp('tf_source').get_parameter_value().string_value

    def build_controller_params(self) -> ControllerParams:
        """Pack current node attributes into a `ControllerParams` struct."""
        return ControllerParams(
            wheelbase=self.wheelbase,
            min_lookahead_distance=self.min_lad,
            max_lookahead_distance=self.max_lad,
            lookahead_gain=self.lookahead_gain,
            use_linear_lad_interp=self.use_linear_lad,
            min_velocity=self.min_velocity,
            max_velocity=self.max_velocity,
            vel_division_factor=self.vel_division_factor,
            a_lat_max=self.a_lat_max,
            curvature_preview_m=self.curvature_preview_m,
            k_sigmoid=self.k_sigmoid,
            skidding_velocity_thresh=self.skidding_velocity_thresh,
            steering_gain=self.steering_gain,
            kp=self.kp,
            kd=self.kd,
            steering_alpha=self.steering_alpha,
            max_steering_angle=self.max_steering_angle,
            max_steering_rate=self.max_steering_rate,
            closed_loop=self.closed_loop_path,
            goal_tolerance_m=self.goal_tolerance_m,
        )

    def publish_safe_stop(self) -> None:
        """Publish zero speed and zero steering safe-stop command."""
        stop_msg = AckermannDriveStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        self.prev_steer = 0.0
        self.prev_gamma = 0.0
        self.cmd_vel_pub.publish(stop_msg)

    def toggle_stop_cb(self, msg: Bool) -> None:
        """Emergency stop / pause callback on `/pause`."""
        self.stop = bool(msg.data)
        if self.stop:
            self.publish_safe_stop()
            self.get_logger().info(
                'Pure Pursuit paused; published safe stop.'
            )
        else:
            self.get_logger().info('Pure Pursuit resumed.')

    def joy_callback(self, msg: Joy) -> None:
        """Joystick parameter tuning and deadman enable callback."""
        btns = msg.buttons
        if len(btns) > 3 and btns[3] == 1:
            self.kd += 0.05
        if len(btns) > 1 and btns[1] == 1:
            self.kd = max(0.0, self.kd - 0.05)
        if len(btns) > 2 and btns[2] == 1:
            self.kp += 0.05
        if len(btns) > 0 and btns[0] == 1:
            self.kp = max(0.0, self.kp - 0.05)
        if len(btns) > 4:
            self.activate_autonomous_vel = btns[4] == 1

    def path_update_cb(self, msg: Path) -> None:
        """Update active `(x, y, v)` waypoint list from `/pp_path`."""
        new_path: List[Tuple[float, float, float]] = []
        for pose in msg.poses:
            px = float(pose.pose.position.x)
            py = float(pose.pose.position.y)
            pv = float(pose.pose.orientation.w)
            if math.isfinite(px) and math.isfinite(py) and math.isfinite(pv):
                new_path.append((px, py, pv))

        if len(new_path) != len(self.path):
            self.last_closest_idx = None
            self.get_logger().info(
                f'Path updated with {len(new_path)} validated points.'
            )
        self.path = new_path

    def odom_callback(self, msg: Odometry) -> None:
        """Store latest vehicle odometry and update freshness timestamp."""
        self.odometry = msg
        self.has_received_odom = True
        self.last_odom_time_ns = self.get_clock().now().nanoseconds

    def _is_odom_stale(self) -> bool:
        if not self.has_received_odom or self.last_odom_time_ns is None:
            return True
        now_ns = self.get_clock().now().nanoseconds
        age_sec = (now_ns - self.last_odom_time_ns) * 1e-9
        return age_sec > self.odom_timeout

    def _resolve_vehicle_pose(self) -> Optional[Tuple[float, float, float]]:
        """Obtain `(x, y, yaw)` from TF2 or fresh odometry fallback."""
        if self.pose_source in ('tf_only', 'tf_or_odom'):
            try:
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform(
                    self.tf_target,
                    self.tf_source,
                    now,
                    timeout=rclpy.duration.Duration(seconds=self.tf_timeout),
                )
                trans = transform.transform.translation
                rot = transform.transform.rotation
                _, _, yaw = euler_from_quaternion(
                    [rot.x, rot.y, rot.z, rot.w]
                )
                return float(trans.x), float(trans.y), float(yaw)
            except Exception as exc:
                if self.pose_source == 'tf_only':
                    self.get_logger().warn(f'Transform unavailable: {exc}')
                    return None

        if self.has_received_odom:
            pos = self.odometry.pose.pose.position
            ori = self.odometry.pose.pose.orientation
            try:
                _, _, yaw = euler_from_quaternion(
                    [ori.x, ori.y, ori.z, ori.w]
                )
                return float(pos.x), float(pos.y), float(yaw)
            except ValueError:
                return None
        return None

    def get_pose(self) -> None:
        """Periodic control loop callback."""
        if self._is_odom_stale():
            self.publish_safe_stop()
            return

        pose = self._resolve_vehicle_pose()
        if pose is None or not self.path:
            self.publish_safe_stop()
            return

        x, y, yaw = pose
        curr_speed = float(self.odometry.twist.twist.linear.x)
        out, self.prev_gamma, self.last_closest_idx = (
            compute_pure_pursuit_step(
                path=self.path,
                x=x,
                y=y,
                yaw=yaw,
                current_speed=curr_speed,
                dt=self.dt,
                params=self.build_controller_params(),
                prev_gamma=self.prev_gamma,
                prev_steer=self.prev_steer,
                last_closest_idx=self.last_closest_idx,
                autonomous_enabled=self.activate_autonomous_vel,
                paused=self.stop,
            )
        )
        self.last_output = out
        self.lookahead_distance = out.lookahead_distance
        self.prev_steer = out.steering_angle

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.drive.speed = float(out.speed)
        cmd.drive.steering_angle = float(out.steering_angle)
        self.cmd_vel_pub.publish(cmd)

        self.publish_lookahead_circle(x, y)
        if out.lookahead_point is not None:
            self.publish_lookahead_marker(out.lookahead_point)

    def get_lad_thresh(self, v: float) -> float:
        """Calculate adaptive lookahead distance `[m]` for speed `v`."""
        return compute_adaptive_lookahead(
            speed=v,
            min_lad=self.min_lad,
            max_lad=self.max_lad,
            min_velocity=self.min_velocity,
            max_velocity=self.max_velocity,
            lookahead_gain=(
                None if self.use_linear_lad else self.lookahead_gain
            ),
        )

    def find_lookahead_point(self, x: float, y: float):
        """Find `(lookahead_pt, closest_pt, target_idx)` along `self.path`."""
        return select_lookahead_point(
            path=self.path,
            x=x,
            y=y,
            lookahead_distance=self.lookahead_distance,
            last_closest_idx=self.last_closest_idx,
            closed_loop=self.closed_loop_path,
        )

    def transform_to_vehicle_frame(
        self, point, x: float, y: float, yaw: float
    ):
        """Transform global `(x, y)` point into vehicle body frame."""
        return transform_to_vehicle_frame(point, x, y, yaw)

    def smooth_vel(self, curr_vel: float, target_vel: float) -> float:
        """Apply acceleration step cap to prevent tire slip."""
        if (target_vel - curr_vel) > self.skidding_velocity_thresh:
            return curr_vel + self.skidding_velocity_thresh
        return target_vel

    def find_linear_vel_steering_controlled_sigmoidally(
        self, gamma: float
    ) -> float:
        """Calculate velocity using sigmoid function of curvature `gamma`."""
        return sigmoid_velocity_profile(
            gamma=gamma,
            v_min=self.min_velocity,
            v_max=self.max_velocity,
            k_sigmoid=self.k_sigmoid,
        )

    def publish_lookahead_marker(
        self, point: Tuple[float, float, float]
    ) -> None:
        marker = Marker()
        marker.header.frame_id = self.tf_target
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'pure_pursuit'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.0
        self.lookahead_marker_pub.publish(marker)

    def publish_lookahead_circle(self, x: float, y: float) -> None:
        marker = Marker()
        marker.header.frame_id = self.tf_target
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'pure_pursuit'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.color.a = 0.7
        marker.color.g = 0.8
        marker.color.b = 1.0

        res = max(12, self.marker_resolution)
        for i in range(res + 1):
            angle = 2.0 * math.pi * i / res
            pt = Point()
            pt.x = x + self.lookahead_distance * math.cos(angle)
            pt.y = y + self.lookahead_distance * math.sin(angle)
            pt.z = 0.05
            marker.points.append(pt)
        self.lookahead_circle_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_safe_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
