#!/usr/bin/env python3
"""
F1TENTH Watchdog Node for AROLA System Health Monitoring and Safety Stop.

Supports both full physical RoboRacer hardware (`vesc_msgs` battery voltage &
motor temperature telemetry) and simulation mode (`VESC_AVAILABLE=False`),
monitoring live `/scan` and `/ego_racecar/odom` message freshness on a periodic
timer and optionally publishing emergency stop `Bool(True)` to `/pause`.
"""

import time
from typing import Any, Optional

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String

try:
    from vesc_msgs.msg import VescStateStamped

    VESC_AVAILABLE = True
except ImportError:  # Simulation / CI environments without physical VESC pkg
    VescStateStamped = None
    VESC_AVAILABLE = False


class WatchdogNode(Node):
    """F1TENTH Watchdog Node for system health monitoring and fail-safe."""

    def __init__(self):
        super().__init__('watchdog_node')
        self._declare_parameters()
        self._load_parameters()
        self._initialize_state_variables()
        self._setup_subscriptions()
        self._setup_publishers()
        self._setup_timers()
        self.get_logger().info(
            f'WatchdogNode started (vesc_available={VESC_AVAILABLE}).'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('critical_voltage', 12.0)
        self.declare_parameter('min_voltage', 9.0)
        self.declare_parameter('max_voltage', 52.0)
        self.declare_parameter('temp_warning_start', 80.0)
        self.declare_parameter('temp_warning_high', 90.0)
        self.declare_parameter('temp_critical', 100.0)
        self.declare_parameter('lidar_timeout', 0.5)
        self.declare_parameter('camera_timeout', 0.5)
        self.declare_parameter('lidar_critical_timeout', 1.0)
        self.declare_parameter('status_publish_interval', 1.0)
        self.declare_parameter('camera_check_interval', 0.5)
        self.declare_parameter('critical_check_interval', 0.5)
        self.declare_parameter('enable_camera_check', False)
        self.declare_parameter('publish_pause_on_critical', True)
        self.declare_parameter('pause_topic', '/pause')
        self.declare_parameter('core_topic', '/sensors/core')
        self.declare_parameter(
            'camera_topic', '/camera/camera/color/image_raw'
        )
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('critical_topic', '/tmp/watchdog/critical')
        self.declare_parameter('status_topic', '/watchdog/system/status')
        self.declare_parameter(
            'camera_status_topic', '/tmp/watchdog/camera_is_live'
        )
        self.declare_parameter('subscription_qos_depth', 10)
        self.declare_parameter('publisher_qos_depth', 10)

    def _load_parameters(self) -> None:
        gp = self.get_parameter
        self.critical_voltage = (
            gp('critical_voltage').get_parameter_value().double_value
        )
        self.min_voltage = (
            gp('min_voltage').get_parameter_value().double_value
        )
        self.max_voltage = (
            gp('max_voltage').get_parameter_value().double_value
        )
        self.temp_warning_start = (
            gp('temp_warning_start').get_parameter_value().double_value
        )
        self.temp_warning_high = (
            gp('temp_warning_high').get_parameter_value().double_value
        )
        self.temp_critical = (
            gp('temp_critical').get_parameter_value().double_value
        )
        self.lidar_timeout = (
            gp('lidar_timeout').get_parameter_value().double_value
        )
        self.camera_timeout = (
            gp('camera_timeout').get_parameter_value().double_value
        )
        self.lidar_critical_timeout = (
            gp('lidar_critical_timeout').get_parameter_value().double_value
        )
        self.status_publish_interval = (
            gp('status_publish_interval').get_parameter_value().double_value
        )
        self.camera_check_interval = (
            gp('camera_check_interval').get_parameter_value().double_value
        )
        self.critical_check_interval = (
            gp('critical_check_interval').get_parameter_value().double_value
        )
        self.enable_camera_check = (
            gp('enable_camera_check').get_parameter_value().bool_value
        )
        self.publish_pause_on_critical = (
            gp('publish_pause_on_critical').get_parameter_value().bool_value
        )
        self.pause_topic = (
            gp('pause_topic').get_parameter_value().string_value
        )
        self.core_topic = gp('core_topic').get_parameter_value().string_value
        self.camera_topic = (
            gp('camera_topic').get_parameter_value().string_value
        )
        self.lidar_topic = (
            gp('lidar_topic').get_parameter_value().string_value
        )
        self.odom_topic = gp('odom_topic').get_parameter_value().string_value
        self.critical_topic = (
            gp('critical_topic').get_parameter_value().string_value
        )
        self.status_topic = (
            gp('status_topic').get_parameter_value().string_value
        )
        self.camera_status_topic = (
            gp('camera_status_topic').get_parameter_value().string_value
        )
        self.qos_depth = int(
            gp('subscription_qos_depth').get_parameter_value().integer_value
        )
        self.pub_qos_depth = int(
            gp('publisher_qos_depth').get_parameter_value().integer_value
        )

    def _initialize_state_variables(self) -> None:
        self.battery_voltage: float = 0.0
        self.motor_current: float = 0.0
        self.motor_temperature: float = 0.0
        self.motor_velocity: float = 0.0
        self.motor_angular_velocity: float = 0.0
        self.lidar_status: Optional[bool] = None
        self.motor_status: bool = False
        self.vesc_status: Optional[bool] = None
        self.camera_is_live: bool = True
        self.last_lidar_clock_ns: Optional[int] = None
        self.last_image_time: float = time.time()
        self.is_critical: bool = False

    def _setup_subscriptions(self) -> None:
        if VESC_AVAILABLE and VescStateStamped is not None:
            self.core_subscription = self.create_subscription(
                VescStateStamped,
                self.core_topic,
                self._core_callback,
                self.qos_depth,
            )
        else:
            self.core_subscription = None

        self.camera_subscription = self.create_subscription(
            Image, self.camera_topic, self._camera_callback, self.qos_depth
        )
        self.lidar_subscription = self.create_subscription(
            LaserScan, self.lidar_topic, self._lidar_callback, self.qos_depth
        )
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, self.qos_depth
        )

    def _setup_publishers(self) -> None:
        self.critical_publisher = self.create_publisher(
            Bool, self.critical_topic, self.pub_qos_depth
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, self.pub_qos_depth
        )
        self.camera_status_publisher = self.create_publisher(
            Bool, self.camera_status_topic, self.pub_qos_depth
        )
        self.pause_publisher = self.create_publisher(
            Bool, self.pause_topic, self.pub_qos_depth
        )

    def _setup_timers(self) -> None:
        self.status_timer = self.create_timer(
            self.status_publish_interval, self._publish_status_message
        )
        self.camera_check_timer = self.create_timer(
            self.camera_check_interval, self._check_camera_status
        )
        self.critical_timer = self.create_timer(
            self.critical_check_interval, self._publish_critical_status
        )
        if self.enable_camera_check:
            self.camera_status_timer = self.create_timer(
                self.critical_check_interval, self._publish_camera_status
            )

    def _odom_callback(self, msg: Odometry) -> None:
        self.motor_velocity = float(msg.twist.twist.linear.x)
        self.motor_angular_velocity = float(msg.twist.twist.angular.z)
        self.motor_status = not (
            abs(self.motor_velocity) < 1e-4
            and abs(self.motor_angular_velocity) < 1e-4
        )

    def _camera_callback(self, msg: Image) -> None:
        self.last_image_time = time.time()

    def _check_camera_status(self) -> None:
        if not self.enable_camera_check:
            return
        self.camera_is_live = (
            time.time() - self.last_image_time
        ) <= self.camera_timeout

    def _core_callback(self, msg: Any) -> None:
        self.battery_voltage = float(msg.state.voltage_input)
        self.motor_current = float(msg.state.current_motor)
        self.motor_temperature = float(msg.state.temp_motor)

        if self.battery_voltage < self.critical_voltage:
            self.get_logger().warn(
                f'Battery critically low: {self.battery_voltage:.2f}V'
            )
            self.is_critical = True

        if self.motor_temperature > self.temp_warning_start:
            self.get_logger().warn(
                f'High motor temperature: {self.motor_temperature:.1f}C'
            )
            self.is_critical = True

    def _lidar_callback(self, msg: LaserScan) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.last_lidar_clock_ns is not None:
            dt_s = (now_ns - self.last_lidar_clock_ns) * 1e-9
            self.lidar_status = dt_s <= self.lidar_timeout
            if dt_s > self.lidar_critical_timeout:
                self.is_critical = True
        else:
            self.lidar_status = True
        self.last_lidar_clock_ns = now_ns

    def _evaluate_lidar_silence(self) -> None:
        """Detect total LiDAR stream dropout even when zero callbacks fire."""
        if self.last_lidar_clock_ns is None:
            return
        age_s = (
            self.get_clock().now().nanoseconds - self.last_lidar_clock_ns
        ) * 1e-9
        if age_s > self.lidar_timeout:
            self.lidar_status = False
        if age_s > self.lidar_critical_timeout:
            self.is_critical = True

    def _generate_status_message(self) -> str:
        lines = ['F1TENTH System Status:']
        if self.battery_voltage == 0.0:
            lines.append('Battery: Simulated / No hardware telemetry')
        else:
            b_state = 'Normal'
            if self.battery_voltage < self.critical_voltage:
                b_state = 'CRITICAL'
            elif self.battery_voltage > self.max_voltage:
                b_state = 'HIGH'
            lines.append(f'Battery: {self.battery_voltage:.2f}V ({b_state})')
        motor_state = 'Active' if self.motor_status else 'Idle'
        lines.append(
            f'Motor: {motor_state} ({self.motor_velocity:.2f} m/s, '
            f'{self.motor_angular_velocity:.2f} rad/s)'
        )
        if self.lidar_status is None:
            lines.append('LiDAR: Waiting for /scan')
        else:
            l_state = 'Operational' if self.lidar_status else 'Faulty'
            lines.append(f'LiDAR: {l_state}')
        return '\n'.join(lines)

    def _publish_status_message(self) -> None:
        msg = String()
        msg.data = self._generate_status_message()
        self.status_publisher.publish(msg)

    def _publish_critical_status(self) -> None:
        self._evaluate_lidar_silence()
        msg = Bool()
        msg.data = self.is_critical
        self.critical_publisher.publish(msg)
        if self.is_critical and self.publish_pause_on_critical:
            pause_msg = Bool()
            pause_msg.data = True
            self.pause_publisher.publish(pause_msg)

    def _publish_camera_status(self) -> None:
        msg = Bool()
        msg.data = self.camera_is_live
        self.camera_status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
