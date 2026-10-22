# MIT License
# Copyright (c) 2020 Hongrui Zheng
# Adapted for GIU Berlin AME RoboRacer Competition (ROS 2 Humble)

import os
import tempfile
import shutil

import gym
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
    TransformStamped,
    Transform,
)
from ackermann_msgs.msg import AckermannDriveStamped
from transforms3d import euler
from tf2_ros import TransformBroadcaster

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover
    get_package_share_directory = None


def resolve_gym_map_path(raw_map_path: str, map_ext: str) -> str:
    """
    Resolve map stem path and ensure 8-bit grayscale encoding for f110_gym.

    Upstream GIU-F1Tenth/f1tenth_maps stores berlin.png as a 1-bit PNG
    (mode='1'), which ScanSimulator2D.set_map() casts directly to {0.0, 1.0},
    treating 100% of pixels as occupied walls unless converted to mode='L'.
    """
    candidates = [raw_map_path]
    if not os.path.isabs(raw_map_path):
        candidates.append(os.path.abspath(raw_map_path))
        if get_package_share_directory is not None:
            try:
                share_dir = get_package_share_directory('f1tenth_gym_ros')
                candidates.append(os.path.join(share_dir, raw_map_path))
            except Exception:
                pass
    pkg_local = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__), '..', raw_map_path.lstrip('/')
        )
    )
    candidates.append(pkg_local)
    candidates.append(
        os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                '..',
                'maps',
                os.path.basename(raw_map_path),
            )
        )
    )

    resolved = None
    for stem in candidates:
        if os.path.exists(stem + '.yaml') and os.path.exists(stem + map_ext):
            resolved = stem
            break
    if resolved is None:
        raise FileNotFoundError(
            f"Map assets not found for '{raw_map_path}' ({map_ext}). "
            f"Checked: {candidates}"
        )

    img = Image.open(resolved + map_ext)
    if img.mode == '1':
        tmp_dir = tempfile.mkdtemp(prefix='f1tenth_map_')
        stem_name = os.path.basename(resolved)
        norm_stem = os.path.join(tmp_dir, stem_name)
        arr = np.where(np.asarray(img, dtype=bool), 255, 0).astype(np.uint8)
        Image.fromarray(arr, mode='L').save(norm_stem + map_ext)
        shutil.copyfile(resolved + '.yaml', norm_stem + '.yaml')
        return norm_stem

    return resolved


class GymBridge(Node):
    """ROS 2 bridge wrapping f110_gym physics and raycast simulator."""

    def __init__(self):
        super().__init__('gym_bridge')

        self.declare_parameter('ego_namespace', 'ego_racecar')
        self.declare_parameter('ego_odom_topic', 'odom')
        self.declare_parameter('ego_opp_odom_topic', 'opp_odom')
        self.declare_parameter('ego_scan_topic', '/scan')
        self.declare_parameter('ego_drive_topic', '/drive')
        self.declare_parameter('opp_namespace', 'opp_racecar')
        self.declare_parameter('opp_odom_topic', 'odom')
        self.declare_parameter('opp_ego_odom_topic', 'opp_odom')
        self.declare_parameter('opp_scan_topic', '/opp_scan')
        self.declare_parameter('opp_drive_topic', '/opp_drive')
        self.declare_parameter('scan_distance_to_base_link', 0.275)
        self.declare_parameter('scan_fov', 4.7)
        self.declare_parameter('scan_beams', 1080)
        self.declare_parameter('map_path', 'maps/berlin')
        self.declare_parameter('map_img_ext', '.png')
        self.declare_parameter('num_agent', 1)
        self.declare_parameter('sx', 2.2933)
        self.declare_parameter('sy', -17.2435)
        self.declare_parameter('stheta', -1.4965)
        self.declare_parameter('sx1', 2.1600)
        self.declare_parameter('sy1', -15.6500)
        self.declare_parameter('stheta1', -1.4965)
        self.declare_parameter('kb_teleop', True)
        self.declare_parameter('publish_map_to_base_tf', True)

        num_agents = self.get_parameter('num_agent').value
        if (
            not isinstance(num_agents, int)
            or num_agents < 1
            or num_agents > 2
        ):
            raise ValueError('num_agent must be an integer in {1, 2}.')

        map_ext = self.get_parameter('map_img_ext').value
        resolved_map = resolve_gym_map_path(
            self.get_parameter('map_path').value, map_ext
        )
        self.publish_map_to_base_tf = bool(
            self.get_parameter('publish_map_to_base_tf').value
        )

        self.env = gym.make(
            'f110_gym:f110-v0',
            map=resolved_map,
            map_ext=map_ext,
            num_agents=num_agents,
            lidar_dist=self.get_parameter('scan_distance_to_base_link').value,
        )

        sx = float(self.get_parameter('sx').value)
        sy = float(self.get_parameter('sy').value)
        stheta = float(self.get_parameter('stheta').value)
        self.ego_pose = [sx, sy, stheta]
        self.ego_speed = [0.0, 0.0, 0.0]
        self.ego_requested_speed = 0.0
        self.ego_steer = 0.0
        self.ego_collision = False

        ego_scan_topic = self.get_parameter('ego_scan_topic').value
        ego_drive_topic = self.get_parameter('ego_drive_topic').value
        scan_fov = float(self.get_parameter('scan_fov').value)
        scan_beams = int(self.get_parameter('scan_beams').value)
        self.angle_min = -scan_fov / 2.0
        self.angle_max = scan_fov / 2.0
        self.angle_inc = scan_fov / scan_beams
        self.ego_namespace = self.get_parameter('ego_namespace').value
        ego_odom_param = self.get_parameter('ego_odom_topic').value
        ego_odom_topic = (
            ego_odom_param
            if ego_odom_param.startswith('/')
            else f'/{self.ego_namespace}/{ego_odom_param}'
        )
        self.scan_distance_to_base_link = float(
            self.get_parameter('scan_distance_to_base_link').value
        )

        if num_agents == 2:
            self.has_opp = True
            self.opp_namespace = self.get_parameter('opp_namespace').value
            sx1 = float(self.get_parameter('sx1').value)
            sy1 = float(self.get_parameter('sy1').value)
            stheta1 = float(self.get_parameter('stheta1').value)
            self.opp_pose = [sx1, sy1, stheta1]
            self.opp_speed = [0.0, 0.0, 0.0]
            self.opp_requested_speed = 0.0
            self.opp_steer = 0.0
            self.opp_collision = False
            init_poses = np.array([[sx, sy, stheta], [sx1, sy1, stheta1]])
            self.obs, _, self.done, _ = self.env.reset(init_poses)
            self.ego_scan = [float(v) for v in self.obs['scans'][0]]
            self.opp_scan = [float(v) for v in self.obs['scans'][1]]

            opp_scan_topic = self.get_parameter('opp_scan_topic').value
            opp_drive_topic = self.get_parameter('opp_drive_topic').value
            opp_odom_name = self.get_parameter('opp_odom_topic').value
            opp_odom_topic = f'/{self.opp_namespace}/{opp_odom_name}'
            ego_opp_odom_topic = (
                f"/{self.ego_namespace}/"
                f"{self.get_parameter('ego_opp_odom_topic').value}"
            )
            opp_ego_odom_topic = (
                f"/{self.opp_namespace}/"
                f"{self.get_parameter('opp_ego_odom_topic').value}"
            )
        else:
            self.has_opp = False
            self.obs, _, self.done, _ = self.env.reset(
                np.array([[sx, sy, stheta]])
            )
            self.ego_scan = [float(v) for v in self.obs['scans'][0]]
        self._update_sim_state()

        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
        self.timer = self.create_timer(0.004, self.timer_callback)
        self.br = TransformBroadcaster(self)

        self.ego_scan_pub = self.create_publisher(
            LaserScan, ego_scan_topic, 10
        )
        self.ego_odom_pub = self.create_publisher(
            Odometry, ego_odom_topic, 10
        )
        self.ego_drive_published = False

        if self.has_opp:
            self.opp_scan_pub = self.create_publisher(
                LaserScan, opp_scan_topic, 10
            )
            self.ego_opp_odom_pub = self.create_publisher(
                Odometry, ego_opp_odom_topic, 10
            )
            self.opp_odom_pub = self.create_publisher(
                Odometry, opp_odom_topic, 10
            )
            self.opp_ego_odom_pub = self.create_publisher(
                Odometry, opp_ego_odom_topic, 10
            )
            self.opp_drive_published = False

        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped, ego_drive_topic, self.drive_callback, 10
        )
        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.ego_reset_callback,
            10,
        )

        if self.has_opp:
            self.opp_drive_sub = self.create_subscription(
                AckermannDriveStamped,
                opp_drive_topic,
                self.opp_drive_callback,
                10,
            )
            self.opp_reset_sub = self.create_subscription(
                PoseStamped, '/goal_pose', self.opp_reset_callback, 10
            )

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist, '/cmd_vel', self.teleop_callback, 10
            )

        self.get_logger().info(
            f"GymBridge initialized on map '{resolved_map}' at pose "
            f"({sx:.3f}, {sy:.3f}, {stheta:.3f} rad)"
        )

    def drive_callback(self, drive_msg: AckermannDriveStamped):
        self.ego_requested_speed = float(drive_msg.drive.speed)
        self.ego_steer = float(drive_msg.drive.steering_angle)
        self.ego_drive_published = True

    def opp_drive_callback(self, drive_msg: AckermannDriveStamped):
        self.opp_requested_speed = float(drive_msg.drive.speed)
        self.opp_steer = float(drive_msg.drive.steering_angle)
        self.opp_drive_published = True

    def ego_reset_callback(self, pose_msg: PoseWithCovarianceStamped):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rq = pose_msg.pose.pose.orientation
        _, _, rtheta = euler.quat2euler([rq.w, rq.x, rq.y, rq.z], axes='sxyz')
        if self.has_opp:
            opp_pose = [
                self.obs['poses_x'][1],
                self.obs['poses_y'][1],
                self.obs['poses_theta'][1],
            ]
            self.obs, _, self.done, _ = self.env.reset(
                np.array([[rx, ry, rtheta], opp_pose])
            )
        else:
            self.obs, _, self.done, _ = self.env.reset(
                np.array([[rx, ry, rtheta]])
            )
        self._update_sim_state()

    def opp_reset_callback(self, pose_msg: PoseStamped):
        if self.has_opp:
            rx = pose_msg.pose.position.x
            ry = pose_msg.pose.position.y
            rq = pose_msg.pose.orientation
            _, _, rtheta = euler.quat2euler(
                [rq.w, rq.x, rq.y, rq.z], axes='sxyz'
            )
            self.obs, _, self.done, _ = self.env.reset(
                np.array([list(self.ego_pose), [rx, ry, rtheta]])
            )
            self._update_sim_state()

    def teleop_callback(self, twist_msg: Twist):
        self.ego_drive_published = True
        self.ego_requested_speed = twist_msg.linear.x
        if twist_msg.angular.z > 0.0:
            self.ego_steer = 0.3
        elif twist_msg.angular.z < 0.0:
            self.ego_steer = -0.3
        else:
            self.ego_steer = 0.0

    def drive_timer_callback(self):
        if self.ego_drive_published and not self.has_opp:
            action = np.array([[self.ego_steer, self.ego_requested_speed]])
            self.obs, _, self.done, _ = self.env.step(action)
        elif (
            self.ego_drive_published
            and self.has_opp
            and self.opp_drive_published
        ):
            action = np.array(
                [
                    [self.ego_steer, self.ego_requested_speed],
                    [self.opp_steer, self.opp_requested_speed],
                ]
            )
            self.obs, _, self.done, _ = self.env.step(action)
        self._update_sim_state()

    def timer_callback(self):
        ts = self.get_clock().now().to_msg()

        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = f'{self.ego_namespace}/laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.0
        scan.range_max = 30.0
        scan.ranges = self.ego_scan
        self.ego_scan_pub.publish(scan)

        if self.has_opp:
            opp_scan = LaserScan()
            opp_scan.header.stamp = ts
            opp_scan.header.frame_id = f'{self.opp_namespace}/laser'
            opp_scan.angle_min = self.angle_min
            opp_scan.angle_max = self.angle_max
            opp_scan.angle_increment = self.angle_inc
            opp_scan.range_min = 0.0
            opp_scan.range_max = 30.0
            opp_scan.ranges = self.opp_scan
            self.opp_scan_pub.publish(opp_scan)

        self._publish_odom(ts)
        if self.publish_map_to_base_tf:
            self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    def _update_sim_state(self):
        self.ego_scan = [float(v) for v in self.obs['scans'][0]]
        self.ego_collision = bool(self.obs['collisions'][0])
        if self.has_opp:
            self.opp_scan = [float(v) for v in self.obs['scans'][1]]
            self.opp_collision = bool(self.obs['collisions'][1])
            self.opp_pose[0] = float(self.obs['poses_x'][1])
            self.opp_pose[1] = float(self.obs['poses_y'][1])
            self.opp_pose[2] = float(self.obs['poses_theta'][1])
            self.opp_speed[0] = float(self.obs['linear_vels_x'][1])
            self.opp_speed[1] = float(self.obs['linear_vels_y'][1])
            self.opp_speed[2] = float(self.obs['ang_vels_z'][1])

        self.ego_pose[0] = float(self.obs['poses_x'][0])
        self.ego_pose[1] = float(self.obs['poses_y'][0])
        self.ego_pose[2] = float(self.obs['poses_theta'][0])
        self.ego_speed[0] = float(self.obs['linear_vels_x'][0])
        self.ego_speed[1] = float(self.obs['linear_vels_y'][0])
        self.ego_speed[2] = float(self.obs['ang_vels_z'][0])

    def _publish_odom(self, ts):
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = 'map'
        ego_odom.child_frame_id = f'{self.ego_namespace}/base_link'
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        ego_quat = euler.euler2quat(0.0, 0.0, self.ego_pose[2], axes='sxyz')
        ego_odom.pose.pose.orientation.x = float(ego_quat[1])
        ego_odom.pose.pose.orientation.y = float(ego_quat[2])
        ego_odom.pose.pose.orientation.z = float(ego_quat[3])
        ego_odom.pose.pose.orientation.w = float(ego_quat[0])
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        self.ego_odom_pub.publish(ego_odom)

        if self.has_opp:
            opp_odom = Odometry()
            opp_odom.header.stamp = ts
            opp_odom.header.frame_id = 'map'
            opp_odom.child_frame_id = f'{self.opp_namespace}/base_link'
            opp_odom.pose.pose.position.x = self.opp_pose[0]
            opp_odom.pose.pose.position.y = self.opp_pose[1]
            opp_quat = euler.euler2quat(
                0.0, 0.0, self.opp_pose[2], axes='sxyz'
            )
            opp_odom.pose.pose.orientation.x = float(opp_quat[1])
            opp_odom.pose.pose.orientation.y = float(opp_quat[2])
            opp_odom.pose.pose.orientation.z = float(opp_quat[3])
            opp_odom.pose.pose.orientation.w = float(opp_quat[0])
            opp_odom.twist.twist.linear.x = self.opp_speed[0]
            opp_odom.twist.twist.linear.y = self.opp_speed[1]
            opp_odom.twist.twist.angular.z = self.opp_speed[2]
            self.opp_odom_pub.publish(opp_odom)
            self.opp_ego_odom_pub.publish(ego_odom)
            self.ego_opp_odom_pub.publish(opp_odom)

    def _publish_transforms(self, ts):
        ego_t = Transform()
        ego_t.translation.x = self.ego_pose[0]
        ego_t.translation.y = self.ego_pose[1]
        ego_t.translation.z = 0.0
        ego_quat = euler.euler2quat(0.0, 0.0, self.ego_pose[2], axes='sxyz')
        ego_t.rotation.x = float(ego_quat[1])
        ego_t.rotation.y = float(ego_quat[2])
        ego_t.rotation.z = float(ego_quat[3])
        ego_t.rotation.w = float(ego_quat[0])

        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp = ts
        ego_ts.header.frame_id = 'map'
        ego_ts.child_frame_id = f'{self.ego_namespace}/base_link'
        self.br.sendTransform(ego_ts)

        if self.has_opp:
            opp_t = Transform()
            opp_t.translation.x = self.opp_pose[0]
            opp_t.translation.y = self.opp_pose[1]
            opp_t.translation.z = 0.0
            opp_quat = euler.euler2quat(
                0.0, 0.0, self.opp_pose[2], axes='sxyz'
            )
            opp_t.rotation.x = float(opp_quat[1])
            opp_t.rotation.y = float(opp_quat[2])
            opp_t.rotation.z = float(opp_quat[3])
            opp_t.rotation.w = float(opp_quat[0])

            opp_ts = TransformStamped()
            opp_ts.transform = opp_t
            opp_ts.header.stamp = ts
            opp_ts.header.frame_id = 'map'
            opp_ts.child_frame_id = f'{self.opp_namespace}/base_link'
            self.br.sendTransform(opp_ts)

    def _publish_wheel_transforms(self, ts):
        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = euler.euler2quat(
            0.0, 0.0, self.ego_steer, axes='sxyz'
        )
        ego_wheel_ts.transform.rotation.x = float(ego_wheel_quat[1])
        ego_wheel_ts.transform.rotation.y = float(ego_wheel_quat[2])
        ego_wheel_ts.transform.rotation.z = float(ego_wheel_quat[3])
        ego_wheel_ts.transform.rotation.w = float(ego_wheel_quat[0])
        ego_wheel_ts.header.stamp = ts
        ego_wheel_ts.header.frame_id = (
            f'{self.ego_namespace}/front_left_hinge'
        )
        ego_wheel_ts.child_frame_id = f'{self.ego_namespace}/front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)
        ego_wheel_ts.header.frame_id = (
            f'{self.ego_namespace}/front_right_hinge'
        )
        ego_wheel_ts.child_frame_id = (
            f'{self.ego_namespace}/front_right_wheel'
        )
        self.br.sendTransform(ego_wheel_ts)

        if self.has_opp:
            opp_wheel_ts = TransformStamped()
            opp_wheel_quat = euler.euler2quat(
                0.0, 0.0, self.opp_steer, axes='sxyz'
            )
            opp_wheel_ts.transform.rotation.x = float(opp_wheel_quat[1])
            opp_wheel_ts.transform.rotation.y = float(opp_wheel_quat[2])
            opp_wheel_ts.transform.rotation.z = float(opp_wheel_quat[3])
            opp_wheel_ts.transform.rotation.w = float(opp_wheel_quat[0])
            opp_wheel_ts.header.stamp = ts
            opp_wheel_ts.header.frame_id = (
                f'{self.opp_namespace}/front_left_hinge'
            )
            opp_wheel_ts.child_frame_id = (
                f'{self.opp_namespace}/front_left_wheel'
            )
            self.br.sendTransform(opp_wheel_ts)
            opp_wheel_ts.header.frame_id = (
                f'{self.opp_namespace}/front_right_hinge'
            )
            opp_wheel_ts.child_frame_id = (
                f'{self.opp_namespace}/front_right_wheel'
            )
            self.br.sendTransform(opp_wheel_ts)

    def _publish_laser_transforms(self, ts):
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        ego_scan_ts.transform.rotation.w = 1.0
        ego_scan_ts.header.stamp = ts
        ego_scan_ts.header.frame_id = f'{self.ego_namespace}/base_link'
        ego_scan_ts.child_frame_id = f'{self.ego_namespace}/laser'
        self.br.sendTransform(ego_scan_ts)

        if self.has_opp:
            opp_scan_ts = TransformStamped()
            opp_scan_ts.transform.translation.x = (
                self.scan_distance_to_base_link
            )
            opp_scan_ts.transform.rotation.w = 1.0
            opp_scan_ts.header.stamp = ts
            opp_scan_ts.header.frame_id = f'{self.opp_namespace}/base_link'
            opp_scan_ts.child_frame_id = f'{self.opp_namespace}/laser'
            self.br.sendTransform(opp_scan_ts)


def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    try:
        rclpy.spin(gym_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        gym_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
