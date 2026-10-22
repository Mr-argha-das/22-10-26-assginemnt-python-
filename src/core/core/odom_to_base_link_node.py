#!/usr/bin/env python3
"""
Odometry-to-TF Bridge Node for AROLA Core Layer.

Prevents duplicate-parent TF2 tree conflicts across simulation modes:
- When `publish_tf=False` (default with `f1tenth_gym_ros` ground-truth TF):
  monitors odometry without re-broadcasting `ego_racecar/base_link` under a
  competing `odom` parent frame.
- When `publish_tf=True` and `publish_map_to_odom=True`: broadcasts complete
  `map -> odom -> ego_racecar/base_link` chain.
- When `publish_tf=True` and `publish_map_to_odom=False` (AMCL active):
  broadcasts `odom -> ego_racecar/base_link` while `amcl` owns `map -> odom`.
"""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import tf2_ros


class OdomToBaseLinkNode(Node):
    """Configurable TF broadcaster bridging `/ego_racecar/odom` into TF2."""

    def __init__(self):
        super().__init__('odom_to_base_link_node')
        self.declare_parameter('topics.odom_topic', '/ego_racecar/odom')
        self.declare_parameter('frames.map_frame', 'map')
        self.declare_parameter('frames.odom_frame', 'odom')
        self.declare_parameter(
            'frames.base_link_frame', 'ego_racecar/base_link'
        )
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('publish_map_to_odom', True)

        self.odom_topic = (
            self.get_parameter('topics.odom_topic')
            .get_parameter_value()
            .string_value
        )
        self.map_frame = (
            self.get_parameter('frames.map_frame')
            .get_parameter_value()
            .string_value
        )
        self.odom_frame = (
            self.get_parameter('frames.odom_frame')
            .get_parameter_value()
            .string_value
        )
        self.base_link_frame = (
            self.get_parameter('frames.base_link_frame')
            .get_parameter_value()
            .string_value
        )
        self.publish_tf = (
            self.get_parameter('publish_tf').get_parameter_value().bool_value
        )
        self.publish_map_to_odom = (
            self.get_parameter('publish_map_to_odom')
            .get_parameter_value()
            .bool_value
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_count = 0
        self.odom_subscriber = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.get_logger().info(
            f'OdomToBaseLinkNode initialized (publish_tf={self.publish_tf}, '
            f'publish_map_to_odom={self.publish_map_to_odom}).'
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.odom_count += 1
        if not self.publish_tf:
            return

        current_time = self.get_clock().now().to_msg()
        transforms = []

        if self.publish_map_to_odom:
            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = current_time
            map_to_odom.header.frame_id = self.map_frame
            map_to_odom.child_frame_id = self.odom_frame
            map_to_odom.transform.rotation.w = 1.0
            transforms.append(map_to_odom)

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = current_time
        odom_to_base.header.frame_id = self.odom_frame
        odom_to_base.child_frame_id = self.base_link_frame
        odom_to_base.transform.translation.x = msg.pose.pose.position.x
        odom_to_base.transform.translation.y = msg.pose.pose.position.y
        odom_to_base.transform.translation.z = msg.pose.pose.position.z
        odom_to_base.transform.rotation.x = msg.pose.pose.orientation.x
        odom_to_base.transform.rotation.y = msg.pose.pose.orientation.y
        odom_to_base.transform.rotation.z = msg.pose.pose.orientation.z
        odom_to_base.transform.rotation.w = msg.pose.pose.orientation.w
        transforms.append(odom_to_base)

        self.tf_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToBaseLinkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
