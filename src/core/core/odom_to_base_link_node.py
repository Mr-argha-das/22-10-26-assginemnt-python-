import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped

#!/usr/bin/env python3


class OdomToBaseLinkNode(Node):
    def __init__(self):
        super().__init__('odom_to_base_link_node')
        # Declare parameters
        self.declare_parameter('topics.odom_topic', '/ego_racecar/odom')
        self.declare_parameter('frames.map_frame', 'map')
        self.declare_parameter('frames.odom_frame', 'odom')
        self.declare_parameter('frames.base_link_frame',
                               'ego_racecar/base_link')

        # Get parameter values
        self.odom_topic = self.get_parameter(
            'topics.odom_topic').get_parameter_value().string_value
        self.map_frame = self.get_parameter(
            'frames.map_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter(
            'frames.odom_frame').get_parameter_value().string_value
        self.base_link_frame = self.get_parameter(
            'frames.base_link_frame').get_parameter_value().string_value

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribe to odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info('Odom to base_link node started')

    def odom_callback(self, msg):
        current_time = self.get_clock().now().to_msg()

        # Transform from map to odom (identity for now)
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = current_time
        map_to_odom.header.frame_id = self.map_frame
        map_to_odom.child_frame_id = self.odom_frame
        map_to_odom.transform.translation.x = 0.0
        map_to_odom.transform.translation.y = 0.0
        map_to_odom.transform.translation.z = 0.0
        map_to_odom.transform.rotation.x = 0.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0

        # Transform from odom to base_link
        odom_to_base_link = TransformStamped()
        odom_to_base_link.header.stamp = current_time
        odom_to_base_link.header.frame_id = self.odom_frame
        odom_to_base_link.child_frame_id = self.base_link_frame
        odom_to_base_link.transform.translation.x = msg.pose.pose.position.x
        odom_to_base_link.transform.translation.y = msg.pose.pose.position.y
        odom_to_base_link.transform.translation.z = msg.pose.pose.position.z
        odom_to_base_link.transform.rotation.x = msg.pose.pose.orientation.x
        odom_to_base_link.transform.rotation.y = msg.pose.pose.orientation.y
        odom_to_base_link.transform.rotation.z = msg.pose.pose.orientation.z
        odom_to_base_link.transform.rotation.w = msg.pose.pose.orientation.w

        # Broadcast transforms
        self.tf_broadcaster.sendTransform([map_to_odom, odom_to_base_link])


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
