#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import pandas as pd
import os


class SimplePathPublisher(Node):
    def __init__(self):
        super().__init__('simple_path_publisher')

        self.declare_parameter('csv_path', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('invert', False)

        csv_path = self.get_parameter(
            'csv_path').get_parameter_value().string_value
        frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value
        self.invert = self.get_parameter(
            'invert').get_parameter_value().bool_value

        self.publisher = self.create_publisher(Path, '/path', 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_path)

        self.path_msg = Path()
        self.path_msg.header.frame_id = frame_id

        if csv_path and os.path.exists(csv_path):
            self.load_path_from_csv(csv_path)
        else:
            self.get_logger().error(f"CSV path not found: {csv_path}")

    def load_path_from_csv(self, csv_path):
        try:
            df = pd.read_csv(csv_path, names=['x', 'y', 'v'])

            for _, row in df.iterrows():
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = self.path_msg.header.frame_id
                pose_stamped.pose.position.x = float(row['x'])
                pose_stamped.pose.position.y = float(row['y'])
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.x = 0.0
                pose_stamped.pose.orientation.y = 0.0
                pose_stamped.pose.orientation.z = 0.0
                pose_stamped.pose.orientation.w = float(
                    row['v'])  # Velocity encoded in w

                self.path_msg.poses.append(pose_stamped)

            self.get_logger().info(
                f"Loaded {len(self.path_msg.poses)} waypoints from {csv_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to load CSV: {str(e)}")

    def publish_path(self):
        if self.path_msg.poses:
            self.path_msg.header.stamp = self.get_clock().now().to_msg()

            if self.invert:
                self.path_msg.poses.reverse()

            self.publisher.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimplePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
