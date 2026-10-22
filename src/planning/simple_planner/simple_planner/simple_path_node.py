#!/usr/bin/env python3
"""
Simple Path Node for AROLA (Autonomous Racing Open Layered Architecture)

This node reads waypoints from a CSV file and publishes them as a Path message.
The velocity is encoded in the angular z (w) field of each pose in the path.

Expected CSV format: x_m, y_m, v_m/s (with optional header comments starting with #)
"""

import csv
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math


class SimplePathNode(Node):
    """Node that publishes a path from a CSV file with velocity encoded in angular.z"""

    def __init__(self):
        super().__init__('simple_path_node')

        # Declare parameters
        self.declare_parameter('csv_path', '')
        self.declare_parameter('invert', False)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('path_topic', '/pp_path')

        # Get parameters
        self.csv_path = self.get_parameter(
            'csv_path').get_parameter_value().string_value
        self.invert = self.get_parameter(
            'invert').get_parameter_value().bool_value
        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value
        self.path_topic = self.get_parameter(
            'path_topic').get_parameter_value().string_value

        # Validate parameters
        if not self.csv_path:
            self.get_logger().error(
                "csv_path parameter is empty. Please provide a valid CSV file path.")
            return

        if not os.path.isabs(self.csv_path):
            original_path = self.csv_path
            found = False
            
            if os.path.exists(self.csv_path):
                self.csv_path = os.path.abspath(self.csv_path)
                found = True
            
            if not found and 'ROS_WORKSPACE' in os.environ:
                workspace_path = os.path.join(os.environ['ROS_WORKSPACE'], original_path)
                if os.path.exists(workspace_path):
                    self.csv_path = workspace_path
                    found = True
                    self.get_logger().info(
                        f"Resolved relative path '{original_path}' via ROS_WORKSPACE to: {self.csv_path}")
            
            if not found:
                self.get_logger().error(
                    f"CSV file not found. Tried relative path '{original_path}' from: "
                    f"current directory and ROS_WORKSPACE.")
                return
        else:
            if not os.path.exists(self.csv_path):
                self.get_logger().error(f"CSV file not found: {self.csv_path}")
                return

        # Create publisher
        self.path_publisher = self.create_publisher(Path, self.path_topic, 10)

        # Load waypoints from CSV
        self.waypoints = self.load_waypoints_from_csv()
        if not self.waypoints:
            self.get_logger().error("No valid waypoints loaded from CSV file")
            return

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints from {self.csv_path}")

        # Create timer to publish path at specified rate
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_path)

        self.get_logger().info(
            f"Simple Path Node initialized. Publishing at {self.publish_rate} Hz")

    def load_waypoints_from_csv(self):
        """Load waypoints from CSV file"""
        waypoints = []

        try:
            with open(self.csv_path, 'r') as csvfile:
                # Skip comment lines that start with #
                lines = []
                for line in csvfile:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        lines.append(line)

                # Parse CSV data
                reader = csv.reader(lines)
                for row_num, row in enumerate(reader, 1):
                    try:
                        if len(row) >= 3:
                            x = float(row[0].strip())
                            y = float(row[1].strip())
                            v = float(row[2].strip())
                            waypoints.append((x, y, v))
                        else:
                            self.get_logger().warn(
                                f"Row {row_num} has insufficient columns: {row}")
                    except ValueError as e:
                        self.get_logger().warn(
                            f"Error parsing row {row_num}: {row} - {e}")

        except Exception as e:
            self.get_logger().error(f"Error reading CSV file: {e}")
            return []

        # Invert waypoints if requested
        if self.invert:
            waypoints.reverse()
            self.get_logger().info("Waypoint order inverted")

        return waypoints

    def publish_path(self):
        """Publish the path message"""
        if not self.waypoints:
            return

        # Create Path message
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id

        # Add each waypoint as a PoseStamped
        for i, (x, y, v) in enumerate(self.waypoints):
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header

            # Set position
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            # IMPORTANT: Encode velocity in angular.z field as specified
            pose_stamped.pose.orientation.w = v

            path_msg.poses.append(pose_stamped)

        # Publish the path
        self.path_publisher.publish(path_msg)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    try:
        node = SimplePathNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in simple_path_node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
