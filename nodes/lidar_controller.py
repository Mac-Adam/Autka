#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class LidarController(Node):
    def __init__(self):
        super().__init__("lidar_controller")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

    def scan_callback(self, scan):
        # Read the nearest valid LiDAR return in a narrow, forward-facing sector.
        front_ranges = [
            distance
            for i, distance in enumerate(scan.ranges)
            if abs(scan.angle_min + i * scan.angle_increment) <= math.radians(10)
            and math.isfinite(distance)
            and scan.range_min <= distance <= scan.range_max
        ]
        front_distance = min(front_ranges, default=float("inf"))
        self.get_logger().debug(f"Nearest distance ahead: {front_distance:.2f} m")

        cmd = Twist()
        if front_distance < 1:
          cmd.linear.x = 0.0
        else:
          cmd.linear.x = 0.4
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LidarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()