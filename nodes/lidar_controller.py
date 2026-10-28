#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray



class LidarController(Node):
    def __init__(self):
        super().__init__("lidar_controller")

        self.prev_left_b = []
        self.prev_right_b = []

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.boundaries_pub = self.create_publisher(
            MarkerArray, "/boundaries", 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )
        
    def make_boundary_marker(self, header, points, marker_id, color):
        marker = Marker()
        marker.header = header
        marker.ns = "boundaries"
        marker.id = marker_id
        marker.type = Marker.POINTS

        # Remove the previous marker if this boundary is now empty.
        marker.action = Marker.ADD if len(points) > 0 else Marker.DELETE

        marker.pose.orientation.w = 1.0

        # Point width and height, in meters.
        marker.scale.x = 0.06
        marker.scale.y = 0.06

        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        # Expects each item in points to contain (x, y).
        marker.points = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in points
        ]

        return marker

    def resample(self, boundary, points):
        if not boundary:
            return []
        idxs = np.linspace(0,len(boundary)-1,points)
        res = []
        for i in idxs:
            low = int(np.floor(i))
            high = int(np.ceil(i))
            res.append(boundary[low] + (i-low)*(boundary[high]-boundary[low]))
        return res

    def scan_callback(self, scan):
        left_boundary = []
        right_boundary = []
        split_angle = None
        last = None
        boundary_jump = 0.7
        short_points = 20
        long_points = 30
        

        for i, distance in enumerate(scan.ranges):
            # loof only forward
            angle = i * scan.angle_increment
            if abs(scan.angle_min + i * scan.angle_increment) >= math.radians(135) or not math.isfinite(distance) or not (scan.range_min <= distance <= scan.range_max - 0.1):
                continue
            x = distance*math.cos(scan.angle_min + i * scan.angle_increment)
            y = distance*math.sin(scan.angle_min + i * scan.angle_increment)
            curr = np.array([x,y])
            if last is None:
                last = np.array([x,y])
            dist = np.linalg.norm(curr-last)

            if split_angle is None and dist > boundary_jump:
                split_angle = angle + scan.angle_increment*0.5
                
            if split_angle is None:
                left_boundary.append(curr)
            else:
                right_boundary.append(curr)
            last = curr
            
        if len(left_boundary) < 2:
            left_boundary = self.prev_left_b
        if len(right_boundary) < 2:
            right_boundary = self.prev_right_b
        self.prev_left_b = left_boundary
        self.prev_right_b = right_boundary
            
        if np.linalg.norm(left_boundary[0] - left_boundary[-1]) < np.linalg.norm(right_boundary[0] - right_boundary[-1]):
            short_boundary = self.resample(left_boundary,short_points)
            long_boundary = self.resample(right_boundary,long_points)
        else:
            short_boundary = self.resample(right_boundary,short_points)
            long_boundary = self.resample(left_boundary,long_points)           
        markers = MarkerArray()
        markers.markers = [
            self.make_boundary_marker(
                scan.header, short_boundary, 0, (0.0, 1.0, 0.0)
            ),
            self.make_boundary_marker(
                scan.header, long_boundary, 1, (0.0, 0.0, 1.0)
            ),
        ]
        
        self.boundaries_pub.publish(markers)
        cmd = Twist()
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