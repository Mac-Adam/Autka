#!/usr/bin/env python3
import math
import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

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

        self.path_pub = self.create_publisher(Path, "/local_path", 10)
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
        if len(boundary) == 0 or points <= 0:
            return []
        
        boundary = np.asarray(boundary, dtype=float)
        lengths_cum = np.zeros(len(boundary))
        for i in range(1,len(boundary)):
            lengths_cum[i] = lengths_cum[i-1] + np.linalg.norm(boundary[i] - boundary[i-1])
        lenghts = np.linspace(0,lengths_cum[-1],points)
        res = []
        
        for l in lenghts:
            
            high = int(np.searchsorted(lengths_cum, l, side="right"))
            if high == len(boundary):
                res.append(boundary[-1].copy())
                continue
            
            low = high - 1
            
            t = (l - lengths_cum[low]) / (lengths_cum[high] - lengths_cum[low])
            res.append(boundary[low] + t*(boundary[high]-boundary[low]))
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
            
        right_boundary = right_boundary[::-1].copy()
            
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
            
        Segments = []
        w_max = 1.8
        w_track = 1.48
        if len(long_boundary) >= 2 and len(short_boundary) > 0:


            for i_l in range(len(long_boundary)):
                distances = np.linalg.norm(
                    short_boundary - long_boundary[i_l],
                    axis=1,
                )
                
                k = int(np.argmin(distances)) 
                if distances[k] < w_max:
                    Segments.append((long_boundary[i_l],short_boundary[k]))
                else:
                    tangents = np.gradient(long_boundary, axis=0)
                    lengths = np.linalg.norm(tangents, axis=1)
                    if np.any(lengths < 1e-9):
                        break
                    tangents = tangents / lengths[:, None]
                    
                    # Rotate each tangent by 90 degrees.
                    normals = np.column_stack((
                        -tangents[:, 1],
                        tangents[:, 0],
                    ))[i_l:]
                    
                    R_long = long_boundary[i_l:]
                    
                    if Segments:
                        toward_short = Segments[-1][1] - Segments[-1][0]
                    else:
                        toward_short = short_boundary[k] - long_boundary[i_l]
                        
                    if np.dot(normals[0], toward_short) < 0:
                        normals = -normals
                        
                    R_short = R_long + normals * w_track

                    Segments.extend(zip(R_long, R_short))
                    break
            
        centers = [ (a + b)/2 for (a,b) in Segments]
        centers = self.resample(centers,50)
     
        markers = MarkerArray()
        markers.markers = [
            self.make_boundary_marker(
                scan.header, short_boundary, 0, (0.0, 1.0, 0.0)
            ),
            self.make_boundary_marker(
                scan.header, long_boundary, 1, (0.0, 0.0, 1.0)
            ),
            self.make_boundary_marker(
                scan.header,centers,2,(0.0,1.0,1.0)
            )
        ]
        
        self.boundaries_pub.publish(markers)

        path_points = list(centers)
        path = Path()
        path.header = scan.header

        for x, y in path_points:
            pose = PoseStamped()
            pose.header = scan.header

            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            
            pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        self.path_pub.publish(path)


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