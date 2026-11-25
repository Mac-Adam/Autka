import math
import time

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry, Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time

from tf2_ros import Buffer, TransformException, TransformListener


SPEED = 0.80
LOOKAHEAD_TIME = 1.8
MIN_LOOKAHEAD = 0.85
MAX_LOOKAHEAD = 1.50


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')
        self.declare_parameter('speed', SPEED)
        self.declare_parameter('lookahead_time', LOOKAHEAD_TIME)
        self.declare_parameter('min_lookahead', MIN_LOOKAHEAD)
        self.declare_parameter('max_lookahead', MAX_LOOKAHEAD)
        self.pose = None
        self.current_speed = 0.0
        self.path = Path()
        self.last_odom = self.last_path = 0.0
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.drive_command_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, qos_profile_sensor_data) #odom ma best effort
        self.path_sub = self.create_subscription(
            Path, '/local_path', self.on_path, 10) #bufor 10 wiadomości
        self.timer = self.create_timer(0.05, self.control)

    def on_odom(self, msg):
        if msg.header.frame_id != 'odom':
            self.pose = None
            return
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw(msg.pose.pose.orientation))
        self.current_speed = msg.twist.twist.linear.x
        self.last_odom = time.monotonic()

    def on_path(self, msg):
        self.path = msg
        self.last_path = time.monotonic()

    def control(self):
        now = time.monotonic()
        drive_command = Twist()
        speed = self.get_parameter('speed').value
        lookahead_time = self.get_parameter('lookahead_time').value
        min_lookahead = self.get_parameter('min_lookahead').value
        max_lookahead = self.get_parameter('max_lookahead').value
        if (
            ((not math.isfinite(speed)) or (speed < 0.0))
            or (not math.isfinite(self.current_speed))
            or ((not math.isfinite(lookahead_time)) or (lookahead_time <= 0.0))
            or ((not math.isfinite(min_lookahead)) or (min_lookahead <= 0.0))
            or ((not math.isfinite(max_lookahead))
                or (max_lookahead < min_lookahead))
            or (self.pose is None)
            or (not self.path.poses)
            or (not self.path.header.frame_id)
            or ((now - min(self.last_odom, self.last_path)) > 1.0)
        ):
            self.drive_command_pub.publish(drive_command) # zatrzymanie samochodu
            return
        lookahead = max(
            min_lookahead,
            min(max_lookahead, abs(self.current_speed) * lookahead_time))
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', self.path.header.frame_id,
                Time.from_msg(self.path.header.stamp)).transform
        except TransformException as exc:
            self.get_logger().warning(
                f'Waiting for path TF: {exc}', throttle_duration_sec=2.0)
            self.drive_command_pub.publish(drive_command) # zatrzymanie samochodu
            return

        x, y, heading = self.pose
        c, s = math.cos(heading), math.sin(heading)
        tc, ts = math.cos(yaw(tf.rotation)), math.sin(yaw(tf.rotation))
        forward_points = []
        for pose in self.path.poses:
            p = pose.pose.position
            dx = tf.translation.x + tc * p.x - ts * p.y - x
            dy = tf.translation.y + ts * p.x + tc * p.y - y
            forward, left = c * dx + s * dy, -s * dx + c * dy
            if (math.isfinite(forward) and math.isfinite(left)
                    and forward > 0.0
                    and forward * forward + left * left > 1e-6):
                forward_points.append((forward, left))

        if forward_points:
            candidates = [p for p in forward_points
                          if p[0]**2 + p[1]**2 >= lookahead ** 2]
            if candidates:
                forward, left = min(
                    candidates, key=lambda p: p[0]**2 + p[1]**2)
            else:
                forward, left = max(forward_points, key=lambda p: p[0])
            distance_squared = forward**2 + left**2
            curvature = 2 * left / distance_squared
            drive_command.linear.x = speed
            drive_command.angular.z = drive_command.linear.x * curvature
        else:
            self.get_logger().warning(
                'No valid waypoint ahead; stopping.',
                throttle_duration_sec=2.0)
        self.drive_command_pub.publish(drive_command)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PurePursuit()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.drive_command_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
