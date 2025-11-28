#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class ReactiveWaypointController(Node):
    def __init__(self):
        super().__init__('reactive_waypoint_controller_small_map')

        # ===== Parameters tuned for small/narrow map =====
        self.declare_parameter('goal_tolerance', 0.05)      # smaller goal tolerance
        self.declare_parameter('lookahead_distance', 0.2)   # shorter lookahead
        self.declare_parameter('max_linear', 0.1)           # slower forward
        self.declare_parameter('max_angular', 0.1)         # slower turning
        self.declare_parameter('side_clearance', 0.15)      # distance from walls on sides
        self.declare_parameter('path_clearance', 0.1)       # path-based lookahead clearance

        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.max_linear = self.get_parameter('max_linear').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular').get_parameter_value().double_value
        self.side_clearance = self.get_parameter('side_clearance').get_parameter_value().double_value
        self.path_clearance = self.get_parameter('path_clearance').get_parameter_value().double_value

        # ===== Publishers & Subscribers =====
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/plan', self.path_callback, 10)

        # ===== State =====
        self.laser_data = None
        self.current_pose = None
        self.path = []
        self.current_waypoint_idx = 0
        self.rotate_velocity = 0.0
        self.move_forward_velocity = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("ReactiveWaypointController with side and path clearance started.")

    def laser_callback(self, msg):
        self.laser_data = msg

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def path_callback(self, msg):
        self.path = msg.poses
        self.current_waypoint_idx = 0
        self.get_logger().info(f"Received path with {len(self.path)} waypoints")

    def control_loop(self):
        if self.laser_data is None or self.current_pose is None or len(self.path) == 0:
            return

        # ===== Obstacle avoidance =====
        self.simple_obstacle_avoidance(self.laser_data)

        # ===== Waypoint following =====
        pos = self.current_pose.position
        yaw = self.get_yaw(self.current_pose)

        # Select lookahead waypoint
        waypoint = self.path[self.current_waypoint_idx].pose.position
        for i in range(self.current_waypoint_idx, len(self.path)):
            dx = self.path[i].pose.position.x - pos.x
            dy = self.path[i].pose.position.y - pos.y
            dist = math.hypot(dx, dy)
            if dist >= self.lookahead_distance or i == len(self.path)-1:
                waypoint = self.path[i].pose.position
                self.current_waypoint_idx = i
                break

        # Check path clearance around lookahead waypoint
        if self.is_path_blocked(waypoint):
            self.rotate_velocity = 0.15  # rotate away from wall
            self.move_forward_velocity = 0.0
        else:
            self.rotate_velocity = 0.0
            self.move_forward_velocity = self.max_linear

        dx = waypoint.x - pos.x
        dy = waypoint.y - pos.y
        dist = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        heading_error = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))

        # Adjust forward speed based on heading
        if abs(heading_error) > 0.3:
            target_linear = 0.0
            target_angular = max(-self.max_angular, min(self.max_angular, 2.0 * heading_error))
        else:
            target_linear = self.move_forward_velocity
            target_angular = max(-self.max_angular, min(self.max_angular, 2.0 * heading_error))

        # Goal handling
        if self.current_waypoint_idx == len(self.path) - 1 and dist < self.goal_tolerance:
            target_linear = 0.0
            target_angular = 0.0
            self.path = []
            self.get_logger().info("Goal reached.")

        # Publish velocities
        twist = Twist()
        twist.linear.x = target_linear
        twist.angular.z = target_angular
        self.cmd_pub.publish(twist)

    def simple_obstacle_avoidance(self, msg):
        """Obstacle avoidance with forward and side checks."""
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=3.5, posinf=3.5)
        obstacle_detected = False

        # Forward arc: 330°–30°
        forward_indices = list(range(330, 360)) + list(range(0, 31))
        if np.min(ranges[forward_indices]) < self.side_clearance:
            obstacle_detected = True

        # Left side: 60°–120°
        left_indices = list(range(60, 121))
        if np.min(ranges[left_indices]) < self.side_clearance:
            obstacle_detected = True

        # Right side: 240°–300°
        right_indices = list(range(240, 301))
        if np.min(ranges[right_indices]) < self.side_clearance:
            obstacle_detected = True

        if obstacle_detected:
            self.rotate_velocity = 0.15  # turn away
            self.move_forward_velocity = 0.0
        else:
            self.rotate_velocity = 0.0
            self.move_forward_velocity = self.max_linear

    def is_path_blocked(self, waypoint):
        """Check for obstacles around the waypoint using LIDAR data."""
        if self.laser_data is None:
            return False

        pos = self.current_pose.position
        yaw = self.get_yaw(self.current_pose)
        ranges = np.array(self.laser_data.ranges)
        ranges = np.nan_to_num(ranges, nan=3.5, posinf=3.5)

        dx = waypoint.x - pos.x
        dy = waypoint.y - pos.y
        dist = math.hypot(dx, dy)
        angle_to_wp = math.atan2(dy, dx) - yaw
        angle_to_wp = (angle_to_wp + math.pi*2) % (2*math.pi)  # normalize 0–2pi
        angle_deg = int(np.degrees(angle_to_wp)) % 360

        # Check ±15° around waypoint direction
        check_indices = [(angle_deg + i) % 360 for i in range(-15, 16)]
        if np.min(ranges[check_indices]) < self.path_clearance:
            return True
        return False

    def get_yaw(self, pose):
        q = pose.orientation
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveWaypointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Keyboard interrupt detected")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()