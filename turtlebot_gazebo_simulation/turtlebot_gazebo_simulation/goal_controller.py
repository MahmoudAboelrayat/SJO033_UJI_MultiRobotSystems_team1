#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class GoalController(Node):

    def __init__(self):
        super().__init__('goal_controller_node')
        self.robot1_velocity_publisher = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.roboot1_odom_subscriber = self.create_subscription(Odometry, '/robot1/odom', self.odom_callback, 10)
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.p_controller)
        self.robot1_goal_position = (0.0, 0.0)
        self.kp_linear = 0.2
        self.kp_angular = 1.0
        self.distance_tolerance = 0.05
        self.orientation_tolerance = 0.05
        self.state = "Wait_signal"
        self.robot1_current_position_x = None
        self.robot1_current_position_y = None
        self.robot1_current_orientation = None
        self.robot1_current_yaw = None
        self.robot1_goal_orientation = None

    def odom_callback(self, msg):
        self.robot1_current_position_x = msg.pose.pose.position.x
        self.robot1_current_position_y = msg.pose.pose.position.y
        self.robot1_current_orientation = msg.pose.pose.orientation
        # Convert quaternion to Euler angles:
        self.robot1_current_yaw = self.quaternion_to_euler(self.robot1_current_orientation)
    
    def quaternion_to_euler(self, q):
        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp) #in radians
        #return math.degrees(yaw) #convert to degrees
        return yaw  # in radians

    def p_controller(self):
        # Make sure we have odometry data before controlling
        if (self.robot1_current_position_x is None or
            self.robot1_current_position_y is None or
            self.robot1_current_yaw is None):
            self.get_logger().warn("Waiting for odometry data...")
            return
        
        #once user press enter key, state change to rotate to goal orientation then move to goal position then stop
        if self.state == "Wait_signal":
            input("Press Enter to start moving to goal...")
            self.state = "Rotate_to_goal_orientation"
        elif self.state == "Rotate_to_goal_orientation":
            twist=Twist()
            # Calculate goal orientation
            delta_x = self.robot1_goal_position[0] - self.robot1_current_position_x
            delta_y = self.robot1_goal_position[1] - self.robot1_current_position_y
            self.robot1_goal_orientation = math.atan2(delta_y, delta_x)
            # Rotate towards goal orientation 
            robot1_angle_error = self.robot1_goal_orientation - self.robot1_current_yaw
            if abs(robot1_angle_error) > self.orientation_tolerance:
                twist.angular.z = self.kp_angular * robot1_angle_error
            else:
                twist.angular.z = 0.0
                self.state = "Move_to_goal_position"
            self.robot1_velocity_publisher.publish(twist)  # Stop rotation

        elif self.state == "Move_to_goal_position":
            twist=Twist()
            #calculate distance to goal
            distance_to_goal = math.sqrt((self.robot1_goal_position[0] - self.robot1_current_position_x) ** 2 +
                                         (self.robot1_goal_position[1] - self.robot1_current_position_y) ** 2)
            if distance_to_goal > self.distance_tolerance:
                twist.linear.x = self.kp_linear * distance_to_goal
            else:
                twist.linear.x = 0.0
                self.state = "Goal_reached"
            self.robot1_velocity_publisher.publish(twist)
        elif self.state == "Goal_reached":
            #loop to state wait signal
            self.get_logger().info("Goal position reached.")
            self.state = "Wait_signal"

def main(args=None):
    rclpy.init(args=args)
    goal_controller_node = GoalController()
    rclpy.spin(goal_controller_node)
    goal_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()