#!/usr/bin/env python3 


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


class CW_Motion_Controller(Node):

    def __init__(self):
        super().__init__('cw_motion_4_Robots_node')

        sim_robot_ids=['robot1','robot2','robot3','robot4']
        #real_robot_ids=['tb_108','tb_109','tb_110','tb_203'] #real robots should be in same order of relative positions as sim_robot_ids inside simulation

        #create publishers for four robots using loop
        self.robot_velocity_publishers = [None] * 4
        for i in range(4):
            self.robot_velocity_publishers[i] = self.create_publisher(Twist, f'/{sim_robot_ids[i]}/cmd_vel', 10)
            #self.robot_velocity_publisher[i] = self.create_publisher(Twist, f'/{real_robot_ids[i]}/cmd_vel', 10)

        #create subscription for each robot odom using loop
        self.roboot_odom_subscribers = [None] * 4
        for i in range(4):
            self.robot_odom_subscribers=self.create_subscription(Odometry, f'/{sim_robot_ids[i]}/odom', self.create_odom_callback(i), 10)
            #self.roboot_odom_subscribers=self.create_subscription(Odometry, f'/{real_robot_ids[i]}/odom', self.create_odom_callback(i), 10)
        
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.p_controller)
        
        self.robot1_goal_position = (0.0, 0.0)
        self.kp_linear = 0.5
        self.kp_angular = 1.0
        self.distance_tolerance = 0.05
        self.orientation_tolerance = 0.03
        self.state = "Wait_signal"
        self.robot_current_positions_x = [0.0]*4
        self.robot_current_positions_y = [0.0]*4
        self.robot_current_orientations = [0.0]*4
        self.robot_current_yaws = [0.0]*4
        self.robot_goal_orientation = 0.0

    def create_odom_callback(self, idx):
        def odom_callback(msg):
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_current_positions_x[idx] = msg.pose.pose.position.x
            self.robot_current_positions_y[idx] = msg.pose.pose.position.y
            self.robot_current_yaws[idx] = math.atan2(siny_cosp, cosy_cosp)
        return odom_callback

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def initialize_goal_positions(self):
        # Store the goal positions for each robot (clockwise neighbors)
        self.goal_positions_x = [0.0] * 4
        self.goal_positions_y = [0.0] * 4
        #get four robots current positions as the starting positions for them 
        start_positions_x = self.robot_current_positions_x.copy()
        start_positions_y = self.robot_current_positions_y.copy()
        #define the four robots goal positions (cw neighbours positions) in clockwise manner
        cw_goal_idx = [3, 0, 1, 2]
        self.goal_positions_x = [start_positions_x[j] for j in cw_goal_idx]
        self.goal_positions_y = [start_positions_y[j] for j in cw_goal_idx]

    def p_controller(self):
        rate = 10  # Hz
        # Make sure we have odometry data from the four robots before controlling
        for i in range(4):
            if (self.robot_current_positions_x[i] is None or
                self.robot_current_positions_y[i] is None or
                self.robot_current_yaws[i] is None):
                self.get_logger().warn("Waiting for odometry data...")
                return


        #once user press enter key, state change to rotate to goal orientation then move to goal position then stop
        if self.state == "Wait_signal":
            input("Press Enter to start moving clockwise...")
            self.state = "Rotate_to_goal_orientation"
            self.initialize_goal_positions()


        elif self.state == "Rotate_to_goal_orientation":
            oriented_flags = [False] * 4  # Track if each robot's orientation bacame the goal orientation
            for i in range(4):
                twist=Twist()
                # Calculate goal orientation
                delta_x = self.goal_positions_x[i] - self.robot_current_positions_x[i]
                delta_y = self.goal_positions_y[i] - self.robot_current_positions_y[i]
                robot_goal_orientation = math.atan2(delta_y, delta_x)
                # Rotate towards goal orientation 
                robot_angle_error = robot_goal_orientation - self.robot_current_yaws[i]
                robot_angle_error = self.normalize_angle(robot_angle_error)
                if abs(robot_angle_error) > self.orientation_tolerance:
                    #twist.angular.z = self.kp_angular * robot_angle_error
                    twist.angular.z = max(min(self.kp_angular * robot_angle_error, 0.5), -0.5)
                    oriented_flags[i] = False
                else:
                    twist.angular.z = 0.0
                    oriented_flags[i] = True
                self.robot_velocity_publishers[i].publish(twist) 
            # Check if all robots are oriented
            if all(oriented_flags):
                    self.get_logger().info("All robots are now oriented towards their goals.")
                    self.state = "Move_to_goal_position"
            time.sleep(1.0 / rate)
                    

        elif self.state == "Move_to_goal_position":
            reached_flags = [False] * 4  # Track if each robot reached its goal position
            for i in range(4):
                twist=Twist()
                #calculate distance to goal
                distance_to_goal = math.sqrt((self.goal_positions_x[i] - self.robot_current_positions_x[i]) ** 2 +
                                            (self.goal_positions_y[i] - self.robot_current_positions_y[i]) ** 2)
                if distance_to_goal > self.distance_tolerance:
                    delta_x = self.goal_positions_x[i] - self.robot_current_positions_x[i]
                    delta_y = self.goal_positions_y[i] - self.robot_current_positions_y[i]
                    robot_goal_orientation = math.atan2(delta_y, delta_x)
                    robot_angle_error = robot_goal_orientation - self.robot_current_yaws[i]
                    robot_angle_error = self.normalize_angle(robot_angle_error)
                    twist.angular.z = max(min(self.kp_angular * robot_angle_error, 0.3), -0.3)
                    twist.linear.x = max(min(self.kp_linear * distance_to_goal, 0.25), 0.05)
                    #twist.linear.x = self.kp_linear * distance_to_goal
                    reached_flags[i] = False
                else:
                    twist.linear.x = 0.0
                    reached_flags[i] = True
                self.robot_velocity_publishers[i].publish(twist) 
            time.sleep(1.0 / rate)
            # Check if all robots have reached their goal positions
            if all(reached_flags):
                self.get_logger().info("All robots have reached their goal positions.")
                self.state = "Goal_reached"
            

        elif self.state == "Goal_reached":
            #loop to state wait signal
            self.get_logger().info("Goal position reached.")
            self.state = "Wait_signal"
            #Stop all robots briefly
            stop = Twist()
            for pub in self.robot_velocity_publishers:
                pub.publish(stop)
            time.sleep(0.3)



def main(args=None):
    rclpy.init(args=args)
    goal_controller_node = CW_Motion_Controller()
    rclpy.spin(goal_controller_node)
    goal_controller_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()