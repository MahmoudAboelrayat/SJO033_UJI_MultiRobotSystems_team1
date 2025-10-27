#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import math
import time
import tf_transformations
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class CW_Motion_Controller(Node):
    def __init__(self):
        super().__init__('cw_motion_4_Robots_node')

        #self.sim_robot_ids=['robot1','robot2','robot3','robot4']
        self.real_robot_ids=['tb_107','tb_108','tb_109','tb_203'] #real robots should be in same order of relative positions as sim_robot_ids inside simulation

        self.kp_linear = 0.5
        self.kp_angular = 1.0
        self.distance_tolerance = 0.07
        self.orientation_tolerance = 0.03
        self.state = "Wait_signal"
        self.robot_current_positions_x = [0.0] * 4
        self.robot_current_positions_y = [0.0] * 4
        self.robot_current_yaws = [0.0] * 4

        #numeric offsets (map → odom) for each robot
        side_length = 1.0  # meters
        offset = side_length / 2.0
        self.robot_offsets = [
            (-offset, -offset, 0.0),  # robot 1
            (offset, -offset, 0.0),     # robot 2
            (offset, offset, 0.0),     # robot 3
            (-offset, offset, 0.0)   # robot 4
        ]

        #Initialize Static TF Broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()

        

        #create publishers for four robots using loop
        self.robot_velocity_publishers = [None] * 4
        for i in range(4):
            #self.robot_velocity_publishers[i] = self.create_publisher(Twist, f'/{self.sim_robot_ids[i]}/cmd_vel', 10)
            self.robot_velocity_publishers[i] = self.create_publisher(Twist, f'/{self.real_robot_ids[i]}/cmd_vel', 10)

        #create subscription for each robot odom using loop
        self.robot_odom_subscribers = [None] * 4
        for i in range(4):
            #self.robot_odom_subscribers=self.create_subscription(Odometry, f'/{self.sim_robot_ids[i]}/odom', self.create_odom_callback(i), 10)
            self.robot_odom_subscribers=self.create_subscription(Odometry, f'/{self.real_robot_ids[i]}/odom', self.create_odom_callback(i), 10)
        

        self.timer = self.create_timer(0.1, self.p_controller)
        self.get_logger().info("CW Motion Controller initialized with static TFs.")

   
    def publish_static_transforms(self):
        static_transforms = []
        for i, (dx, dy, dyaw) in enumerate(self.robot_offsets):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = f'{self.real_robot_ids[i]}/odom'
            #t.child_frame_id = f'{self.sim_robot_ids[i]}/odom'
            t.transform.translation.x = float(dx)
            t.transform.translation.y = float(dy)
            t.transform.translation.z = 0.0

            q = tf_transformations.quaternion_from_euler(0.0, 0.0, dyaw)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            static_transforms.append(t)

        self.static_broadcaster.sendTransform(static_transforms)
        self.get_logger().info("Published static map→odom transforms for all robots.")

   
    def create_odom_callback(self, idx):
        def odom_callback(msg):
            pos = msg.pose.pose.position
            q = msg.pose.pose.orientation
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

            # Static offset for this robot
            dx, dy, dyaw = self.robot_offsets[idx]

            # Apply rigid transform (odom → map)
            x_map = pos.x * math.cos(dyaw) - pos.y * math.sin(dyaw) + dx
            y_map = pos.x * math.sin(dyaw) + pos.y * math.cos(dyaw) + dy
            #x_map = pos.x + dx
            #y_map = pos.y + dy
            yaw_map = self.normalize_angle(yaw + dyaw)

            # Save transformed position
            self.robot_current_positions_x[idx] = x_map
            self.robot_current_positions_y[idx] = y_map
            self.robot_current_yaws[idx] = yaw_map
        return odom_callback

   
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    
    def initialize_goal_positions(self):
        start_x = self.robot_current_positions_x.copy()
        start_y = self.robot_current_positions_y.copy()
        start_yaw = self.robot_current_yaws.copy()
        self.get_logger().info(f"start positions X: {start_x}")
        self.get_logger().info(f"start positions Y: {start_y}")
        cw_goal_idx = [3, 0, 1, 2]  # Each robot moves to next one clockwise
        self.goal_positions_x = [start_x[j] for j in cw_goal_idx]
        self.goal_positions_y = [start_y[j] for j in cw_goal_idx]
        self.goal_yaw = 0.0

        self.get_logger().info(f"Goal positions X: {self.goal_positions_x}")
        self.get_logger().info(f"Goal positions Y: {self.goal_positions_y}")
        

  
    def p_controller(self):
        rate = 10
        for i in range(4):
            if self.robot_current_positions_x[i] is None or self.robot_current_positions_x[i]==0.0: #re + map/odom or odom subscribe?
                return

        if self.state == "Wait_signal":
            input("Press Enter to start moving clockwise...")
            self.initialize_goal_positions()
            self.state = "Rotate_to_goal_orientation"
            

        elif self.state == "Rotate_to_goal_orientation":
            oriented_flags = [False] * 4
            for i in range(4):
                twist = Twist()
                dx = self.goal_positions_x[i] - self.robot_current_positions_x[i]
                dy = self.goal_positions_y[i] - self.robot_current_positions_y[i]
                goal_orientation = math.atan2(dy, dx)
                angle_error = self.normalize_angle(goal_orientation - self.robot_current_yaws[i])

                if abs(angle_error) > self.orientation_tolerance:
                    twist.angular.z = max(min(self.kp_angular * angle_error, 0.25), -0.25)
                else:
                    twist.angular.z = 0.0
                    twist.angular.x = 0.0
                    oriented_flags[i] = True

                self.robot_velocity_publishers[i].publish(twist)

            if all(oriented_flags):
                self.get_logger().info("All robots oriented to goal direction.")
                self.state = "Move_to_goal_position"
            time.sleep(1.0 / rate)

        elif self.state == "Move_to_goal_position":
            self.get_logger().info("Move_to_goal_position.")
            reached_flags = [False] * 4
            for i in range(4):
                twist = Twist()
                dx = self.goal_positions_x[i] - self.robot_current_positions_x[i]
                dy = self.goal_positions_y[i] - self.robot_current_positions_y[i]
                distance = math.sqrt(dx ** 2 + dy ** 2)
                self.get_logger().info(f"distancce = {distance}")
                
                if distance >= self.distance_tolerance:
                    self.get_logger().info("distance is bigger than threshold")
                    goal_orientation = math.atan2(dy, dx)
                    angle_error = self.normalize_angle(goal_orientation - self.robot_current_yaws[i])
                    twist.angular.z = max(min(self.kp_angular * angle_error, 0.25), -0.25)
                    #twist.angular.z = 0.0
                    twist.linear.x = max(min(self.kp_linear * distance, 0.05), 0.01)
                    self.get_logger().info(f"speed = {twist.linear.x}")
                else:
                    self.get_logger().info("distance is smaller than threshold")
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    reached_flags[i] = True
                self.robot_velocity_publishers[i].publish(twist)

            if all(reached_flags):
                self.get_logger().info("All robots reached their goal positions.")
                self.state = "Goal_reached"
            time.sleep(1.0 / rate)

        elif self.state == "Goal_reached":
            stop = Twist()
            for pub in self.robot_velocity_publishers:
                pub.publish(stop)
            self.get_logger().info("Goal reached, waiting for next signal.")
	     #while True:
            #for i in range(4):
             #   twist = Twist()
              #  goal_orientation = self.goal_yaw
               # angle_error = self.normalize_angle(goal_orientation[i] - self.robot_current_yaws[i])

                #if abs(angle_error) > self.orientation_tolerance:
                 #   twist.angular.z = max(min(self.kp_angular * angle_error, 0.25), -0.25)
                #else:
                 #   twist.angular.z = 0.0
                  #  twist.angular.x = 0.0
            
            self.state = "Wait_signal"
            time.sleep(0.3)

def main(args=None):
    rclpy.init(args=args)
    node = CW_Motion_Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
