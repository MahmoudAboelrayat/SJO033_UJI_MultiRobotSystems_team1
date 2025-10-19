#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class velocity_Publisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher_node')
        self.robot1_velocity_publisher = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.publish_velocity)

    def publish_velocity(self):
        vel_msg = Twist()
        vel_msg.linear.x = -1.0
        #vel_msg.angular.z =0.5
        self.robot1_velocity_publisher.publish(vel_msg)
        self.get_logger().info("Publishing velocity command to robot1")

def main(args=None):
    rclpy.init(args=args)
    robot1_velocity_node = velocity_Publisher()
    rclpy.spin(robot1_velocity_node)
    robot1_velocity_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()