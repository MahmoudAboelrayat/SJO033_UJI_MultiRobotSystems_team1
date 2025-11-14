#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time

class SetInitialPose(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.initial_x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('initial_yaw').get_parameter_value().double_value

         # Publish to namespaced initialpose
        ns = self.get_namespace().rstrip('/')  # remove trailing slash
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            f'{ns}/initialpose',
            10
        )

        # give a small delay before publishing to let localization node start
        self.create_timer(1.0, self.publish_once)

        self.published = False
        self.get_logger().info(f"Will publish initialpose: ({self.initial_x},{self.initial_y},{self.initial_yaw}) in namespace {self.get_namespace()}")

    def publish_once(self):
        if self.published:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'    # initial pose is in 'map' frame
        msg.pose.pose.position.x = float(self.initial_x)
        msg.pose.pose.position.y = float(self.initial_y)
        # quaternion from yaw
        yaw = float(self.initial_yaw)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # small covariance (x,y,yaw)
        cov = [0.0]*36
        cov[0] = 0.25   # x var
        cov[7] = 0.25   # y var
        cov[35] = 0.06853891945200942  # yaw var ~ (0.261799 rad)^2
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.get_logger().info("Published initialpose")
        self.published = True

def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
