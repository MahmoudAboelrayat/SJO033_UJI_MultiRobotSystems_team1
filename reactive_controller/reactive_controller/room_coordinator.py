#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty

class RoomCoordinator(Node):
    def __init__(self):
        super().__init__('room_coordinator')

        self.declare_parameter('robot_ids', '')

        robot_ids_str = self.get_parameter('robot_ids').get_parameter_value().string_value
        self.robot_ids = robot_ids_str.split(',') if robot_ids_str else []

        if not self.robot_ids:
            self.get_logger().error("No robot_ids provided! Exiting...")
            raise ValueError("robot_ids parameter is required")

        self.get_logger().info(f"Active robots: {', '.join(self.robot_ids)}")

        # Define starting positions / rooms
        self.room_positions = [
            (0.35, -0.3),
            (-0.7, -0.5),
            (-0.4, 0.7),
            (0.75, 0.7)
        ]

        # Each robot keeps track of its current room index
        self.robot_indices = {rid: idx for idx, rid in enumerate(self.robot_ids)}

        # Publishers for each robot's clicked_point
        self.robot_publishers = {
            rid: self.create_publisher(PointStamped, f'/{rid}/clicked_point', 10)
            for rid in self.robot_ids
        }

        # Subscribe to trigger topic
        self.create_subscription(Empty, '/room_trigger', self.trigger_callback, 10)
        self.get_logger().info(
            "RoomCoordinator ready. Publish to /room_trigger to move all robots synchronously."
        )

    def trigger_callback(self, msg):
        """Move all robots to the next room when a trigger message is received."""
        self.send_goals()

    def send_goals(self):
        for rid, pub in self.robot_publishers.items():
            # Compute next room index for this robot
            current_idx = self.robot_indices[rid]
            next_idx = (current_idx + 1) % len(self.room_positions)
            self.robot_indices[rid] = next_idx

            x, y = self.room_positions[next_idx]
            msg = PointStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = x
            msg.point.y = y
            msg.point.z = 0.0

            pub.publish(msg)
            self.get_logger().info(f" → Sent {rid} to Room {next_idx + 1} at ({x}, {y})")


def main(args=None):
    rclpy.init(args=args)
    node = RoomCoordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
