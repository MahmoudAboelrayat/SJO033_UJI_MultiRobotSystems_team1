#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty

# how to run : r1,r2,etc is the rooms for gazebo/real robots named robot1,robot2,robot3, etc...
#cmd command: ros2 run reactive_controller room_coordinator   --ros-args   -p robot_ids:="r1,r2,r4"   -p gazebo_names:="robot1,robot2,robot3" 



class RoomCoordinator(Node):
    def __init__(self):
        super().__init__('room_coordinator')

        # Logical robot IDs (your chosen names)
        self.declare_parameter('robot_ids', '')
        robot_ids_str = self.get_parameter('robot_ids').get_parameter_value().string_value
        self.robot_ids = robot_ids_str.split(',') if robot_ids_str else []

        # Actual Gazebo robot names
        self.declare_parameter('gazebo_names', '')
        gazebo_ids_str = self.get_parameter('gazebo_names').get_parameter_value().string_value
        self.gazebo_ids = gazebo_ids_str.split(',') if gazebo_ids_str else []

        if len(self.robot_ids) != len(self.gazebo_ids):
            raise ValueError("robot_ids and gazebo_names must have the SAME length")

        # Map: gazebo → logical AND logical → gazebo
        self.gazebo_to_logical = dict(zip(self.gazebo_ids, self.robot_ids))
        self.logical_to_gazebo = dict(zip(self.robot_ids, self.gazebo_ids))

        self.get_logger().info("Robot mapping:")
        for g, l in self.gazebo_to_logical.items():
            self.get_logger().info(f"  {g}  →  {l}")

        # Define 4 rooms coordinates
        self.room_positions = [
            (0.35, -0.3),
            (-0.7, -0.5),
            (-0.4, 0.7),
            (0.75, 0.7),
        ]

        # Extract number from IDs like r1, robot2, r4, etc.
        def extract_number(rid):
            digits = "".join(filter(str.isdigit, rid))
            if digits == "":
                raise ValueError(f"Robot ID '{rid}' has no number")
            return int(digits)

        # Each robot starts in the room that corresponds to its number
        #ex:  r1 → room 1, r2 → room 2, r4 → room 4, ...
        self.robot_indices = {}
        for rid in self.robot_ids:
            num = extract_number(rid)
            room_index = (num - 1) % len(self.room_positions)
            self.robot_indices[rid] = room_index
            self.get_logger().info(f"{rid} starts in Room {room_index + 1}")

        # Create publishers using GAZEBO namespaces
        self.robot_publishers = {
            rid: self.create_publisher(
                PointStamped,
                f'/{self.logical_to_gazebo[rid]}/clicked_point',
                10
            )
            for rid in self.robot_ids
        }

        # Subscribe to trigger
        self.create_subscription(Empty, '/room_trigger', self.trigger_callback, 10)
        self.get_logger().info("RoomCoordinator ready. Publish to /room_trigger to move robots.")

    def trigger_callback(self, msg):
        self.send_goals()

    def send_goals(self):
        for rid in self.robot_ids:

            # Move robot to next room
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

            pub = self.robot_publishers[rid]
            pub.publish(msg)

            gazebo_name = self.logical_to_gazebo[rid]

            self.get_logger().info(
                f" → Sent {rid} (actual: {gazebo_name}) to Room {next_idx+1} at ({x}, {y})"
            )


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
