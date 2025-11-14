#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import heapq

class AStarGlobalPlanner(Node):
    def __init__(self):
        super().__init__('astar_global_planner')

        self.map_data = None
        self.map_received = False
        self.odom_pose = None
        self.goal_pose = None
        self.laser_data = None
        self.user_goal_active = True


        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(PointStamped, 'clicked_point', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)

        # Publisher
        self.plan_pub = self.create_publisher(Path, 'plan', 10)

        # Timer for periodic replanning
        self.timer = self.create_timer(0.5, self.publish_path)

    # ----------------- Callbacks -----------------
    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.grid = np.array(msg.data).reshape((self.height, self.width))
        self.inflate_obstacles(inflation_radius=0.115)  # margin
        self.map_received = True
        self.get_logger().info("Map received and obstacles inflated.")

    def odom_callback(self, msg: Odometry):
        self.odom_pose = msg.pose.pose

    def goal_callback(self, msg: PointStamped):
        from geometry_msgs.msg import Pose
        self.goal_pose = PoseStamped().pose
        self.goal_pose.position.x = msg.point.x
        self.goal_pose.position.y = msg.point.y
        self.goal_pose.position.z = 0.0
        self.goal_pose.orientation.w = 1.0
        self.user_goal_active = True
        self.get_logger().info(f"New goal received at ({self.goal_pose.position.x}, {self.goal_pose.position.y})")

    def laser_callback(self, msg: LaserScan):
        self.laser_data = msg

    # ----------------- Utility functions -----------------
    def world_to_map(self, x, y):
        mx = int((x - self.origin[0]) / self.resolution)
        my = int((y - self.origin[1]) / self.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        x = mx * self.resolution + self.origin[0] + self.resolution / 2
        y = my * self.resolution + self.origin[1] + self.resolution / 2
        return x, y

    def get_yaw(self, pose):
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    # ----------------- Obstacle Inflation -----------------
    def inflate_obstacles(self, inflation_radius=0.2):
        """Mark cells around obstacles as occupied to keep a safety margin."""
        if self.grid is None:
            return
        inflation_cells = int(inflation_radius / self.resolution)
        inflated_grid = self.grid.copy()
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y, x] > 50:  # occupied
                    y_min = max(0, y - inflation_cells)
                    y_max = min(self.height, y + inflation_cells + 1)
                    x_min = max(0, x - inflation_cells)
                    x_max = min(self.width, x + inflation_cells + 1)
                    inflated_grid[y_min:y_max, x_min:x_max] = 100
        self.grid = inflated_grid

    def update_dynamic_obstacles(self):
        """Overlay LIDAR points as temporary obstacles with inflation for safety."""
        if self.laser_data is None or self.odom_pose is None or not self.map_received:
            return self.grid
        inflation_radius=0.12
        dynamic_grid = self.grid.copy()
        angle = self.laser_data.angle_min
        inflation_cells = max(1, int(inflation_radius / self.resolution))  # inflate around each LIDAR point

        for r in self.laser_data.ranges:
            if 0.05 < r < 1.0:  # ignore too close or too far points (adjust 1.0 for narrow map)
                x_obs = self.odom_pose.position.x + r * math.cos(angle + self.get_yaw(self.odom_pose))
                y_obs = self.odom_pose.position.y + r * math.sin(angle + self.get_yaw(self.odom_pose))
                mx, my = self.world_to_map(x_obs, y_obs)

                # Inflate the obstacle around LIDAR hit
                for dx in range(-inflation_cells, inflation_cells+1):
                    for dy in range(-inflation_cells, inflation_cells+1):
                        nx, ny = mx + dx, my + dy
                        if 0 <= nx < self.width and 0 <= ny < self.height:
                            dynamic_grid[ny, nx] = 100  # mark as obstacle

            angle += self.laser_data.angle_increment

        return dynamic_grid


    # ----------------- A* -----------------
    def neighbors(self, node, grid):
        x, y = node
        result = []
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,-1),(-1,1),(1,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if grid[ny, nx] == 0:
                    result.append((nx, ny))
        return result

    def heuristic(self, a, b):
        return math.hypot(b[0]-a[0], b[1]-a[1])

    def astar(self, start, goal, grid):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for neighbor in self.neighbors(current, grid):
                tentative_g = g_score[current] + self.heuristic(current, neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return None

    # ----------------- Path Publisher -----------------
    def publish_path(self):
        if not self.map_received or self.odom_pose is None or self.goal_pose is None:
            return

        start_m = self.world_to_map(self.odom_pose.position.x, self.odom_pose.position.y)
        goal_m = self.world_to_map(self.goal_pose.position.x, self.goal_pose.position.y)

        # Use a dynamic grid with LIDAR obstacles
        dynamic_grid = self.update_dynamic_obstacles()

        path_cells = self.astar(start_m, goal_m, dynamic_grid)
        if path_cells is None:
            self.get_logger().warn("No path found!")
            return #should be replaced and add a new method here if i didn't find a path...inside rooms cases
            
        # Compute path cost
        cost = sum(self.heuristic(path_cells[i-1], path_cells[i]) for i in range(1, len(path_cells)))

        # Publish path if:
        # 1. It is a user goal (always publish), or
        # 2. Automatic replanning produces shorter path
        if getattr(self, 'user_goal_active', False) or not hasattr(self, 'last_path_cost') or cost < getattr(self, 'last_path_cost', float('inf')):
            self.last_path_cost = cost
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for mx, my in path_cells:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                x, y = self.map_to_world(mx, my)
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)

            self.plan_pub.publish(path_msg)
            self.get_logger().info(f"Published path with {len(path_msg.poses)} waypoints, cost={cost:.2f}")

            # reset user goal flag after publishing once
            self.user_goal_active = False
        else:
            self.get_logger().info(f"New path ignored (cost {cost:.2f} > last cost {self.last_path_cost:.2f})")



def main(args=None):
    rclpy.init(args=args)
    node = AStarGlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
