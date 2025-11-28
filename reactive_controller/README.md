
# Multi-Robot Navigation with ROS 2

This repository contains code to run a multi-robot navigation simulation in Gazebo with ROS 2, including a **room coordinator** to move robots synchronously between rooms.

---

## Launch the Simulation

Start the multi-robot navigation simulation with SLAM and localization:

```bash
ros2 launch reactive_controller multi_robot_nav_with_slam_localization.launch.py
```

This launch will:

- Spawn 3 robots (`robot1`, `robot2`, `robot3`) in Gazebo
- Start SLAM Toolbox localization for each robot
- Launch the `astar_global_planner_multirobot` and `reactive_waypoint_controller_multirobot` nodes for each robot
- Publish initial poses for each robot after a short delay
- Open RViz with the `multirobot_navigation.rviz` configuration

---

## Initial Robot Setup

| Logical Robot | Gazebo Namespace | Initial Room | Initial Position (x, y, yaw) |
|---------------|-----------------|--------------|------------------------------|
| r1            | robot1          | 1            | (0.35, -0.3, 0.0)           |
| r2            | robot2          | 2            | (-0.7, -0.5, 0.0)           |
| r4            | robot3          | 4            | (0.75, 0.7, 0.0)            |

> Note: Room 3 is skipped. `r4` corresponds to `robot3` in Gazebo.


---

## Start the Room Coordinator

The Room Coordinator moves robots synchronously between rooms when triggered:

```bash
ros2 run reactive_controller room_coordinator \
  --ros-args \
  -p robot_ids:="r1,r2,r4" \
  -p gazebo_names:="robot1,robot2,robot3"
```

- `robot_ids` → robot rooms names used internally by the coordinator  
- `gazebo_names` → actual robot namespaces created by Gazebo  

**Initial room assignment:**

| Logical ID | Gazebo Name | Room |
|------------|-------------|------|
| r1         | robot1      | 1    |
| r2         | robot2      | 2    |
| r4         | robot3      | 4    |

> Note: The coordinator assigns robots to rooms based on their **logical ID number**, so `r4` goes to Room 1 even though Gazebo names it `robot3`.

---

## Move Robots to the Next Room

Trigger all robots to move to the next room:

```bash
ros2 topic pub /room_trigger std_msgs/msg/Empty "{}" -1
```

- Each trigger moves **all robots** to the next room cyclically  
- Resent to cycle robots through next rooms

---
