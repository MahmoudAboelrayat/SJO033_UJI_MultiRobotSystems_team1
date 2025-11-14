# launch/multirobot_localization_and_control.launch.py
import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('reactive_controller')
    resource_dir = os.path.join(pkg_share, 'resource')
    rviz_config_dir = os.path.join(resource_dir, 'multirobot_navigation.rviz')
    slam_params_file = os.path.join(resource_dir, 'slam_localization_multirobot.yaml')

    slam_toolbox_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'localization_launch.py'
    )

    sim_robot_ids = ['robot1', 'robot2', 'robot3', 'robot4']
    robot_ids = sim_robot_ids

    initial_poses = [
        (0.35, -0.3, 0.0),
        (-0.7, -0.5, 0.0),
        (-0.4, 0.7, 0.0),
        (0.75, 0.7, 0.0),
    ]

    nodes = []

    # Spawn world with multiple robots
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('sim_multirobot_nav'),
                '/launch/multirobot_turtlebot_rooms.launch.py'
            ])
        )
    )

    # SLAM + controllers + planner per robot
    for idx, robot in enumerate(robot_ids):
        # Include SLAM Toolbox launch per namespace
        slam_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'namespace': robot,
                'use_sim_time': 'True',
                'map_publisher': 'True' if idx == 0 else 'False'
            }.items()
        )
        nodes.append(slam_include)

        # Planner
        planner_node = Node(
            package='reactive_controller',
            executable='astar_global_planner_multirobot',
            name='astar_global_planner_multirobot',
            namespace=robot,
            output='screen',
            parameters=[{
                'initial_x': initial_poses[idx][0],
                'initial_y': initial_poses[idx][1],
                'initial_yaw': initial_poses[idx][2],
            }]
        )
        nodes.append(planner_node)

        # Controller
        controller_node = Node(
            package='reactive_controller',
            executable='reactive_waypoint_controller_multirobot',
            name='reactive_waypoint_controller_multirobot',
            namespace=robot,
            output='screen',
            parameters=[{'safe_distance': 0.08}],
        )
        nodes.append(controller_node)

        # Initial pose publisher (delayed to let SLAM start)
        init_pose_node = TimerAction(
            period=2.0,  # wait 2 sec before publishing
            actions=[Node(
                package='reactive_controller',
                executable='set_initial_pose',
                name='set_initial_pose',
                namespace=robot,
                output='screen',
                parameters=[{
                    'initial_x': initial_poses[idx][0],
                    'initial_y': initial_poses[idx][1],
                    'initial_yaw': initial_poses[idx][2],
                }]
            )]
        )
        nodes.append(init_pose_node)

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
    )
    nodes.append(rviz_node)

    return nodes

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
