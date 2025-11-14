import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('reactive_controller'), 'resource')
   
    rviz_config_dir = os.path.join(config_dir, 'navigation.rviz')

    slam_params_file = os.path.join(config_dir, 'slam_localization.yaml')

    slam_toolbox_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'localization_launch.py'
    )

    return LaunchDescription([
        # --- 1. Launch Gazebo World ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('sim_multirobot_nav'),
                '/launch/turtlebot_world.launch.py'
            ])
        ),   

        # --- 2. Launch SLAM Toolbox in Localization Mode ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={
                'slam_params_file': slam_params_file
            }.items()
        )
        ,

        # --- 3. Launch custom A* planner ---
        Node(
            package='reactive_controller',             
            executable='astar_global_planner',
            name='astar_global_planner',
            output='screen',
        ),


        # --- 4. Launch custom Reactive Controller for Waypoint Following ---
        Node(
            package='reactive_controller',             
            executable='reactive_waypoint_controller',
            name='reactive_waypoint_controller',
            output='screen',
            parameters=[{'safe_distance': 0.08}],
            remappings=[
                ('/plan', '/plan'),
                ('/scan', '/scan'),
                ('/odom', '/odom'),
                ('/cmd_vel', '/cmd_vel')
            ]
        ),

        # --- 5. Launch RViz ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
    ])
