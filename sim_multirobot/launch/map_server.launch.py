import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import launch.logging

def generate_launch_description():
    ld = LaunchDescription()

    robots = [
        {'name': 'tb1', 'x_pose': '0.35', 'y_pose': '-0.3', 'z_pose': 0.01},
        {'name': 'tb2', 'x_pose': '-0.7', 'y_pose': '-0.5', 'z_pose': 0.01},
        {'name': 'tb3', 'x_pose': '-0.4', 'y_pose': '0.7', 'z_pose': 0.01},
        # {'name': 'tb4', 'x_pose': '0.75', 'y_pose': '0.7', 'z_pose': 0.01},
        ]

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('multi_nav2'), 'map', 'map_cart_good.yaml'),
                     },],
        remappings=remappings)

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])

    
    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)

    
    return ld
