import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    multi_nav2_share = get_package_share_directory('multi_nav2')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    nav_launch_dir = os.path.join(nav2_bringup_share, 'launch')
    default_map = os.path.join(multi_nav2_share, 'map', 'my_map4.yaml')
    default_params = os.path.join(multi_nav2_share, 'param', 'nav2_params.yaml')

    rviz_config_file =os.path.join(
            nav2_bringup_share, 'rviz', 'nav2_namespaced_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1', description='Namespace for the robot'),
        DeclareLaunchArgument('use_namespace', default_value='True', description='use the namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation time'),
        DeclareLaunchArgument('map', default_value=default_map, description='Full path to map yaml file'),
        DeclareLaunchArgument('params_file', default_value=default_params, description='Full path to Nav2 parameter file'),
        DeclareLaunchArgument('autostart', default_value='True', description='Automatically start Nav2 nodes'),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': use_namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'map': map_file,
                'params_file': params_file
            }.items(),
        ),
         IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'namespace': namespace,
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file, 'log_level': 'warn'}.items(),
                                    ),
    ])
