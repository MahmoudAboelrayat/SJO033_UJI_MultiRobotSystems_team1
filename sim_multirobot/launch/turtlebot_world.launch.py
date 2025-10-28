from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
import math
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_folder = f'turtlebot3_{TURTLEBOT3_MODEL}'
    turtlebot_pkg = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(turtlebot_pkg, 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_share = FindPackageShare('sim_multirobot').find('sim_multirobot')

    urdf_path = os.path.join(
        turtlebot_pkg,
        'models',
        model_folder,
        'model.sdf'
    )

    world = os.path.join(
        pkg_share,
        'worlds',
        'multirobot_world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    save_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'tmp'
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Launch robot_state_publisher (using TurtleBot3’s file)
    
    robot_state_publisher = (
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                }.items()
        )
    )

    # Spawn the TurtleBot
    spawn_turtlebot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'turtlebot3_burger',
            '-database', 'turtlebot3_burger',
            '-x', '0.0', '-y', '0.2', '-Y', str(math.radians(0))
        ],
        output='screen'
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_turtlebot)
    return ld
  
