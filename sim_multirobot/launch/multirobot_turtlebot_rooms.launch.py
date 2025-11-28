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

    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive', default_value=enable_drive, description='Enable robot drive node'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )

    turtlebot_dir = get_package_share_directory('turtlebot3_gazebo')

    urdf = os.path.join(
        turtlebot_dir, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )

    world = os.path.join(
        get_package_share_directory('sim_multirobot'),
        'worlds', 'multirobot_world.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )

        
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
 
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

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    last_action = None
    # Spawn turtlebot3 instances in gazebo
    for robot in robots:

        namespace = [ '/' + robot['name'] ]

        # Create state publisher node for that instance
        turtlebot_state_publisher = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                            'publish_frequency': 10.0}],
            remappings=remappings,
            arguments=[urdf],
        )

        # Create spawn call
        spawn_turtlebot3_burger = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', os.path.join(turtlebot_dir,'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-x', robot['x_pose'], '-y', robot['y_pose'],
                '-z', '0.01', '-Y', '0.0',
                '-unpause',
            ],
            output='screen',
        )
        if last_action is None:
            ld.add_action(turtlebot_state_publisher)
            ld.add_action(spawn_turtlebot3_burger)
            # ld.add_action(bringup_cmd)

        else:
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[spawn_turtlebot3_burger,
                            turtlebot_state_publisher],
                            # bringup_cmd],
                )
            )

            ld.add_action(spawn_turtlebot3_event)

        last_action = spawn_turtlebot3_burger
    return ld
