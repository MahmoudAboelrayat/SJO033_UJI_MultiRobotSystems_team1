import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown
import xml.etree.ElementTree as ET
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_folder = f'turtlebot3_{TURTLEBOT3_MODEL}'
    turtlebot_pkg = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(turtlebot_pkg, 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = FindPackageShare('sim_multirobot').find('sim_multirobot')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_path = os.path.join(turtlebot_pkg, 'models', model_folder, 'model.sdf')

    world = os.path.join(pkg_share, 'worlds', 'multirobot_world')

    save_path = os.path.join(turtlebot_pkg, 'models', model_folder, 'tmp')

    # Launch Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Single robot pose
    x, y = '0.35', '-0.3'
    robot_name = f'{TURTLEBOT3_MODEL}_1'
    ns = 'robot1'

    turtlebot3_description_pkg = get_package_share_directory('turtlebot3_description')
    urdf_path = os.path.join(turtlebot3_description_pkg, 'urdf', f'turtlebot3_{TURTLEBOT3_MODEL}.urdf')
    with open(urdf_path, 'r') as file:
        urdf_content = file.read()

    # Modify URDF/SDF frames for this robot
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for odom_frame_tag in root.iter('odometry_frame'):
        odom_frame_tag.text = f'{ns}/odom'
    for base_frame_tag in root.iter('robot_base_frame'):
        base_frame_tag.text = f'{ns}/base_footprint'
    for scan_frame_tag in root.iter('frame_name'):
        scan_frame_tag.text = f'{ns}/base_scan'
    urdf_modified = ET.tostring(root, encoding='unicode')
    urdf_modified = '<?xml version="1.0" ?>\n' + urdf_modified

    sdf_file = os.path.join(save_path, 'robot1.sdf')
    with open(sdf_file, 'w') as file:
        file.write(urdf_modified)

    # Robot State Publisher
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     namespace=ns,
    #     parameters=[{
    #         'use_sim_time': True,
    #         'robot_description': urdf_content,
    #         'frame_prefix': f'{ns}/'
    #     }],
    #     remappings=[
    #         ('/tf', f'{ns}/tf'),
    #         ('/tf_static', f'{ns}/tf_static')
    #     ],
    #     output='screen'
    # )

    from launch.substitutions import Command

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_burger.urdf.xacro'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_path, ' namespace:=robot1']),
            'frame_prefix': 'robot1/'
        }],
        remappings=[('/tf', 'robot1/tf'), ('/tf_static', 'robot1/tf_static')],
        output='screen'
    )


    ld.add_action(robot_state_publisher)

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-file', sdf_file,
            '-x', x,
            '-y', y,
            '-z', '0.01',
            '-Y', '0.0',
            '-robot_namespace', ns
        ],
        output='screen',
    )
    ld.add_action(spawn_robot)

    # Static TFs
    # map_frame = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='world_map',
    #     arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    # )
    # ld.add_action(map_frame)

    # robot_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map_to_robot1_odom',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', f'{namespace}/odom']
    # )
    # ld.add_action(robot_tf)

    # Rviz node
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare('sim_multirobot'), 'rviz', 'trutlebot_sqaure.rviz']
    # )
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_file]
    # )
    # ld.add_action(rviz_node)

    # Cleanup temporary files on shutdown
    ld.add_action(RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event, context: os.remove(sdf_file)
        )
    ))

    return ld
