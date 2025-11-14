import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xml.etree.ElementTree as ET
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.actions import GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_folder = f'turtlebot3_{TURTLEBOT3_MODEL}'
    turtlebot_pkg = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(turtlebot_pkg, 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    urdf_path = os.path.join(
        turtlebot_pkg,
        'models',
        model_folder,
        'model.sdf'
    )

    world = os.path.join(
        turtlebot_pkg,
        'worlds',
        'empty_world.world'
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

    # Define square formation positions
    x_center = LaunchConfiguration('x_center', default='0.0')
    y_center = LaunchConfiguration('y_center', default='0.0')
    side_length = LaunchConfiguration('length', default='1.0')
    yaw = LaunchConfiguration('yaw', default='0.0')
    

    offset = PythonExpression(['(', side_length, ' / 2.0)'])
    poses = [
        (PythonExpression(['(', x_center, ' - ', offset, ')']), PythonExpression(['(', y_center, ' - ', offset, ')'])),  # Robot 1
        (PythonExpression(['(', x_center, ' + ', offset, ')']), PythonExpression(['(', y_center, ' - ', offset, ')'])),  # Robot 2
        (PythonExpression(['(', x_center, ' + ', offset, ')']), PythonExpression(['(', y_center, ' + ', offset, ')'])),  # Robot 3
        (PythonExpression(['(', x_center, ' - ', offset, ')']), PythonExpression(['(', y_center, ' + ', offset, ')']))   # Robot 4
    ]   

    # Loop to spawn each robot
    for i, (x, y) in enumerate(poses, start=1):
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        robot_name = f'{TURTLEBOT3_MODEL}_{i}'
        namespace = f'robot{i}'
        yaw_angle = PythonExpression(['(', yaw, ' + (1.57 * ', str(i - 2), '))'])
        for odom_frame_tag in root.iter('odometry_frame'):
            odom_frame_tag.text = f'{namespace}/odom'
        for base_frame_tag in root.iter('robot_base_frame'):
            base_frame_tag.text = f'{namespace}/base_footprint'
        for scan_frame_tag in root.iter('frame_name'):
            scan_frame_tag.text = f'{namespace}/base_scan'
        urdf_modified = ET.tostring(tree.getroot(), encoding='unicode')
        urdf_modified = '<?xml version="1.0" ?>\n'+urdf_modified
        with open(f'{save_path}{i+1}.sdf', 'w') as file:
            file.write(urdf_modified)


        

        robot_state_publisher = (
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'frame_prefix': namespace
                    }.items()
            )
        )
        spawn_robot  = (Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-file', f'{save_path}{i+1}.sdf',
            '-x', x,
            '-y', y,
            '-z', '0.01',
            '-Y', '0.0',
            '-robot_namespace', namespace
        ],
        output='screen',
        ))
        
        ld.add_action(spawn_robot)
        ld.add_action(robot_state_publisher)


    #add the TFs
    map_frame =  Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='world_map',
    arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )
    ld.add_action(map_frame)

    for i, (x, y) in enumerate(poses, start=1):
        robot_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'map_to_robot{i}_odom',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'map', f'robot{i}/odom'
        ]
        )
        ld.add_action(robot_tf)


    # Rviz node
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare('sim_multirobot'),
            'rviz',
            'trutlebot_sqaure.rviz'
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    ld.add_action(rviz_node)


     # Register shutdown event to clean up temporary files

    ld.add_action(RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event,
            context: [os.remove(f'{save_path}{count+1}.sdf') for count in range(4)]
        )
    ))

    return ld
