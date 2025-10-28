from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution


def generate_launch_description():
    servers = []
    kp = LaunchConfiguration('kp', default='0.8')
    ki = LaunchConfiguration('ki', default='0.0')
    th_angle = LaunchConfiguration('th_angle', default='0.01')
    th_distance = LaunchConfiguration('th_distance', default='0.05')
    side_length = LaunchConfiguration('length', default='1.0')
    init_angle = LaunchConfiguration('init_angle', default='0.0')
    max_speed = LaunchConfiguration('max_speed', default='0.2')
    max_yaw_rate = LaunchConfiguration('max_yaw_rate', default='0.2')
    # Separate LaunchConfigurations for each robot namespace
    ns1 = LaunchConfiguration('ns1', default='tb_108')
    ns2 = LaunchConfiguration('ns2', default='tb_109')
    ns3 = LaunchConfiguration('ns3', default='tb_110')
    ns4 = LaunchConfiguration('ns4', default='tb_111')
    client_namespaces = 'ns1,ns2,ns3,ns4'

    namespace_list = [ns1, ns2, ns3, ns4]

    controller = Node(
        package="simple_multirobot_square_formation",
        executable="simple_square_node",
        name="simple_square_node",
        parameters=[{"robot_namespaces": [ns1, TextSubstitution(text=','), 
                                     ns2, TextSubstitution(text=','), 
                                     ns3, TextSubstitution(text=','), 
                                     ns4],"kp": kp,""
                                     "ki": ki,
                                     "distance_threshold": th_distance,
                                       "angle_th": th_angle,
                                       "max_linear_speed" : max_speed,
                                       "max_angular_speed": max_yaw_rate,
                                       "simulation": False}],
        output="screen",
    )


    #Odom transforms for robot frames
    transform = Node(
        package="simple_multirobot_square_formation",
        executable="transform_odom",
        name="transform_odom",
        parameters=[{"robot_namespaces": [ns1, TextSubstitution(text=','), 
                                     ns2, TextSubstitution(text=','), 
                                     ns3, TextSubstitution(text=','), 
                                     ns4]}],
        output="screen",
    )


    offset = PythonExpression(['(', side_length, ' / 2.0)'])
    poses = [
        (PythonExpression(["-(", offset, ")"]), PythonExpression(["-(", offset, ")"]), "0.0"),   # Robot 1
        (offset, PythonExpression(["-(", offset, ")"]), "0.0"),                                   # Robot 2
        (offset, offset, "0.0"),                                                                # Robot 3
        (PythonExpression(["-(", offset, ")"]), offset, "0.0"),                                # Robot 4
    ]


    # TF nodes
    tfs = []
    for i, (x, y, yaw) in enumerate(poses, start=1):
        robot_ns = PythonExpression(["'", namespace_list[i-1], "/odom'"])
        tfs.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"map_to_odom_{i}",
                arguments=[x, y, '0.0', yaw, '0.0', '0.0', 'map',robot_ns],
                output="screen",
            )
        )

    return LaunchDescription([controller] + tfs+ [transform])
