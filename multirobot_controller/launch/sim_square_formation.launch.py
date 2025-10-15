from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    servers = []
    kp = LaunchConfiguration('kp', default='0.8')
    ki = LaunchConfiguration('ki', default='0.0')
    th_angle = LaunchConfiguration('th_angle', default='0.01')
    th_distance = LaunchConfiguration('th_distance', default='0.05')
    for i in range(1, 5):
        servers.append(
            Node(
                package="multirobot_controller",
                executable="square_controller_server",
                name=f"square_controller_server_{i}",
                namespace=f"robot{i}",
                parameters=[{"kp": kp,"ki": ki,"distance_threshold": th_distance, "angle_th": th_angle}],
                output="screen",
            )
        )

    client = Node(
        package="multirobot_controller",
        executable="square_controller_client",
        name="square_controller_client",
        output="screen",
    )

    return LaunchDescription(servers + [client])
