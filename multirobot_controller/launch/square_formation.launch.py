from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    servers = []
    for i in range(1, 5):
        servers.append(
            Node(
                package="multirobot_controller",
                executable="square_controller_server",
                name=f"square_controller_server_{i}",
                namespace=f"robot{i}",
                parameters=[{"kp": 0.8}],
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
