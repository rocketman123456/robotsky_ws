import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription(
        [
            Node(package="robotsky_imu", executable="imu_node", name="imu_node"),
        ]
    )
