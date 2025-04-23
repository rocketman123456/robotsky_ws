import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory("robotsky_description"), "urdf", "robotsky_wq.urdf")

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    return launch.LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc}],
                arguments=[urdf_path],
            ),
            # Node(package="robotsky_motor", executable="robot_node", name="robot_node"),
            # Node(package="robotsky_imu", executable="imu_node", name="imu_node"),
        ]
    )
