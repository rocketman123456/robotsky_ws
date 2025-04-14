from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    rviz_path = FindPackageShare("srobot_hardware")
    default_rviz_config_path = PathJoinSubstitution([rviz_path, "rviz", "imu.rviz"])

    return LaunchDescription(
        [
            # Optional RViz config
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", default_rviz_config_path],
            ),
        ]
    )
