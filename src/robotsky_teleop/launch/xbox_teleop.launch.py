"""Launch joy_node + xbox_cmd_vel for Xbox / similar gamepad teleop."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy_dev",
                default_value="/dev/input/js0",
                description="Joystick device path (e.g. /dev/input/js0 or js1)",
            ),
            DeclareLaunchArgument(
                "linear_scale",
                default_value="0.6",
                description="Scale for linear.x / linear.y from sticks",
            ),
            DeclareLaunchArgument(
                "angular_scale",
                default_value="0.8",
                description="Scale for angular.z from right stick",
            ),
            DeclareLaunchArgument(
                "deadzone",
                default_value="0.1",
                description="Analog stick deadzone [0, 1)",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Output Twist topic",
            ),
            DeclareLaunchArgument(
                "command_timeout",
                default_value="0.25",
                description="Publish zero cmd_vel if no Joy message arrives within this many seconds",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[{"dev": LaunchConfiguration("joy_dev")}],
            ),
            Node(
                package="robotsky_teleop",
                executable="xbox_cmd_vel",
                name="xbox_cmd_vel",
                output="screen",
                parameters=[
                    {
                        "joy_topic": "/joy",
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "linear_scale": LaunchConfiguration("linear_scale"),
                        "angular_scale": LaunchConfiguration("angular_scale"),
                        "deadzone": LaunchConfiguration("deadzone"),
                        "command_timeout": LaunchConfiguration("command_timeout"),
                    }
                ],
            ),
        ]
    )
