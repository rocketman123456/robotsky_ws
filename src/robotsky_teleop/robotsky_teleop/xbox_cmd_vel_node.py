"""Map Xbox /joy to geometry_msgs/Twist (cmd_vel).

- Left stick: linear.x (forward/back), linear.y (strafe left/right)
- Right stick: linear.x from vertical axis, angular.z (yaw rate) from horizontal axis

Default axis indices follow the team's current mapping (left stick 0/1, right stick 2/3).
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy


def _deadzone(v: float, dz: float) -> float:
    if dz <= 0.0:
        return v
    if abs(v) < dz:
        return 0.0
    # Rescale so value is continuous outside deadzone
    sign = 1.0 if v > 0.0 else -1.0
    return sign * (abs(v) - dz) / max(1.0 - dz, 1e-6)


def _is_zero_twist(msg: Twist, eps: float = 1e-6) -> bool:
    return (
        abs(msg.linear.x) <= eps
        and abs(msg.linear.y) <= eps
        and abs(msg.linear.z) <= eps
        and abs(msg.angular.x) <= eps
        and abs(msg.angular.y) <= eps
        and abs(msg.angular.z) <= eps
    )


class XboxCmdVelNode(Node):
    def __init__(self) -> None:
        super().__init__("xbox_cmd_vel")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("angular_scale", 1.0)
        self.declare_parameter("deadzone", 0.12)
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("timeout_check_period", 0.05)

        # Controller mappings vary; these defaults match the current workspace setup.
        self.declare_parameter("axis_left_x", 0)
        self.declare_parameter("axis_left_y", 1)
        self.declare_parameter("axis_right_x", 3)  # 2
        self.declare_parameter("axis_right_y", 4)  # 3

        self.declare_parameter("invert_left_x", False)
        self.declare_parameter("invert_left_y", False)
        self.declare_parameter("invert_right_x", False)
        self.declare_parameter("invert_right_y", False)

        # If true, linear.x uses only left Y + right Y; linear.y only left X
        self.declare_parameter("right_stick_affects_linear_x", True)

        self._joy_topic = self.get_parameter("joy_topic").value
        self._cmd_topic = self.get_parameter("cmd_vel_topic").value
        self._linear_scale = float(self.get_parameter("linear_scale").value)
        self._angular_scale = float(self.get_parameter("angular_scale").value)
        self._deadzone = float(self.get_parameter("deadzone").value)
        self._command_timeout = float(self.get_parameter("command_timeout").value)
        self._timeout_check_period = float(self.get_parameter("timeout_check_period").value)

        self._ax_lx = int(self.get_parameter("axis_left_x").value)
        self._ax_ly = int(self.get_parameter("axis_left_y").value)
        self._ax_rx = int(self.get_parameter("axis_right_x").value)
        self._ax_ry = int(self.get_parameter("axis_right_y").value)

        self._inv_lx = -1.0 if self.get_parameter("invert_left_x").value else 1.0
        self._inv_ly = -1.0 if self.get_parameter("invert_left_y").value else 1.0
        self._inv_rx = -1.0 if self.get_parameter("invert_right_x").value else 1.0
        self._inv_ry = -1.0 if self.get_parameter("invert_right_y").value else 1.0

        self._right_linear_x = bool(self.get_parameter("right_stick_affects_linear_x").value)
        self._last_joy_time: float | None = None
        self._last_command_was_zero = True

        # Match rl_controller_node /cmd_vel subscription (BEST_EFFORT).
        _cmd_vel_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        _joy_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._pub = self.create_publisher(Twist, self._cmd_topic, _cmd_vel_qos)
        self._joy_sub = self.create_subscription(Joy, self._joy_topic, self._on_joy, _joy_qos)
        self._timeout_timer = None
        if self._command_timeout > 0.0 and self._timeout_check_period > 0.0:
            self._timeout_timer = self.create_timer(self._timeout_check_period, self._on_timeout)

        self.get_logger().info(
            f"Xbox cmd_vel: {self._joy_topic} -> {self._cmd_topic} "
            f"(linear_scale={self._linear_scale}, angular_scale={self._angular_scale}, timeout={self._command_timeout}s)"
        )

    def _axis(self, msg: Joy, index: int) -> float:
        if index < 0 or index >= len(msg.axes):
            return 0.0
        v = float(msg.axes[index])
        if not math.isfinite(v):
            return 0.0
        return max(-1.0, min(1.0, v))

    def _on_joy(self, msg: Joy) -> None:
        self._last_joy_time = time.monotonic()
        lx = _deadzone(self._axis(msg, self._ax_lx) * self._inv_lx, self._deadzone)
        ly = _deadzone(self._axis(msg, self._ax_ly) * self._inv_ly, self._deadzone)
        rx = _deadzone(self._axis(msg, self._ax_rx) * self._inv_rx, self._deadzone)
        ry = _deadzone(self._axis(msg, self._ax_ry) * self._inv_ry, self._deadzone)

        # Left: xy velocity (ROS: x forward, y left)
        out = Twist()
        out.linear.x = self._linear_scale * ly
        out.linear.y = self._linear_scale * lx * 0.5
        out.linear.z = 0.0

        # Right: yaw rate + optional extra forward/back on linear.x
        out.angular.z = self._angular_scale * rx
        out.angular.x = 0.0
        out.angular.y = 0.0

        if self._right_linear_x:
            out.linear.x += self._linear_scale * ry

        self._publish_twist(out)

    def _publish_twist(self, msg: Twist) -> None:
        self._pub.publish(msg)
        self._last_command_was_zero = _is_zero_twist(msg)

    def _on_timeout(self) -> None:
        if self._last_joy_time is None or self._last_command_was_zero:
            return
        if time.monotonic() - self._last_joy_time < self._command_timeout:
            return

        zero = Twist()
        self._publish_twist(zero)
        self.get_logger().warn("Joystick timeout detected, publishing zero cmd_vel", throttle_duration_sec=1.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XboxCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
