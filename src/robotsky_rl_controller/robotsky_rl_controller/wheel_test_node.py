"""Wheel-only hardware test node for RobotSky."""

from __future__ import annotations

import threading
from typing import Sequence

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from robotsky_interface.msg import MotorCmd, MotorCmds, MotorStates

NUM_JOINTS = 16
WHEEL_JOINT_INDICES = [3, 7, 11, 15]
WHEEL_NAMES = ["RF_Wheel", "LF_Wheel", "RB_Wheel", "LB_Wheel"]

DEFAULT_KP = np.array(
    [
        20.0,
        20.0,
        40.0,
        0.0,
        20.0,
        20.0,
        40.0,
        0.0,
        20.0,
        20.0,
        40.0,
        0.0,
        20.0,
        20.0,
        40.0,
        0.0,
    ],
    dtype=np.float64,
)
DEFAULT_KD = np.ones(NUM_JOINTS, dtype=np.float64)
DEFAULT_JOINT_POS = np.array(
    [
        0.0,
        -0.0,
        0.0,
        0.0,
        -0.0,
        -0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -0.0,
        0.0,
        -0.0,
        0.0,
        -0.0,
        0.0,
    ],
    dtype=np.float64,
)


class WheelTestNode(Node):
    """Hold the leg joints fixed and command wheel speed only."""

    def __init__(self) -> None:
        super().__init__("wheel_test")

        self.declare_parameter("motor_states_topic", "/motor_states")
        self.declare_parameter("motor_cmds_topic", "/motor_cmds")
        self.declare_parameter("publish_frequency", 50.0)
        self.declare_parameter("start_delay_sec", 1.0)
        self.declare_parameter("test_duration_sec", 0.0)
        self.declare_parameter("wheel_joint_indices", WHEEL_JOINT_INDICES)
        self.declare_parameter("wheel_speeds", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("kp", DEFAULT_KP.tolist())
        self.declare_parameter("kd", DEFAULT_KD.tolist())
        self.declare_parameter("default_joint_pos", DEFAULT_JOINT_POS.tolist())
        self.declare_parameter("hold_from_current_state", True)
        self.declare_parameter("report_interval_sec", 1.0)
        self.declare_parameter("active_speed_threshold", 0.5)
        self.declare_parameter("stuck_feedback_threshold", 0.1)

        self._motor_states_topic = str(self.get_parameter("motor_states_topic").value)
        self._motor_cmds_topic = str(self.get_parameter("motor_cmds_topic").value)
        self._publish_frequency = float(self.get_parameter("publish_frequency").value)
        self._start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self._test_duration_sec = float(self.get_parameter("test_duration_sec").value)
        self._hold_from_current_state = bool(self.get_parameter("hold_from_current_state").value)
        self._report_interval_sec = float(self.get_parameter("report_interval_sec").value)
        self._active_speed_threshold = float(self.get_parameter("active_speed_threshold").value)
        self._stuck_feedback_threshold = float(self.get_parameter("stuck_feedback_threshold").value)

        self._kp = self._read_vector_parameter("kp", DEFAULT_KP)
        self._kd = self._read_vector_parameter("kd", DEFAULT_KD)
        self._hold_joint_pos = self._read_vector_parameter("default_joint_pos", DEFAULT_JOINT_POS)

        wheel_joint_indices = [int(v) for v in self.get_parameter("wheel_joint_indices").value]
        if len(wheel_joint_indices) != len(WHEEL_NAMES):
            self.get_logger().warn(f"wheel_joint_indices length={len(wheel_joint_indices)} does not match expected {len(WHEEL_NAMES)}, using defaults")
            wheel_joint_indices = list(WHEEL_JOINT_INDICES)
        self._wheel_joint_indices = wheel_joint_indices
        self._wheel_set = set(self._wheel_joint_indices)
        self._leg_joint_indices = [i for i in range(NUM_JOINTS) if i not in self._wheel_set]

        wheel_speeds = [float(v) for v in self.get_parameter("wheel_speeds").value]
        if len(wheel_speeds) != len(WHEEL_NAMES):
            self.get_logger().warn(f"wheel_speeds length={len(wheel_speeds)} does not match expected {len(WHEEL_NAMES)}, using zeros")
            wheel_speeds = [0.0] * len(WHEEL_NAMES)
        self._wheel_speed_cmd = np.zeros(NUM_JOINTS, dtype=np.float64)
        for wheel_name, joint_idx, speed in zip(WHEEL_NAMES, self._wheel_joint_indices, wheel_speeds):
            self._wheel_speed_cmd[joint_idx] = speed
            self.get_logger().info(f"{wheel_name}: joint_index={joint_idx}, desired_speed={speed:.3f} rad/s")

        self._obs_lock = threading.Lock()
        self._latest_pos = np.zeros(NUM_JOINTS, dtype=np.float64)
        self._latest_vel = np.zeros(NUM_JOINTS, dtype=np.float64)
        self._has_motor_state = False
        self._hold_initialized = False
        self._test_complete_logged = False

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sub_motors = self.create_subscription(MotorStates, self._motor_states_topic, self._cb_motor_states, qos_profile)
        self._pub_cmds = self.create_publisher(MotorCmds, self._motor_cmds_topic, 10)

        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(1.0 / self._publish_frequency, self._publish_cmd)
        self._report_timer = self.create_timer(self._report_interval_sec, self._report_status)

        self.get_logger().info(
            f"Wheel test started: publish={self._publish_frequency:.1f} Hz, start_delay={self._start_delay_sec:.1f} s, "
            f"duration={self._test_duration_sec:.1f} s (0 means continuous)"
        )

    def _read_vector_parameter(self, name: str, default_values: np.ndarray) -> np.ndarray:
        values = np.asarray([float(v) for v in self.get_parameter(name).value], dtype=np.float64)
        if values.shape != default_values.shape:
            self.get_logger().warn(f"{name} length={len(values)} does not match expected {len(default_values)}, using defaults")
            return default_values.copy()
        return values

    def _cb_motor_states(self, msg: MotorStates) -> None:
        if len(msg.states) < NUM_JOINTS:
            return

        pos = np.asarray([msg.states[i].pos for i in range(NUM_JOINTS)], dtype=np.float64)
        vel = np.asarray([msg.states[i].vel for i in range(NUM_JOINTS)], dtype=np.float64)

        with self._obs_lock:
            self._latest_pos[:] = pos
            self._latest_vel[:] = vel
            self._has_motor_state = True
            if self._hold_from_current_state and not self._hold_initialized:
                self._hold_joint_pos[self._leg_joint_indices] = pos[self._leg_joint_indices]
                self._hold_initialized = True
                self.get_logger().info("Captured current leg joint positions as hold targets")

    def _elapsed_sec(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    def _active_wheel_command(self) -> np.ndarray:
        elapsed = self._elapsed_sec()
        if elapsed < self._start_delay_sec:
            return np.zeros(NUM_JOINTS, dtype=np.float64)
        if self._test_duration_sec > 0.0 and elapsed >= self._start_delay_sec + self._test_duration_sec:
            if not self._test_complete_logged:
                self.get_logger().info("Wheel test duration elapsed, commanding zero wheel speed")
                self._test_complete_logged = True
            return np.zeros(NUM_JOINTS, dtype=np.float64)
        return self._wheel_speed_cmd

    def _publish_cmd(self) -> None:
        with self._obs_lock:
            if not self._has_motor_state:
                return
            latest_pos = self._latest_pos.copy()

        msg = MotorCmds()
        msg.header.stamp = self.get_clock().now().to_msg()
        wheel_speed_cmd = self._active_wheel_command()

        for i in range(NUM_JOINTS):
            cmd = MotorCmd()
            if i in self._wheel_set:
                cmd.pos = 0.0
                cmd.vel = float(wheel_speed_cmd[i])
            else:
                hold_pos = float(self._hold_joint_pos[i] if self._hold_initialized or not self._hold_from_current_state else latest_pos[i])
                cmd.pos = 0.0  # hold_pos
                cmd.vel = 0.0
            cmd.tau = 0.0
            cmd.kp = float(self._kp[i])
            cmd.kd = float(self._kd[i])
            msg.cmds.append(cmd)

        self._pub_cmds.publish(msg)

    def _report_status(self) -> None:
        with self._obs_lock:
            if not self._has_motor_state:
                self.get_logger().info("Waiting for /motor_states before starting wheel test")
                return
            measured_vel = self._latest_vel.copy()

        commanded = self._active_wheel_command()
        for wheel_name, joint_idx in zip(WHEEL_NAMES, self._wheel_joint_indices):
            cmd = float(commanded[joint_idx])
            meas = float(measured_vel[joint_idx])
            status = "OK"
            if abs(cmd) >= self._active_speed_threshold:
                if abs(meas) < self._stuck_feedback_threshold:
                    status = "STUCK_OR_NO_FEEDBACK"
                elif cmd * meas < 0.0:
                    status = "INVERTED_DIRECTION"
            self.get_logger().info(f"{wheel_name}: cmd={cmd:+.3f} rad/s, meas={meas:+.3f} rad/s, status={status}")

    def stop(self) -> None:
        with self._obs_lock:
            if not self._has_motor_state:
                return
            latest_pos = self._latest_pos.copy()

        msg = MotorCmds()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(NUM_JOINTS):
            cmd = MotorCmd()
            if i in self._wheel_set:
                cmd.pos = 0.0
                cmd.vel = 0.0
            else:
                hold_pos = float(self._hold_joint_pos[i] if self._hold_initialized or not self._hold_from_current_state else latest_pos[i])
                cmd.pos = hold_pos
                cmd.vel = 0.0
            cmd.tau = 0.0
            cmd.kp = float(self._kp[i])
            cmd.kd = float(self._kd[i])
            msg.cmds.append(cmd)
        self._pub_cmds.publish(msg)


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WheelTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
