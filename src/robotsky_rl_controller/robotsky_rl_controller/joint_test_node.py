"""Joint sine-wave hardware test node for RobotSky."""

from __future__ import annotations

import math
import threading
from collections import deque
from typing import Sequence

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from robotsky_interface.msg import MotorCmd, MotorCmds, MotorStates

NUM_JOINTS = 16
WHEEL_JOINT_INDICES = [3, 7, 11, 15]
WHEEL_SET = set(WHEEL_JOINT_INDICES)
JOINT_NAMES = [
    "RF_Roll",
    "RF_Hip",
    "RF_Knee",
    "RF_Wheel",
    "LF_Roll",
    "LF_Hip",
    "LF_Knee",
    "LF_Wheel",
    "RB_Roll",
    "RB_Hip",
    "RB_Knee",
    "RB_Wheel",
    "LB_Roll",
    "LB_Hip",
    "LB_Knee",
    "LB_Wheel",
]

DEFAULT_KP = np.array(
    [
        20.0, 20.0, 40.0, 0.0,
        20.0, 20.0, 40.0, 0.0,
        20.0, 20.0, 40.0, 0.0,
        20.0, 20.0, 40.0, 0.0,
    ],
    dtype=np.float64,
)
DEFAULT_KD = np.ones(NUM_JOINTS, dtype=np.float64)
DEFAULT_JOINT_POS = np.array(
    [
        0.0, -0.0, 0.0, 0.0,
        -0.0, -0.0, 0.0, 0.0,
        0.0, 0.0, -0.0, 0.0,
        -0.0, 0.0, -0.0, 0.0,
    ],
    dtype=np.float64,
)


class JointTestNode(Node):
    """Command selected non-wheel joints with a sine-wave position target."""

    def __init__(self) -> None:
        super().__init__("joint_test")

        self.declare_parameter("motor_states_topic", "/motor_states")
        self.declare_parameter("motor_cmds_topic", "/motor_cmds")
        self.declare_parameter("publish_frequency", 50.0)
        self.declare_parameter("start_delay_sec", 1.0)
        self.declare_parameter("test_duration_sec", 0.0)
        self.declare_parameter("joint_indices", [0])
        self.declare_parameter("amplitudes", [0.15])
        self.declare_parameter("frequencies_hz", [0.25])
        self.declare_parameter("phases_rad", [0.0])
        self.declare_parameter("center_positions", [])
        self.declare_parameter("kp", DEFAULT_KP.tolist())
        self.declare_parameter("kd", DEFAULT_KD.tolist())
        self.declare_parameter("default_joint_pos", DEFAULT_JOINT_POS.tolist())
        self.declare_parameter("hold_from_current_state", True)
        self.declare_parameter("report_interval_sec", 1.0)
        self.declare_parameter("status_window_sec", 4.0)
        self.declare_parameter("active_amplitude_threshold", 0.05)
        self.declare_parameter("min_motion_span", 0.04)
        self.declare_parameter("low_amplitude_ratio_threshold", 0.35)
        self.declare_parameter("correlation_warn_threshold", -0.3)
        self.declare_parameter("rmse_warn_threshold", 0.25)

        self._motor_states_topic = str(self.get_parameter("motor_states_topic").value)
        self._motor_cmds_topic = str(self.get_parameter("motor_cmds_topic").value)
        self._publish_frequency = float(self.get_parameter("publish_frequency").value)
        self._start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self._test_duration_sec = float(self.get_parameter("test_duration_sec").value)
        self._hold_from_current_state = bool(self.get_parameter("hold_from_current_state").value)
        self._report_interval_sec = float(self.get_parameter("report_interval_sec").value)
        self._status_window_sec = float(self.get_parameter("status_window_sec").value)
        self._active_amplitude_threshold = float(self.get_parameter("active_amplitude_threshold").value)
        self._min_motion_span = float(self.get_parameter("min_motion_span").value)
        self._low_amplitude_ratio_threshold = float(self.get_parameter("low_amplitude_ratio_threshold").value)
        self._correlation_warn_threshold = float(self.get_parameter("correlation_warn_threshold").value)
        self._rmse_warn_threshold = float(self.get_parameter("rmse_warn_threshold").value)

        self._kp = self._read_vector_parameter("kp", DEFAULT_KP)
        self._kd = self._read_vector_parameter("kd", DEFAULT_KD)
        self._base_joint_pos = self._read_vector_parameter("default_joint_pos", DEFAULT_JOINT_POS)

        self._joint_indices = self._sanitize_joint_indices(self.get_parameter("joint_indices").value)
        self._amplitudes = self._read_broadcast_parameter("amplitudes", len(self._joint_indices), 0.15)
        self._frequencies_hz = self._read_broadcast_parameter("frequencies_hz", len(self._joint_indices), 0.25)
        self._phases_rad = self._read_broadcast_parameter("phases_rad", len(self._joint_indices), 0.0)
        self._center_override = self._read_optional_centers()

        self._obs_lock = threading.Lock()
        self._latest_pos = np.zeros(NUM_JOINTS, dtype=np.float64)
        self._latest_vel = np.zeros(NUM_JOINTS, dtype=np.float64)
        self._has_motor_state = False
        self._base_initialized = not self._hold_from_current_state
        self._test_complete_logged = False
        self._history = {joint_idx: deque() for joint_idx in self._joint_indices}

        if self._center_override is not None and self._base_initialized:
            for joint_idx, center in zip(self._joint_indices, self._center_override):
                self._base_joint_pos[joint_idx] = center

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sub_motors = self.create_subscription(MotorStates, self._motor_states_topic, self._cb_motor_states, qos_profile)
        self._pub_cmds = self.create_publisher(MotorCmds, self._motor_cmds_topic, 10)

        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(1.0 / self._publish_frequency, self._publish_cmd)
        self._report_timer = self.create_timer(self._report_interval_sec, self._report_status)

        for idx, amplitude, frequency, phase in zip(
            self._joint_indices, self._amplitudes, self._frequencies_hz, self._phases_rad
        ):
            self.get_logger().info(
                f"{JOINT_NAMES[idx]}: joint_index={idx}, amplitude={amplitude:.3f} rad, "
                f"frequency={frequency:.3f} Hz, phase={phase:.3f} rad"
            )
        self.get_logger().info(
            f"Joint test started: publish={self._publish_frequency:.1f} Hz, start_delay={self._start_delay_sec:.1f} s, "
            f"duration={self._test_duration_sec:.1f} s (0 means continuous)"
        )

    def _read_vector_parameter(self, name: str, default_values: np.ndarray) -> np.ndarray:
        values = np.asarray([float(v) for v in self.get_parameter(name).value], dtype=np.float64)
        if values.shape != default_values.shape:
            self.get_logger().warn(
                f"{name} length={len(values)} does not match expected {len(default_values)}, using defaults"
            )
            return default_values.copy()
        return values

    def _read_broadcast_parameter(self, name: str, expected_length: int, default_value: float) -> np.ndarray:
        values = [float(v) for v in self.get_parameter(name).value]
        if not values:
            return np.full(expected_length, default_value, dtype=np.float64)
        if len(values) == 1:
            return np.full(expected_length, values[0], dtype=np.float64)
        if len(values) != expected_length:
            self.get_logger().warn(
                f"{name} length={len(values)} does not match expected {expected_length}, using broadcast default {default_value}"
            )
            return np.full(expected_length, default_value, dtype=np.float64)
        return np.asarray(values, dtype=np.float64)

    def _read_optional_centers(self) -> np.ndarray | None:
        values = [float(v) for v in self.get_parameter("center_positions").value]
        if not values:
            return None
        if len(values) != len(self._joint_indices):
            self.get_logger().warn(
                f"center_positions length={len(values)} does not match expected {len(self._joint_indices)}, ignoring override"
            )
            return None
        return np.asarray(values, dtype=np.float64)

    def _sanitize_joint_indices(self, raw_indices: Sequence[object]) -> list[int]:
        filtered: list[int] = []
        for raw in raw_indices:
            idx = int(raw)
            if idx < 0 or idx >= NUM_JOINTS:
                self.get_logger().warn(f"joint index {idx} is out of range, skipping")
                continue
            if idx in WHEEL_SET:
                self.get_logger().warn(f"joint index {idx} ({JOINT_NAMES[idx]}) is a wheel joint, use wheel_test instead")
                continue
            if idx in filtered:
                self.get_logger().warn(f"joint index {idx} appears multiple times, keeping one entry")
                continue
            filtered.append(idx)

        if not filtered:
            self.get_logger().warn("No valid non-wheel joint indices provided, falling back to RF_Roll (0)")
            return [0]
        return filtered

    def _elapsed_sec(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    def _cb_motor_states(self, msg: MotorStates) -> None:
        if len(msg.states) < NUM_JOINTS:
            return

        pos = np.asarray([msg.states[i].pos for i in range(NUM_JOINTS)], dtype=np.float64)
        vel = np.asarray([msg.states[i].vel for i in range(NUM_JOINTS)], dtype=np.float64)

        with self._obs_lock:
            self._latest_pos[:] = pos
            self._latest_vel[:] = vel
            self._has_motor_state = True

            if self._hold_from_current_state and not self._base_initialized:
                self._base_joint_pos[:] = pos
                if self._center_override is not None:
                    for joint_idx, center in zip(self._joint_indices, self._center_override):
                        self._base_joint_pos[joint_idx] = center
                self._base_initialized = True
                self.get_logger().info("Captured current joint positions as sine-wave centers / hold targets")

    def _desired_positions(self, elapsed_sec: float) -> np.ndarray:
        desired = self._base_joint_pos.copy()
        if elapsed_sec < self._start_delay_sec:
            return desired
        if self._test_duration_sec > 0.0 and elapsed_sec >= self._start_delay_sec + self._test_duration_sec:
            if not self._test_complete_logged:
                self.get_logger().info("Joint test duration elapsed, holding base pose")
                self._test_complete_logged = True
            return desired

        active_time = elapsed_sec - self._start_delay_sec
        for local_idx, joint_idx in enumerate(self._joint_indices):
            desired[joint_idx] = self._base_joint_pos[joint_idx] + self._amplitudes[local_idx] * math.sin(
                2.0 * math.pi * self._frequencies_hz[local_idx] * active_time + self._phases_rad[local_idx]
            )
        return desired

    def _trim_history(self, now_sec: float) -> None:
        cutoff = now_sec - self._status_window_sec
        for history in self._history.values():
            while history and history[0][0] < cutoff:
                history.popleft()

    def _publish_cmd(self) -> None:
        with self._obs_lock:
            if not self._has_motor_state or not self._base_initialized:
                return
            latest_pos = self._latest_pos.copy()

        elapsed_sec = self._elapsed_sec()
        desired = self._desired_positions(elapsed_sec)

        msg = MotorCmds()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(NUM_JOINTS):
            cmd = MotorCmd()
            if i in WHEEL_SET:
                cmd.pos = 0.0
                cmd.vel = 0.0
            else:
                cmd.pos = float(desired[i])
                cmd.vel = 0.0
            cmd.tau = 0.0
            cmd.kp = float(self._kp[i])
            cmd.kd = float(self._kd[i])
            msg.cmds.append(cmd)
        self._pub_cmds.publish(msg)

        self._trim_history(elapsed_sec)
        for joint_idx in self._joint_indices:
            self._history[joint_idx].append((elapsed_sec, float(desired[joint_idx]), float(latest_pos[joint_idx])))

    def _report_status(self) -> None:
        with self._obs_lock:
            if not self._has_motor_state or not self._base_initialized:
                self.get_logger().info("Waiting for /motor_states before starting joint test")
                return

        now_sec = self._elapsed_sec()
        self._trim_history(now_sec)

        for joint_idx in self._joint_indices:
            history = self._history[joint_idx]
            if len(history) < 4:
                self.get_logger().info(f"{JOINT_NAMES[joint_idx]}: status=WARMUP")
                continue

            desired = np.asarray([item[1] for item in history], dtype=np.float64)
            measured = np.asarray([item[2] for item in history], dtype=np.float64)

            cmd_now = float(desired[-1])
            meas_now = float(measured[-1])
            err_now = cmd_now - meas_now
            desired_span = float(np.ptp(desired))
            measured_span = float(np.ptp(measured))
            rmse = float(np.sqrt(np.mean(np.square(desired - measured))))

            desired_dev = desired - np.mean(desired)
            measured_dev = measured - np.mean(measured)
            desired_std = float(np.std(desired_dev))
            measured_std = float(np.std(measured_dev))
            if desired_std > 1e-6 and measured_std > 1e-6:
                corr = float(np.corrcoef(desired_dev, measured_dev)[0, 1])
                amplitude_ratio = measured_std / desired_std
            else:
                corr = float("nan")
                amplitude_ratio = float("nan")

            status = "OK"
            if desired_span < 2.0 * self._active_amplitude_threshold:
                status = "IDLE"
            elif measured_span < self._min_motion_span:
                status = "STUCK_OR_NO_MOTION"
            elif np.isfinite(corr) and corr < self._correlation_warn_threshold:
                status = "INVERTED_RESPONSE"
            elif np.isfinite(amplitude_ratio) and amplitude_ratio < self._low_amplitude_ratio_threshold:
                status = "LOW_AMPLITUDE"
            elif rmse > self._rmse_warn_threshold:
                status = "LARGE_TRACKING_ERROR"

            corr_text = f"{corr:+.2f}" if np.isfinite(corr) else "nan"
            amp_text = f"{amplitude_ratio:.2f}" if np.isfinite(amplitude_ratio) else "nan"
            self.get_logger().info(
                f"{JOINT_NAMES[joint_idx]}: cmd={cmd_now:+.3f} rad, meas={meas_now:+.3f} rad, err={err_now:+.3f} rad, "
                f"span={measured_span:.3f} rad, amp_ratio={amp_text}, corr={corr_text}, rmse={rmse:.3f}, status={status}"
            )

    def stop(self) -> None:
        with self._obs_lock:
            if not self._has_motor_state or not self._base_initialized:
                return
            hold_pos = self._base_joint_pos.copy()

        msg = MotorCmds()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(NUM_JOINTS):
            cmd = MotorCmd()
            if i in WHEEL_SET:
                cmd.pos = 0.0
                cmd.vel = 0.0
            else:
                cmd.pos = float(hold_pos[i])
                cmd.vel = 0.0
            cmd.tau = 0.0
            cmd.kp = float(self._kp[i])
            cmd.kd = float(self._kd[i])
            msg.cmds.append(cmd)
        self._pub_cmds.publish(msg)


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JointTestNode()
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
