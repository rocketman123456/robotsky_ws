"""ROS 2 RL Controller Node for robotsky_wq wheel-legged quadruped."""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import List, Optional, Union

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from robotsky_interface.msg import MotorCmd, MotorCmds, MotorState, MotorStates

from .policy import MLPPolicy, OnnxPolicy


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

NUM_JOINTS = 16

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

# Observation dimensions (match robotsky_wq_env.py actor_obs)
# [ang_vel(3), projected_gravity(3), cmd(3), joint_pos(16), joint_vel(16), action(16)]
OBS_ANG_VEL = 3
OBS_GRAVITY = 3
OBS_CMD = 3
OBS_JPOS = 16
OBS_JVEL = 16
OBS_PREV_ACT = 16
OBS_DIM = OBS_ANG_VEL + OBS_GRAVITY + OBS_CMD + OBS_JPOS + OBS_JVEL + OBS_PREV_ACT  # 57
ACTION_DIM = NUM_JOINTS

# Joint order mapping between:
# - IsaacLab/policy order (used by obs/action tensors)
# - MotorStates/MotorCmds order (used by ROS motor interface + MuJoCo XML)
#
# This must stay consistent with the exported policy + the simulator joint order.
# ISAAC_TO_MOTOR_IDX = np.array([3, 7, 11, 15, 1, 5, 9, 13, 2, 6, 10, 14, 0, 4, 8, 12], dtype=np.int64)
# MOTOR_TO_ISAAC_IDX = np.array([12, 4, 8, 0, 13, 5, 9, 1, 14, 6, 10, 2, 15, 7, 11, 3], dtype=np.int64)
ISAAC_TO_MOTOR_IDX = np.array([3, 7, 11, 15, 1, 5, 9, 13, 2, 6, 10, 14, 0, 4, 8, 12], dtype=np.int64)
MOTOR_TO_ISAAC_IDX = np.array([12, 4, 8, 0, 13, 5, 9, 1, 14, 6, 10, 2, 15, 7, 11, 3], dtype=np.int64)


# ---------------------------------------------------------------------------
# RLControllerNode
# ---------------------------------------------------------------------------


class RLControllerNode(Node):
    """
    ROS 2 node that runs an RL policy at a configurable frequency and
    publishes joint commands to /motor_cmds.

    Parameters (ROS):
        control_frequency (double, default 50.0)  : inference loop Hz
        cpu_core          (int,    default -1)     : CPU affinity; -1 = no binding
        model_path        (string, default "")     : path to .onnx or .pt/.pth;
                                                     empty = random MLP weights
        action_scale      (double, default 0.25)   : scale applied to policy output
        wheel_action_scale (double, default 4.0)  : scale applied to wheel joint actions
        clip_observations (double, default 100.0) : clip obs values before network
        clip_actions      (double, default 100.0) : clip actions before publishing / buffering
        kp                (double, default 20.0)   : position gain sent with every cmd
        kd                (double, default 0.5)    : damping gain sent with every cmd

    Topics:
        /motor_states  (MotorStates) ← subscribe
        /robotsky_imu  (Imu)         ← subscribe
        /cmd_vel       (Twist)       ← subscribe (optional)
        /motor_cmds    (MotorCmds)   → publish
    """

    def __init__(self) -> None:
        super().__init__("rl_controller")

        # ----------------------------------------------------------------
        # Declare & read parameters
        # ----------------------------------------------------------------
        self.declare_parameter("control_frequency", 50.0)
        self.declare_parameter("cpu_core", -1)
        self.declare_parameter("model_path", "")
        self.declare_parameter("action_scale", 0.25)
        self.declare_parameter("wheel_action_scale", 4.0)
        # fmt:off
        self.declare_parameter("kp", [
            20.0, 20.0, 40.0, 0.0,
            20.0, 20.0, 40.0, 0.0,
            20.0, 20.0, 40.0, 0.0,
            20.0, 20.0, 40.0, 0.0,
        ])
        self.declare_parameter("kd", [
            1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0,
        ])
        # fmt:on
        self.declare_parameter("wheel_joint_indices", [3, 7, 11, 15])
        # Default joint positions in *motor* (MotorStates) index order.
        # Matches the init_state used by the IsaacLab training environment.
        # fmt:off
        self.declare_parameter("default_joint_pos", [
                0.1, -0.5, 1.0, 0.0,  # RF
                -0.1, -0.5, 1.0, 0.0,  # LF
                0.1, 0.5, -1.0, 0.0,  # RB
                -0.1, 0.5, -1.0, 0.0,  # LB
            ],
        )
        # fmt:on
        self.declare_parameter("default_joint_vel", [0.0] * NUM_JOINTS)
        self.declare_parameter("obs_scale_ang_vel", 1.0)
        self.declare_parameter("obs_scale_projected_gravity", 1.0)
        self.declare_parameter("obs_scale_commands", 1.0)
        self.declare_parameter("obs_scale_joint_pos", 1.0)
        # IsaacLab env: obs_scales.joint_vel = 1.0 for leg joints (wheel joints use obs_scales.wheel_vel = 0.1)
        self.declare_parameter("obs_scale_joint_vel_leg", 1.0)
        self.declare_parameter("obs_scale_wheel_vel", 0.1)
        self.declare_parameter("obs_scale_actions", 1.0)
        # Some exported policies are trained with stacked actor observations:
        # obs_dim = actor_obs_dim * obs_history_len = 57 * 10 = 570.
        self.declare_parameter("obs_history_len", 10)
        self.declare_parameter("clip_observations", 100.0)
        self.declare_parameter("clip_actions", 100.0)
        self.declare_parameter("pause_topic", "/pause_flag")
        # When true, controller will stop rolling obs_history and will not publish MotorCmds.
        self.declare_parameter("pause_initial", True)

        self._freq = self.get_parameter("control_frequency").value
        self._cpu_core = self.get_parameter("cpu_core").value
        self._model_path = self.get_parameter("model_path").value
        self._kp = self.get_parameter("kp").value
        self._kd = self.get_parameter("kd").value
        self._action_scale = float(self.get_parameter("action_scale").value)
        self._wheel_action_scale = float(self.get_parameter("wheel_action_scale").value)

        wheel_indices = list(self.get_parameter("wheel_joint_indices").value)
        self._wheel_joint_ids = [int(i) for i in wheel_indices]
        self._leg_joint_ids = [i for i in range(NUM_JOINTS) if i not in self._wheel_joint_ids]
        self._wheel_mask_motor = np.zeros(NUM_JOINTS, dtype=bool)
        self._wheel_mask_motor[self._wheel_joint_ids] = True

        # These masks are in ISAAC/policy order (after MOTOR_TO_ISAAC_IDX reordering).
        self._wheel_joint_ids_isaac = [isaac_idx for isaac_idx in range(NUM_JOINTS) if int(MOTOR_TO_ISAAC_IDX[isaac_idx]) in self._wheel_joint_ids]
        self._leg_joint_ids_isaac = [i for i in range(NUM_JOINTS) if i not in self._wheel_joint_ids_isaac]

        self._default_joint_pos = np.array(self.get_parameter("default_joint_pos").value, dtype=np.float64)
        self._default_joint_vel = np.array(self.get_parameter("default_joint_vel").value, dtype=np.float64)
        if self._default_joint_pos.shape[0] != NUM_JOINTS:
            self.get_logger().warn("default_joint_pos length != 16, using zeros")
            self._default_joint_pos = np.zeros(NUM_JOINTS, dtype=np.float64)
        if self._default_joint_vel.shape[0] != NUM_JOINTS:
            self.get_logger().warn("default_joint_vel length != 16, using zeros")
            self._default_joint_vel = np.zeros(NUM_JOINTS, dtype=np.float64)

        self._obs_scale_ang_vel = float(self.get_parameter("obs_scale_ang_vel").value)
        self._obs_scale_projected_gravity = float(self.get_parameter("obs_scale_projected_gravity").value)
        self._obs_scale_commands = float(self.get_parameter("obs_scale_commands").value)
        self._obs_scale_joint_pos = float(self.get_parameter("obs_scale_joint_pos").value)
        self._obs_scale_joint_vel_leg = float(self.get_parameter("obs_scale_joint_vel_leg").value)
        self._obs_scale_wheel_vel = float(self.get_parameter("obs_scale_wheel_vel").value)
        self._obs_scale_actions = float(self.get_parameter("obs_scale_actions").value)
        self._obs_history_len = int(self.get_parameter("obs_history_len").value)
        if self._obs_history_len <= 0:
            self.get_logger().warn("obs_history_len must be > 0, fallback to 1")
            self._obs_history_len = 1

        self._clip_observations = float(self.get_parameter("clip_observations").value)
        self._clip_actions = float(self.get_parameter("clip_actions").value)
        if self._clip_observations <= 0:
            self.get_logger().warn("clip_observations <= 0, disabled clipping")
            self._clip_observations = 0.0
        if self._clip_actions <= 0:
            self.get_logger().warn("clip_actions <= 0, disabled action clipping")
            self._clip_actions = 0.0

        self._paused = bool(self.get_parameter("pause_initial").value)

        self.get_logger().info(f"RL Controller | freq={self._freq} Hz | " f"cpu_core={self._cpu_core} | " f"kp={self._kp} | kd={self._kd}")

        # ----------------------------------------------------------------
        # Observation state (protected by _obs_lock)
        # ----------------------------------------------------------------
        self._obs_lock = threading.Lock()
        self._quat = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self._ang_vel = np.zeros(3)
        self._projected_gravity = np.array([0.0, 0.0, -1.0])
        self._command = np.zeros(3)  # vx, vy, yaw_rate
        self._joint_pos = np.zeros(NUM_JOINTS)
        self._joint_vel = np.zeros(NUM_JOINTS)
        self._prev_action = np.zeros(ACTION_DIM)
        self._obs_history = np.zeros((self._obs_history_len, OBS_DIM), dtype=np.float64)
        self._has_imu = False
        self._has_motors = False

        # ----------------------------------------------------------------
        # Policy (.onnx via ONNX Runtime, or .pt/.pth via PyTorch)
        # ----------------------------------------------------------------
        obs_dim_total = OBS_DIM * self._obs_history_len
        self._policy: Union[MLPPolicy, OnnxPolicy] = self._init_policy(obs_dim_total)

        # ----------------------------------------------------------------
        # ROS 2 subscribers & publisher
        # ----------------------------------------------------------------
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self._sub_motors = self.create_subscription(MotorStates, "/motor_states", self._cb_motor_states, qos_profile)
        self._sub_imu = self.create_subscription(Imu, "/robotsky_imu", self._cb_imu, qos_profile)
        self._sub_cmd = self.create_subscription(Twist, "/cmd_vel", self._cb_cmd_vel, qos_profile)
        self._pub_cmds = self.create_publisher(MotorCmds, "/motor_cmds", 10)  #qos_profile
        self._sub_pause = self.create_subscription(Bool, self.get_parameter("pause_topic").value, self._cb_pause_flag, 10)

        # ----------------------------------------------------------------
        # Inference thread (fixed-rate, separate from ROS spin thread)
        # ----------------------------------------------------------------
        self._running = True
        self._inference_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._inference_thread.start()

        self.get_logger().info("RLControllerNode started.")

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _cb_motor_states(self, msg: MotorStates) -> None:
        if len(msg.states) < NUM_JOINTS:
            return
        with self._obs_lock:
            for i in range(NUM_JOINTS):
                self._joint_pos[i] = msg.states[i].pos
                self._joint_vel[i] = msg.states[i].vel
            self._has_motors = True

    def _cb_imu(self, msg: Imu) -> None:
        with self._obs_lock:
            o = msg.orientation
            self._quat[:] = [o.w, o.x, o.y, o.z]
            av = msg.angular_velocity
            self._ang_vel[:] = [av.x, av.y, av.z]
            self._projected_gravity[:] = self._compute_projected_gravity(self._quat)
            self._has_imu = True

    def _cb_cmd_vel(self, msg: Twist) -> None:
        with self._obs_lock:
            self._command[:] = [msg.linear.x, msg.linear.y, msg.angular.z]

    # ------------------------------------------------------------------
    # Inference loop
    # ------------------------------------------------------------------

    def _inference_loop(self) -> None:
        """Fixed-rate inference loop running in a dedicated thread."""
        self._bind_cpu_core()

        interval = 1.0 / self._freq
        next_time = time.monotonic() + interval

        while self._running:
            self._step()

            now = time.monotonic()
            sleep_time = next_time - now
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Missed deadline — don't accumulate lag
                self.get_logger().warn(f"Inference loop overrun by {-sleep_time * 1000:.1f} ms", throttle_duration_sec=1.0)
            next_time += interval

    def _step(self) -> None:
        """Build observation → run policy → publish action."""
        with self._obs_lock:
            if not (self._has_imu and self._has_motors):
                return  # wait until both topics received at least once
            if self._paused:
                return
            base_obs = self._build_obs()
            # actor_obs stacking (most recent frame at the end)
            self._obs_history = np.roll(self._obs_history, -1, axis=0)
            self._obs_history[-1, :] = base_obs
            obs = self._obs_history.reshape(-1).copy()

        action_isaac = self._policy.forward(obs)  # (action_dim,) in policy order
        if self._clip_actions > 0.0:
            action_isaac = np.clip(action_isaac, -self._clip_actions, self._clip_actions)

        action_mujoco = action_isaac[ISAAC_TO_MOTOR_IDX]

        # Observations must keep `prev_action` in the same order as the model's
        # expected actor_obs layout (isaac order), while motor commands need
        # mujoco order.
        with self._obs_lock:
            if self._paused:
                return
            self._prev_action[:] = action_isaac

        with self._obs_lock:
            if self._paused:
                return

        # self.get_logger().info(f"Action (mujoco): {action_mujoco}")
        self._publish_action(action_mujoco)

    def _build_obs(self) -> np.ndarray:
        """Concatenate observation vector (called with _obs_lock held)."""
        # Match env observation strategy:
        # - joint_pos is deviation from default; wheel positions are zeroed
        # - joint_vel is deviation from default; scaled separately for leg & wheel joints
        # MotorStates order → Isaac/policy order (so wheel ids match the exported observation layout).
        joint_pos_isaac = (self._joint_pos - self._default_joint_pos)[MOTOR_TO_ISAAC_IDX].copy()
        joint_vel_isaac = (self._joint_vel - self._default_joint_vel)[MOTOR_TO_ISAAC_IDX].copy()

        # Isaac/policy order: wheel positions are zeroed (no positional info), but wheel velocities stay.
        joint_pos_isaac[self._wheel_joint_ids_isaac] = 0.0
        joint_vel_isaac[self._leg_joint_ids_isaac] *= self._obs_scale_joint_vel_leg
        joint_vel_isaac[self._wheel_joint_ids_isaac] *= self._obs_scale_wheel_vel

        base_obs = np.concatenate(
            [
                self._ang_vel * self._obs_scale_ang_vel,  # 3
                self._projected_gravity * self._obs_scale_projected_gravity,  # 3
                self._command * self._obs_scale_commands,  # 3
                joint_pos_isaac * self._obs_scale_joint_pos,  # 16
                joint_vel_isaac,  # 16
                self._prev_action * self._obs_scale_actions,  # 16
            ]
        )  # total: 57

        if self._clip_observations > 0.0:
            base_obs = np.clip(base_obs, -self._clip_observations, self._clip_observations)

        return base_obs

    def _publish_action(self, action: np.ndarray) -> None:
        msg = MotorCmds()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(NUM_JOINTS):
            cmd = MotorCmd()
            a = float(action[i])
            if self._wheel_mask_motor[i]:
                # Wheel joints are velocity-controlled in the robot config.
                # In simulation, `robotsky_sim` currently forwards `MotorCmd.pos` as
                # Mujoco `data.ctrl`, so we write the desired velocity into both
                # `pos` and `vel` to keep sim + real consistent.
                wheel_vel = float(a * self._wheel_action_scale)
                # rosidl_py requires builtin float (not int / numpy scalar) for float64 fields.
                cmd.pos = 0.0
                cmd.vel = wheel_vel
            else:
                # Leg joints: position target around default
                cmd.pos = float(a * self._action_scale + self._default_joint_pos[i])
                cmd.vel = 0.0
            cmd.tau = 0.0
            cmd.kp = float(self._kp[i])
            cmd.kd = float(self._kd[i])
            msg.cmds.append(cmd)
        self._pub_cmds.publish(msg)

    def _cb_pause_flag(self, msg: Bool) -> None:
        with self._obs_lock:
            self._paused = bool(msg.data)

    # ------------------------------------------------------------------
    # Math helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _compute_projected_gravity(quat_wxyz: np.ndarray) -> np.ndarray:
        # Replicates IsaacLab's projected_gravity_b meaning: gravity vector expressed in body frame.
        # Using world gravity [0, 0, -1] rotated into body frame via q^{-1} * g * q.
        w, x, y, z = quat_wxyz.tolist()
        # Rotation matrix from body->world for unit quaternion (w,x,y,z)
        r00 = 1.0 - 2.0 * (y * y + z * z)
        r01 = 2.0 * (x * y - z * w)
        r02 = 2.0 * (x * z + y * w)
        r10 = 2.0 * (x * y + z * w)
        r11 = 1.0 - 2.0 * (x * x + z * z)
        r12 = 2.0 * (y * z - x * w)
        r20 = 2.0 * (x * z - y * w)
        r21 = 2.0 * (y * z + x * w)
        r22 = 1.0 - 2.0 * (x * x + y * y)
        # world gravity
        gw = np.array([0.0, 0.0, -1.0])
        # body gravity = R^T * gw
        gb = np.array(
            [
                r00 * gw[0] + r10 * gw[1] + r20 * gw[2],
                r01 * gw[0] + r11 * gw[1] + r21 * gw[2],
                r02 * gw[0] + r12 * gw[1] + r22 * gw[2],
            ]
        )
        return gb

    # ------------------------------------------------------------------
    # CPU affinity
    # ------------------------------------------------------------------

    def _bind_cpu_core(self) -> None:
        """Bind the inference thread to a specific CPU core if configured."""
        if self._cpu_core < 0:
            return
        try:
            os.sched_setaffinity(0, {self._cpu_core})
            self.get_logger().info(f"Inference thread bound to CPU core {self._cpu_core}")
        except (AttributeError, OSError) as e:
            self.get_logger().warn(f"CPU affinity not set: {e}")

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self._running = False
        self._inference_thread.join(timeout=2.0)
        super().destroy_node()

    # ------------------------------------------------------------------
    # Path helpers
    # ------------------------------------------------------------------

    def _init_policy(self, obs_dim: int) -> Union[MLPPolicy, OnnxPolicy]:
        action_scale_for_policy = 1.0

        def random_mlp() -> MLPPolicy:
            return MLPPolicy(
                obs_dim=obs_dim,
                action_dim=ACTION_DIM,
                hidden_dims=[512, 256, 128],
                activation="elu",
                action_scale=action_scale_for_policy,
                device="cpu",
            )

        if not self._model_path:
            self.get_logger().warn("No model_path set — using random weights")
            return random_mlp()

        resolved = self._resolve_model_path(self._model_path)
        if resolved is None:
            self.get_logger().warn(
                f"Model not found for model_path='{self._model_path}'. Using random weights."
            )
            return random_mlp()

        self.get_logger().info(f"Loading model from {resolved}")
        if resolved.suffix.lower() == ".onnx":
            policy = OnnxPolicy(
                obs_dim=obs_dim,
                action_dim=ACTION_DIM,
                action_scale=action_scale_for_policy,
            )
            policy.load(resolved)
            return policy

        policy = random_mlp()
        policy.load(resolved)
        return policy

    def _resolve_model_path(self, model_path: str) -> Optional[Path]:
        """
        Resolve a checkpoint path robustly across dev/install layouts.

        Search order:
        - absolute path
        - relative to current working directory
        - relative to package share directory
        - relative to package source directory (common in colcon --symlink-install)
        """
        p = Path(model_path)
        candidates: List[Path] = []

        if p.is_absolute():
            candidates.append(p)
        else:
            candidates.append(Path.cwd() / p)
            try:
                share = Path(get_package_share_directory("robotsky_rl_controller"))
                candidates.append(share / p)
            except Exception:
                pass
            try:
                # .../robotsky_rl_controller/rl_controller_node.py -> package dir -> src root
                src_root = Path(__file__).resolve().parents[2]
                candidates.append(src_root / p)
            except Exception:
                pass

        for c in candidates:
            if c.exists():
                return c

        self.get_logger().warn("Tried model paths: " + ", ".join(str(c) for c in candidates))
        return None


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RLControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # Ctrl-C can trigger shutdown already in some ROS 2 versions.
            pass


if __name__ == "__main__":
    main()
