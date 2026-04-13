from __future__ import annotations

from typing import Sequence

from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Bool

from robotsky_interface.msg import MotorCmds, MotorState, MotorStates

from .sim import RobotCfg, SceneCfg, SimulationCfg

NUM_JOINTS = 16
JOINT_NAMES = [
    "RF_Roll_Joint",
    "RF_Hip_Joint",
    "RF_Knee_Joint",
    "RF_Wheel_Joint",
    "LF_Roll_Joint",
    "LF_Hip_Joint",
    "LF_Knee_Joint",
    "LF_Wheel_Joint",
    "RB_Roll_Joint",
    "RB_Hip_Joint",
    "RB_Knee_Joint",
    "RB_Wheel_Joint",
    "LB_Roll_Joint",
    "LB_Hip_Joint",
    "LB_Knee_Joint",
    "LB_Wheel_Joint",
]

class SimManager(Node):
    def __init__(self, sim_cfg: SimulationCfg, robot_cfg: RobotCfg, scene_cfg: SceneCfg):
        super().__init__("robotsky_sim")

        if sim_cfg.simulator_type == "none":
            raise ValueError("Simulator type must be specified.")
        if sim_cfg.simulator_type == "pybullet":
            from .sim.pybullet_sim import PybulletSim

            self.sim = PybulletSim(sim_cfg=sim_cfg)
        elif sim_cfg.simulator_type == "mujoco":
            from .sim.mujoco_sim import MujocoSim

            self.sim = MujocoSim(sim_cfg=sim_cfg)
        elif sim_cfg.simulator_type == "genesis":
            from .sim.genesis_sim import GenesisSim

            self.sim = GenesisSim(sim_cfg=sim_cfg)
        else:
            raise ValueError(f"Unknown simulator type: {sim_cfg.simulator_type}")

        self.sim.initialize(robot_cfg=robot_cfg, scene_cfg=scene_cfg)

        self.prev_pause_flag = bool(self.sim.pause_flag)
        self.curr_pause_flag = bool(self.sim.pause_flag)
        self._pause_flag_initialized = False

        self.joint_state = JointState()
        self.joint_state.name = JOINT_NAMES
        self.joint_state.position = NUM_JOINTS * [0.0]
        self.joint_state.velocity = NUM_JOINTS * [0.0]
        self.joint_state.effort = NUM_JOINTS * [0.0]

        # add action subscriber
        self.sub_cmds = self.create_subscription(MotorCmds, "/motor_cmds", self.receive_ros_action, 10)

        # add state publisher
        self.pub_joints = self.create_publisher(MotorStates, "/motor_states", 10)
        self.pub_rviz = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_imu = self.create_publisher(Imu, "/robotsky_imu", 10)
        self.pub_sim_state = self.create_publisher(Bool, "/pause_flag", 10)

        timer_period = 1.0 / 500.0  # seconds
        self._timer_period = timer_period
        self.timer = self.create_timer(timer_period, self.step)

        # MuJoCo: publish /robotsky_imu at 200 Hz to match real robot IMU; motor_states stay 500 Hz.
        self._imu_pub_period_s: float | None = None
        self._imu_pub_accum = 0.0
        if sim_cfg.simulator_type == "mujoco":
            self._imu_pub_period_s = 1.0 / 200.0

    def is_running(self):
        return self.sim.is_running()

    def set_action(self, action):
        self.sim.set_action(action)

    def get_state(self):
        return self.sim.get_state()

    def step(self):
        try:
            self.sim.step()
            self.sim.render()
            self.publish_ros_state()
        except Exception as e:
            self.get_logger().error(f"step error: {e}")

    def run(self):
        while self.is_running():
            self.step()

        self.sim.finalize()

    def receive_ros_action(self, msg: MotorCmds):
        self.sim.receive_ros_action(msg)
        # if len(msg.cmds) >= 16:
        #     action = [msg.cmds[i].pos for i in range(16)]
        #     self.sim.set_action(action)

    def _pub_pause_flag(self):
        self.curr_pause_flag = self.sim.pause_flag
        if not self._pause_flag_initialized or self.curr_pause_flag != self.prev_pause_flag:
            sim_state = Bool()
            sim_state.data = self.sim.pause_flag
            self.pub_sim_state.publish(sim_state)
            self._pause_flag_initialized = True
        self.prev_pause_flag = self.curr_pause_flag

    def _pub_motor_state(self, t, qp, qv):
        motor_state_msg = MotorStates()
        motor_state_msg.header.stamp = t.to_msg()
        for i in range(NUM_JOINTS):
            obj = MotorState()
            obj.pos = float(qp[i])
            obj.vel = float(qv[i])
            obj.tau = 0.0
            motor_state_msg.states.append(obj)

        self.pub_joints.publish(motor_state_msg)

    def _pub_motor_state_rviz(self, t, qp, qv):
        self.joint_state.header.stamp = t.to_msg()
        for i in range(NUM_JOINTS):
            self.joint_state.position[i] = float(qp[i])
            self.joint_state.velocity[i] = float(qv[i])

        self.pub_rviz.publish(self.joint_state)

    def _pub_imu(self, t, quat, gyro, acc):
        imu_msg = Imu()
        imu_msg.header.stamp = t.to_msg()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.orientation.w = float(quat[0])
        imu_msg.orientation.x = float(quat[1])
        imu_msg.orientation.y = float(quat[2])
        imu_msg.orientation.z = float(quat[3])

        imu_msg.angular_velocity.x = float(gyro[0])
        imu_msg.angular_velocity.y = float(gyro[1])
        imu_msg.angular_velocity.z = float(gyro[2])

        imu_msg.linear_acceleration.x = float(acc[0])
        imu_msg.linear_acceleration.y = float(acc[1])
        imu_msg.linear_acceleration.z = float(acc[2])

        self.pub_imu.publish(imu_msg)

    @staticmethod
    def _ensure_length(values: Sequence[float], expected_length: int, fill_value: float = 0.0) -> list[float]:
        normalized = [float(v) for v in values[:expected_length]]
        if len(normalized) < expected_length:
            normalized.extend([fill_value] * (expected_length - len(normalized)))
        return normalized

    def publish_ros_state(self):
        t = self.get_clock().now()
        qp, qv, quat, gyro, acc = self.get_state()
        qp = self._ensure_length(qp, NUM_JOINTS)
        qv = self._ensure_length(qv, NUM_JOINTS)
        quat = self._ensure_length(quat, 4)
        gyro = self._ensure_length(gyro, 3)
        acc = self._ensure_length(acc, 3)

        self._pub_pause_flag()
        self._pub_motor_state(t, qp, qv)
        self._pub_motor_state_rviz(t, qp, qv)
        if self._imu_pub_period_s is None:
            self._pub_imu(t, quat, gyro, acc)
        else:
            self._imu_pub_accum += self._timer_period
            if self._imu_pub_accum >= self._imu_pub_period_s:
                self._imu_pub_accum -= self._imu_pub_period_s
                self._pub_imu(t, quat, gyro, acc)
