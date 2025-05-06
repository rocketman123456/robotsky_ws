from .sim import *
import time

from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState, Imu
from robotsky_interface.msg import MotorCmds, MotorStates, MotorCmd, MotorState

class SimManager(Node):
    def __init__(self, sim_cfg: SimulationCfg, robot_cfg: RobotCfg, scene_cfg: SceneCfg):
        super().__init__("robotsky_sim")

        # self.get_logger().info('xml_path: "%s"' % xml_path)

        if sim_cfg.simulatior_type == "none":
            self.sim = SimBase(sim_cfg=sim_cfg)
            raise ValueError("Simulator type must be specified.")
        elif sim_cfg.simulatior_type == "pybullet":
            from .sim.pybullet_sim import PybulletSim

            self.sim = PybulletSim(sim_cfg=sim_cfg)
        elif sim_cfg.simulatior_type == "mujoco":
            from .sim.mujoco_sim import MujocoSim

            self.sim = MujocoSim(sim_cfg=sim_cfg)
        elif sim_cfg.simulatior_type == "genesis":
            from .sim.genesis_sim import GenesisSim

            self.sim = GenesisSim(sim_cfg=sim_cfg)
        else:
            raise ValueError(f"Unknown simulator type: {sim_cfg.simulatior_type}")
        
        self.sim.initialize(robot_cfg=robot_cfg, scene_cfg=scene_cfg)

        self.prev_pause_flag = True
        self.curr_pause_flag = True

        self.joint_state = JointState()
        self.joint_state.name = [
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

        # add action subscriber
        self.sub_cmds = self.create_subscription(MotorCmds, "/motor_cmds", self.receive_ros_action, 10)

        # add state publisher
        self.pub_joints = self.create_publisher(MotorStates, "/motor_states", 10)
        self.pub_rviz = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_imu = self.create_publisher(Imu, "/robotsky_imu", 10)
        self.pub_sim_state = self.create_publisher(Bool, "/pause_flag", 10)

        timer_period = 1.0 / 500.0  # seconds
        self.timer = self.create_timer(timer_period, self.step)

    def is_running(self):
        return self.sim.is_running()

    def set_action(self, action):
        self.sim.set_action(action)

    def get_state(self):
        return self.sim.get_state()

    def step(self):
        try:
            # send state msg
            self.publish_ros_state()

            self.sim.step()
            self.sim.render()
        except:
            print()

    def run(self):
        while self.is_running():
            self.step()

        self.sim.finalize()

    def receive_ros_action(self, action):
        pass

    def _pub_pause_flag(self):
        # TODO : check
        self.curr_pause_flag = self.sim.pause_flag
        if self.curr_pause_flag != self.prev_pause_flag:
            sim_state = Bool()
            sim_state.data = self.sim.pause_flag
            self.pub_sim_state.publish(sim_state)
        self.prev_pause_flag = self.curr_pause_flag

    def _pub_motor_state(self, t, qp, qv):
        motor_state_msg = MotorStates()
        motor_state_msg.header.stamp = t.to_msg()
        for i in range(16):
            obj = MotorState()
            obj.pos = qp[i]
            obj.vel = qv[i]
            obj.tau = 0.0
            motor_state_msg.states.append(obj)

        self.pub_joints.publish(motor_state_msg)

    def _pub_motor_state_rviz(self, t, qp, qv):
        self.joint_state.header.stamp = t.to_msg()
        self.joint_state.position = 16 * [float(0.0)]
        self.joint_state.velocity = 16 * [float(0.0)]
        for i in range(16):
            self.joint_state.position[i] = float(qp[i])
            self.joint_state.velocity[i] = float(qv[i])

        self.pub_rviz.publish(self.joint_state)

    def _pub_imu(self, t, quat, gyro, acc):
        imu_msg = Imu()
        imu_msg.header.stamp = t.to_msg()

        imu_msg.orientation.w = quat[0]
        imu_msg.orientation.x = quat[1]
        imu_msg.orientation.y = quat[2]
        imu_msg.orientation.z = quat[3]

        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]

        imu_msg.linear_acceleration.x = acc[0]
        imu_msg.linear_acceleration.y = acc[1]
        imu_msg.linear_acceleration.z = acc[2]

        self.pub_imu.publish(imu_msg)

    def publish_ros_state(self):
        t = self.get_clock().now()
        qp, qv, quat, gyro, acc = self.get_state()

        self._pub_pause_flag()
        self._pub_motor_state(t, qp, qv)
        self._pub_motor_state_rviz(t, qp, qv)
        self._pub_imu(t, quat, gyro, acc)
