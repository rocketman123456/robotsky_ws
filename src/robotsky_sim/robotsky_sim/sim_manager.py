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

        self.pub_joints = self.create_publisher(MotorStates, "/motor_states", 10)
        self.pub_rviz = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_imu = self.create_publisher(Imu, "/srobot_imu", 10)
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
            action = [
                0.0, -0.5, 1.0, 0.0, #
                0.0, -0.5, 1.0, 0.0, #
                0.0, 0.5, -1.0, 0.0, #
                0.0, 0.5, -1.0, 0.0, #
            ]
            state = self.get_state()
            self.set_action(action)

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

    def publish_ros_state(self):
        sim_state = Bool()
        sim_state.data = self.sim.pause_flag
        self.pub_sim_state.publish(sim_state)
