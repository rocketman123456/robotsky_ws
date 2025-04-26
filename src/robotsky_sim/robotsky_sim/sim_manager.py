from .sim import *
import time


class SimManager:
    def __init__(self, sim_cfg: SimulationCfg, robot_cfg: RobotCfg, scene_cfg: SceneCfg):
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

    def is_running(self):
        return self.sim.is_running()

    def step(self):
        state = self.sim.get_state()
        action = []  # Replace with actual action logic
        self.sim.set_action(action)

        # send state msg

        self.sim.step()
        self.sim.render()

    def run(self):
        # self.sim.reset()

        while self.is_running():
            self.step()

        self.sim.finalize()

    def receive_ros_action(self, action):
        pass

    def publish_ros_state(self, state):
        pass
