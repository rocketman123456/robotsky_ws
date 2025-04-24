from .sim_base import SimBase
from .sim_config import *

import numpy as np
import pybullet
import pybullet_data
import time


class PybulletSim(SimBase):
    def __init__(self, sim_cfg: SimulationCfg):
        super().__init__(sim_cfg)
        if sim_cfg.headless:
            self.physicsClient = x.connect(pybullet.DIRECT)
        else:
            self.physicsClient = pybullet.connect(pybullet.GUI)

    def initialize(self, robot_cfg: RobotCfg, scene_cfg: SceneCfg):
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.81)

        # Load the robot model
        self.robot_id = pybullet.loadURDF(robot_cfg.robot_asset_path)
        self.start_pos = robot_cfg.robot_init_position
        self.start_orientation = pybullet.getQuaternionFromEuler(robot_cfg.robot_init_orientation)
        pybullet.resetBasePositionAndOrientation(self.robot_id, self.start_pos, self.start_orientation)

        # load scene
        self.plane_id = pybullet.loadURDF("plane.urdf")

    def finalize(self):
        pybullet.disconnect()

    def reset(self):
        pass

    def is_running(self):
        return True

    def step(self):
        # TODO : add action
        pybullet.stepSimulation()

    def render(self):
        pass

    def get_state(self):
        pass

    def set_action(self, action):
        pass


# # 4. Load a URDF model (R2D2 robot as an example) :contentReference[oaicite:4]{index=4}
# start_pos = [0, 0, 1]
# start_orientation = p.getQuaternionFromEuler([0, 0, 0])
# box_id = p.loadURDF("r2d2.urdf", start_pos, start_orientation)

# # 5. Main simulation loop: step the simulation and sleep to slow it to real time
# for i in range(10000):
#     p.stepSimulation()  # advance the physics simulation by one time‐step :contentReference[oaicite:5]{index=5}
#     time.sleep(1.0 / 240.0)  # match the default PyBullet step frequency of 240 Hz

# # 6. Disconnect (when done) :contentReference[oaicite:6]{index=6}
# p.disconnect()
