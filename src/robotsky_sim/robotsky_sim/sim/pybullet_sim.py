from .sim_base import SimBase
from .sim_config import *

import numpy as np
import pybullet as p
import pybullet_data
import time


class PybulletSim(SimBase):
    def __init__(self, sim_cfg: SimulationCfg):
        super().__init__(sim_cfg)
        if sim_cfg.headless:
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)

    def initialize(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("plane.urdf")

    def is_running(self):
        pass

    def reset(self):
        pass

    def step(self):
        pass

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
