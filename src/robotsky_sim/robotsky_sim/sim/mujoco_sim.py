from .sim_base import SimBase
from .sim_config import *

import numpy as np
import mujoco


class MujocoSim(SimBase):
    def __init__(self, sim_cfg: SimulationCfg):
        pass

    def initialize(self, robot_cfg: RobotCfg, scene_cfg: SceneCfg):
        pass

    def finalize(self):
        pass

    def reset(self):
        pass

    def is_running(self):
        pass

    def step(self):
        pass

    def render(self):
        pass

    def get_state(self):
        pass

    def set_action(self, action):
        pass
