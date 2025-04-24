from .sim_base import SimBase
from .sim_config import *

import numpy as np


class GenesisSim(SimBase):
    def __init__(self, sim_cfg: SimulationCfg):
        pass

    def initialize(self):
        pass

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
