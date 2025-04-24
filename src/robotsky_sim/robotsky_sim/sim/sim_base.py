from .sim_config import *

import numpy as np
from abc import ABC, abstractmethod


class SimBase:
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
        raise NotImplementedError("This method should be overridden by subclasses")

    def set_action(self, action):
        raise NotImplementedError("This method should be overridden by subclasses")
