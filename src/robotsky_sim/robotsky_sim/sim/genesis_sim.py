from .sim_base import SimBase
from .sim_config import *

import numpy as np
import genesis as gs

# import os

# # Force GLX backend for both MuJoCo and PyOpenGL
# os.environ["MUJOCO_GL"]         = "glx"
# os.environ["PYOPENGL_PLATFORM"] = "glx"

class GenesisSim(SimBase):
    def __init__(self, sim_cfg: SimulationCfg):
        super().__init__(sim_cfg)
        self.sim_cfg = sim_cfg
        self.timestep = self.sim_cfg.timestep
        self.pause_flag = True
        self.ctrl = [0] * 16

    def initialize(self, robot_cfg: RobotCfg, scene_cfg: SceneCfg):
        self.robot_cfg = robot_cfg
        self.scene_cfg = scene_cfg

        gs.init(backend=gs.cpu)
        # gs.init(
        #     seed                = None,
        #     precision           = '32',
        #     debug               = False,
        #     eps                 = 1e-12,
        #     logging_level       = None,
        #     backend             = gs.cpu, # gpu
        #     theme               = 'dark',
        #     logger_verbose_time = False
        # )

        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(
                dt=0.002,
                gravity=(0, 0, -9.81),
            ),
            show_viewer=True,
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(3.5, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=40,
            ),
        )
        self.plane = self.scene.add_entity(gs.morphs.Plane())
        # self.robot = self.scene.add_entity(
        #     # gs.morphs.MJCF(file=robot_cfg.robot_asset_path),
        #     gs.morphs.URDF(file=robot_cfg.robot_asset_path),
        # )
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file  = robot_cfg.robot_asset_path,
                pos   = (0, 0, 0),
                euler = (0, 0, 90), # we follow scipy's extrinsic x-y-z rotation convention, in degrees,
                # quat  = (1.0, 0.0, 0.0, 0.0), # we use w-x-y-z convention for quaternions,
                scale = 1.0,
            ),
        )
        self.scene.build()

    def finalize(self):
        pass

    def reset(self):
        pass

    def is_running(self):
        return True

    def step(self):
        self.scene.step()

    def render(self):
        pass

    def get_state(self):
        qp = []
        qv = []
        quat = []
        gyro = []
        acc = []
        return qp, qv, quat, gyro, acc

    def set_action(self, action):
        pass
