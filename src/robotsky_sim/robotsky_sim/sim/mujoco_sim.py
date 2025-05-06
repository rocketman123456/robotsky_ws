from .sim_base import SimBase
from .sim_config import *

# import glfw
# # Tell GLFW “yes, decorate windows” before MuJoCo calls its window‑creation code:
# glfw.init()
# glfw.window_hint(glfw.DECORATED, glfw.TRUE)

import numpy as np
import threading
import time
import mujoco
import mujoco.viewer


class MujocoSim(SimBase):
    def __init__(self, sim_cfg: SimulationCfg):
        super().__init__(sim_cfg)
        self.sim_cfg = sim_cfg
        self.timestep = self.sim_cfg.timestep
        self.pause_flag = True
        self.ctrl = [0] * 16

    def initialize(self, robot_cfg: RobotCfg, scene_cfg: SceneCfg):
        self.robot_cfg = robot_cfg
        self.scene_cfg = scene_cfg

        self.model = mujoco.MjModel.from_xml_path(robot_cfg.robot_asset_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.timestep

        total_mass = sum(self.model.body_mass)
        print("total mass: ", total_mass)

        self.key_id = self.model.key("home").id

        self.viewer = None
        self.running = False

        # TODO : set robot default position
        # if hasattr(robot_cfg, 'initial_qpos'):
        #     for name, q in robot_cfg.initial_qpos.items():
        #         idx = self.model.joint(name).qposadr
        #         self.data.qpos[idx] = q

        # Reset the simulation to the initial keyframe.
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.key_id)
        mujoco.mj_forward(self.model, self.data)

        # Setup viewer if rendering is needed
        if not self.sim_cfg.headless:
            self.viewer = mujoco.viewer.launch_passive(
                self.model, 
                self.data, 
                show_left_ui=True, 
                show_right_ui=True, 
                key_callback=self._key_callback,
            )
            # Reset the simulation to the initial keyframe.
            mujoco.mj_resetDataKeyframe(self.model, self.data, self.key_id)
            # Initialize the camera view to track the base link.
            mujoco.mjv_defaultCamera(self.viewer.cam)
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.viewer.cam.trackbodyid = self.model.body("base_link").id
            self.viewer.sync()

        self.thread_view = threading.Thread(target=self._sync_loop)
        self.thread_view.start()

        self.running = True

    def finalize(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        self.thread_view.join()
        self.running = False

    def reset(self):
        # TODO
        pass

    def is_running(self):
        if self.viewer != None:
            self.running = self.running and self.viewer.is_running()
        return self.running

    def step(self):
        if self.viewer != None:
            with self.viewer.lock():
                if not self.pause_flag:
                    mujoco.mj_step(self.model, self.data)
                    # self.publish_msg()
                else:
                    mujoco.mj_forward(self.model, self.data)
        else:
            if not self.pause_flag:
                mujoco.mj_step(self.model, self.data)
                # self.publish_msg()
            else:
                mujoco.mj_forward(self.model, self.data)

    def render(self):
        # self.viewer.sync()
        pass

    def get_state(self):
        try:
            qp = self.data.qpos[-16:].copy()
            qv = self.data.qvel[-16:].copy()
            quat = self.data.sensor("BodyQuat").data.copy()
            gyro = self.data.sensor("BodyGyro").data.copy()
            acc = self.data.sensor("BodyAcc").data.copy()
            return qp, qv, quat, gyro, acc
        except:
            qp = []
            qv = []
            quat = []
            gyro = []
            acc = []
            return qp, qv, quat, gyro, acc

    def set_action(self, action):
        self._control_callback(action)

    def _sync_loop(self):
        if self.viewer != None:
            while self.viewer.is_running():
                self.viewer.sync()
                time.sleep(0.010)
        else:
            time.sleep(0.010)

    def _control_callback(self, action):
        for i in range(16):
            self.ctrl[i] = action[i]
        self.data.ctrl[:] = self.ctrl

    def _key_callback(self, keycode):
        try:
            with self.viewer.lock():
                if chr(keycode) == " ":
                    self.pause_flag = not self.pause_flag
        except:
            # self.viewer.unlock()
            print()
