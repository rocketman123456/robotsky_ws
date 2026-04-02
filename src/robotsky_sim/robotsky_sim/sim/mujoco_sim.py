from .sim_base import SimBase
from .sim_config import RobotCfg, SceneCfg, SimulationCfg

import logging
import numpy as np
import threading
import time
import mujoco
import mujoco.viewer
from robotsky_interface.msg import MotorCmds

LOGGER = logging.getLogger(__name__)


class MujocoSim(SimBase):
    def __init__(self, sim_cfg: SimulationCfg):
        super().__init__(sim_cfg)
        self.timestep = self.sim_cfg.timestep
        self.pause_flag = True
        self.ctrl = np.zeros(16, dtype=np.float64)
        self.viewer = None
        self.thread_view: threading.Thread | None = None
        self.running = False

    def initialize(self, robot_cfg: RobotCfg, scene_cfg: SceneCfg):
        self.robot_cfg = robot_cfg
        self.scene_cfg = scene_cfg

        self.model = mujoco.MjModel.from_xml_path(robot_cfg.robot_asset_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.timestep

        total_mass = sum(self.model.body_mass)
        LOGGER.info("MuJoCo model loaded with total mass %.3f kg", total_mass)

        self.key_id = self.model.key("home").id

        # TODO : set robot default position
        # if hasattr(robot_cfg, 'initial_qpos'):
        #     for name, q in robot_cfg.initial_qpos.items():
        #         idx = self.model.joint(name).qposadr
        #         self.data.qpos[idx] = q

        # Reset the simulation to the initial keyframe.
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.key_id)
        mujoco.mj_forward(self.model, self.data)

        self.running = True

        # Setup viewer if rendering is needed
        if not self.sim_cfg.headless:
            self.viewer = mujoco.viewer.launch_passive(
                self.model, 
                self.data, 
                show_left_ui=False, 
                show_right_ui=False, 
                key_callback=self._key_callback,
            )
            # Reset the simulation to the initial keyframe.
            mujoco.mj_resetDataKeyframe(self.model, self.data, self.key_id)
            # Initialize the camera view to track the base link.
            mujoco.mjv_defaultCamera(self.viewer.cam)
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.viewer.cam.trackbodyid = self.model.body("base_link").id
            self.viewer.sync()

            self.thread_view = threading.Thread(target=self._sync_loop, daemon=True)
            self.thread_view.start()

    def finalize(self):
        self.running = False
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        if self.thread_view is not None:
            self.thread_view.join(timeout=1.0)
            self.thread_view = None

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.key_id)
        mujoco.mj_forward(self.model, self.data)

    def is_running(self):
        if self.viewer is not None:
            self.running = self.running and self.viewer.is_running()
        return self.running

    def step(self):
        if self.viewer is not None:
            with self.viewer.lock():
                if not self.pause_flag:
                    mujoco.mj_step(self.model, self.data)
                else:
                    mujoco.mj_forward(self.model, self.data)
        else:
            if not self.pause_flag:
                mujoco.mj_step(self.model, self.data)
            else:
                mujoco.mj_forward(self.model, self.data)

    def render(self):
        # self.viewer.sync()
        pass

    def get_state(self):
        try:
            qp   = self.data.qpos[-16:].copy()
            qv   = self.data.qvel[-16:].copy()
            quat = self.data.sensor("BodyQuat").data.copy()
            gyro = self.data.sensor("BodyGyro").data.copy()
            acc  = self.data.sensor("BodyAcc").data.copy()
            return qp, qv, quat, gyro, acc
        except Exception as e:
            LOGGER.warning("Failed to read MuJoCo state: %s", e)
            return (
                np.zeros(16, dtype=np.float64),
                np.zeros(16, dtype=np.float64),
                np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
                np.zeros(3, dtype=np.float64),
                np.zeros(3, dtype=np.float64),
            )

    def receive_ros_action(self, msg: MotorCmds):
        if len(msg.cmds) < 16:
            return
        action = [msg.cmds[i].pos for i in range(16)]
        action[3] = msg.cmds[3].vel
        action[7] = msg.cmds[7].vel
        action[11] = msg.cmds[11].vel
        action[15] = msg.cmds[15].vel
        self.set_action(action)

    def set_action(self, action):
        action_array = np.asarray(action, dtype=np.float64).reshape(-1)
        if action_array.shape[0] != 16:
            raise ValueError(f"Expected 16 control values, got {action_array.shape[0]}")
        self._control_callback(action_array)

    def _sync_loop(self):
        while self.running and self.viewer is not None and self.viewer.is_running():
            try:
                self.viewer.sync()
            except Exception as e:
                LOGGER.warning("MuJoCo viewer sync failed: %s", e)
                break
            time.sleep(0.010)

    def _control_callback(self, action):
        self.ctrl[:] = action
        self.data.ctrl[:] = self.ctrl

    def _key_callback(self, keycode):
        try:
            with self.viewer.lock():
                if chr(keycode) == " ":
                    self.pause_flag = not self.pause_flag
        except Exception as e:
            LOGGER.warning("MuJoCo key callback failed: %s", e)
