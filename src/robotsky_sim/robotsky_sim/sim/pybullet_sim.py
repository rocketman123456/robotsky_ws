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
            self.physicsClient = pybullet.connect(pybullet.DIRECT)
        else:
            self.physicsClient = pybullet.connect(pybullet.GUI)

    def initialize(self, robot_cfg: RobotCfg, scene_cfg: SceneCfg):
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.81)
        pybullet.setTimeStep(1./500.)  # finer integration
        pybullet.setPhysicsEngineParameter(numSolverIterations=100)  # more solver loops

        # Load the robot model
        self.robot_id = pybullet.loadURDF(robot_cfg.robot_asset_path)
        self.start_pos = robot_cfg.robot_init_position
        self.start_orientation = pybullet.getQuaternionFromEuler(robot_cfg.robot_init_orientation)
        pybullet.resetBasePositionAndOrientation(self.robot_id, self.start_pos, self.start_orientation)

        self.num_joints = pybullet.getNumJoints(self.robot_id) # count joints
        for i in range(self.num_joints):
            info = pybullet.getJointInfo(self.robot_id, i)                            
            joint_name = info[1].decode('utf-8')                          
            joint_type = info[2]       # e.g. p.JOINT_REVOLUTE, p.JOINT_PRISMATIC
            lower_limit, upper_limit = info[8], info[9]                    
            print(f"Joint {i}: {joint_name}, type={joint_type}, limits=({lower_limit},{upper_limit})")

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
        # position control
        for i in range(self.num_joints):
            # print(i)
            pybullet.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=i,                       # target joint index
                controlMode=pybullet.POSITION_CONTROL,     # position control mode :contentReference[oaicite:5]{index=5}
                targetPosition=action[i],                 # desired joint angle in radians :contentReference[oaicite:6]{index=6}
                targetVelocity=0,
                positionGain=1.0,
                velocityGain=0.5,
                maxVelocity=20.0,                    # velocity limit (optional) :contentReference[oaicite:8]{index=8}
                force=15                            # maximum torque/force to apply :contentReference[oaicite:9]{index=9}
            )


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
