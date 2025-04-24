from sim import *
import time


class SimManager:
    def __init__(self, sim_cfg: SimulationCfg, simulatior_type="none"):
        if simulatior_type == "none":
            self.sim = SimBase(sim_cfg=sim_cfg)
            raise ValueError("Simulator type must be specified.")
        elif simulatior_type == "pybullet":
            from sim.pybullet_sim import PybulletSim

            self.sim = PybulletSim(sim_cfg=sim_cfg)
        elif simulatior_type == "mujoco":
            from sim.mujoco_sim import MujocoSim

            self.sim = MujocoSim(sim_cfg=sim_cfg)
        elif simulatior_type == "Genesis":
            from sim.genesis_sim import GenesisSim

            self.sim = GenesisSim(sim_cfg=sim_cfg)
        else:
            raise ValueError(f"Unknown simulator type: {simulatior_type}")

    def run(self):
        self.sim.initialize()
        # self.sim.reset()

        while self.sim.is_running():
            state = self.sim.get_state()
            action = []  # Replace with actual action logic
            self.sim.set_action(action)

            # send state msg

            self.sim.step()
            self.sim.render()

            # TODO : sleep

        self.sim.finalize()

    def receive_ros_action(self, action):
        pass

    def publish_ros_state(self, state):
        pass
