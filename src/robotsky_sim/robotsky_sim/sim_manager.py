from sim import *

class SimManager:
    def __init__(self, simulatior_type = "none"):
        if simulatior_type == "none":
            self.sim = SimBase()
            raise ValueError("Simulator type must be specified.")
        elif simulatior_type == "pybullet":
            from sim.pybullet_sim import SimPyBullet
            self.sim = SimPyBullet()
        elif simulatior_type == "mujoco":   
            from sim.mujoco_sim import SimMuJoCo
            self.sim = SimMuJoCo()
        elif simulatior_type == "Genesis":
            from sim.genesis_sim import GenesisSim
            self.sim = GenesisSim()
        else:
            raise ValueError(f"Unknown simulator type: {simulatior_type}")

        self.sim.initialize()

    def run(self):
        self.sim.reset()
        
        while self.sim.is_running():
            state = self.sim.get_state()
            action = []  # Replace with actual action logic
            self.sim.set_action(action)

            # send state msg

            self.sim.step()
            self.sim.render()

    def receive_ros_action(self, action):
        pass

    def publish_ros_state(self, state):
        pass