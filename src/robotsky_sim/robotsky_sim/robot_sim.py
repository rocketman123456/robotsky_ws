from ament_index_python.packages import get_package_share_directory
import os

from .sim import *
from .sim_manager import SimManager

def main():
    # Create an instance of the RobotSkySim class
    sim_cfg = SimulationCfg()
    sim_cfg.simulatior_type = "pybullet"

    pkg_share = get_package_share_directory('robotsky_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robotsky_wq.urdf')
    print(f"URDF is at: {urdf_path}")
    robot_cfg = RobotCfg()
    robot_cfg.robot_asset_path = urdf_path

    scene_cfg = SceneCfg()

    sim = SimManager(sim_cfg = sim_cfg, robot_cfg = robot_cfg, scene_cfg = scene_cfg)

    # add action subscriber
    # add state publisher

    # Run the simulation
    while sim.is_running():
        # state = sim.get_state()
        # sim.set_action([])

        # send state msg

        sim.step()
        # sim.render()

if __name__ == "__main__":
    main()