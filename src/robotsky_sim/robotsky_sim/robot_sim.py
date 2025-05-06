from ament_index_python.packages import get_package_share_directory
import os

from .sim import *
from .sim_manager import SimManager

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState, Imu
from robotsky_interface.msg import MotorCmds, MotorStates, MotorCmd, MotorState

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the RobotSkySim class
    sim_cfg = SimulationCfg()
    # sim_cfg.simulatior_type = "pybullet"
    sim_cfg.simulatior_type = "mujoco"
    # sim_cfg.simulatior_type = "genesis"

    pkg_share = get_package_share_directory('robotsky_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robotsky_wq.urdf')
    mjcf_path = os.path.join(pkg_share, 'mjcf', 'robotsky_wq.xml')
    print(f"URDF is at: {urdf_path}")
    print(f"MJCF is at: {mjcf_path}")
    robot_cfg = RobotCfg()
    robot_cfg.robot_asset_path = mjcf_path
    # robot_cfg.robot_asset_path = urdf_path

    scene_cfg = SceneCfg()

    sim = SimManager(sim_cfg = sim_cfg, robot_cfg = robot_cfg, scene_cfg = scene_cfg)

    # add action subscriber
    # add state publisher

    # Run the simulation
    try:
        rclpy.spin(sim)
    except:
            print()

    sim.sim.finalize()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()