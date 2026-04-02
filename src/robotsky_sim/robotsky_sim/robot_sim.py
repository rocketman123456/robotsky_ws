import os

import rclpy
from ament_index_python.packages import get_package_share_directory

from .sim import RobotCfg, SceneCfg, SimulationCfg
from .sim_manager import SimManager


def _resolve_robot_asset_path(simulator_type: str) -> str:
    pkg_share = get_package_share_directory("robotsky_description")
    asset_dir = "mjcf" if simulator_type == "mujoco" else "urdf"
    asset_name = "robotsky_wq.xml" if simulator_type == "mujoco" else "robotsky_wq.urdf"
    return os.path.join(pkg_share, asset_dir, asset_name)

def main(args=None):
    rclpy.init(args=args)

    sim_cfg = SimulationCfg()
    sim_cfg.simulator_type = os.environ.get("ROBOTSKY_SIMULATOR", sim_cfg.simulator_type)

    robot_cfg = RobotCfg()
    robot_cfg.robot_asset_path = _resolve_robot_asset_path(sim_cfg.simulator_type)
    scene_cfg = SceneCfg()

    sim = SimManager(sim_cfg=sim_cfg, robot_cfg=robot_cfg, scene_cfg=scene_cfg)
    sim.get_logger().info(
        f"Robot simulator started with backend '{sim_cfg.simulator_type}' using asset '{robot_cfg.robot_asset_path}'"
    )

    action = [
        0.0, -0.5, 1.0, 0.0, #
        0.0, -0.5, 1.0, 0.0, #
        0.0, 0.5, -1.0, 0.0, #
        0.0, 0.5, -1.0, 0.0, #
    ]
    sim.set_action(action)

    try:
        rclpy.spin(sim)
    except KeyboardInterrupt:
        pass
    finally:
        sim.sim.finalize()
        sim.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
