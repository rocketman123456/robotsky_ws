from __future__ import annotations


class SimulationCfg:
    headless: bool = False
    simulatior_type: str = "none"
    timestep: float = 0.002
    framewidth: int = 1600
    frameheight: int = 900


class RobotCfg:
    robot_asset_path: str = ""
    robot_init_position: list = [0.0, 0.0, 0.5]
    robot_init_orientation: list = [0.0, 0.0, 0.0]


class SceneCfg:
    scene_asset_count: int = 0
    scene_asset_paths: list = []
    scene_init_positions: list = []
    scene_init_orientations: list = []


class ActionCfg:
    pass


class ObservationCfg:
    pass
