from __future__ import annotations


class SimulationCfg:
    headless: bool = False


class RobotCfg:
    robot_asset_path: str = ""
    robot_init_position: list = []
    robot_init_orientation: list = []


class SceneCfg:
    scene_asset_count: int = 0
    scene_asset_paths: list = []
    scene_init_positions: list = []
    scene_init_orientations: list = []


class ActionCfg:
    pass


class ObservationCfg:
    pass
