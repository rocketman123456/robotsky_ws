from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class SimulationCfg:
    headless: bool = False
    simulatior_type: str = "mujoco"
    timestep: float = 0.002
    framewidth: int = 1600
    frameheight: int = 900


@dataclass
class RobotCfg:
    robot_asset_path: str = ""
    robot_init_position: list = field(default_factory=lambda: [0.0, 0.0, 0.5])
    robot_init_orientation: list = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class SceneCfg:
    scene_asset_count: int = 0
    scene_asset_paths: list = field(default_factory=list)
    scene_init_positions: list = field(default_factory=list)
    scene_init_orientations: list = field(default_factory=list)


@dataclass
class ActionCfg:
    pass


@dataclass
class ObservationCfg:
    pass
