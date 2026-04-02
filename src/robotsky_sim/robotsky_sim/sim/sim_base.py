from abc import ABC, abstractmethod

from .sim_config import RobotCfg, SceneCfg, SimulationCfg


class SimBase(ABC):
    def __init__(self, sim_cfg: SimulationCfg):
        self.sim_cfg = sim_cfg
        self.pause_flag = False

    @abstractmethod
    def initialize(self, robot_cfg: RobotCfg, scene_cfg: SceneCfg):
        """Initialize the simulator backend."""

    @abstractmethod
    def finalize(self):
        """Release simulator resources."""

    @abstractmethod
    def reset(self):
        """Reset the simulator to its initial state."""

    @abstractmethod
    def is_running(self):
        """Return whether the simulator should keep running."""

    @abstractmethod
    def step(self):
        """Advance the simulator by one step."""

    @abstractmethod
    def render(self):
        """Render the simulator, if the backend supports it."""

    @abstractmethod
    def get_state(self):
        raise NotImplementedError("This method should be overridden by subclasses")

    @abstractmethod
    def set_action(self, action):
        raise NotImplementedError("This method should be overridden by subclasses")

    def receive_ros_action(self, msg) -> None:
        """Optional hook for backends that consume raw ROS motor command messages."""
        del msg
