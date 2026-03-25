"""Abstract base class for all RL policies."""

from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional

import numpy as np
import torch


class BasePolicy(ABC):
    """
    Abstract base class for RL inference policies.

    Subclasses implement the network architecture and forward pass.
    The interface is kept minimal so any policy type (MLP, RNN, Transformer)
    can be dropped in without changing the controller node.
    """

    def __init__(
        self,
        obs_dim: int,
        action_dim: int,
        device: str = 'cpu',
    ) -> None:
        """Create the policy wrapper (network is built by subclasses)."""
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.device = torch.device(device)
        self._model: Optional[torch.nn.Module] = None

    # ------------------------------------------------------------------
    # Abstract interface
    # ------------------------------------------------------------------

    @abstractmethod
    def build(self) -> None:
        """Construct the network architecture."""

    @abstractmethod
    def forward(self, obs: np.ndarray) -> np.ndarray:
        """
        Run one inference step.

        :param obs: observation vector, shape (obs_dim,)
        :returns: action vector, shape (action_dim,)
        """

    # ------------------------------------------------------------------
    # Checkpoint helpers
    # ------------------------------------------------------------------

    def load(self, path: str | Path) -> None:
        """Load model weights from a .pt / .pth checkpoint."""
        if self._model is None:
            self.build()
        ckpt_path = str(path)

        # Some exported policies are TorchScript archives.
        # For these, torch.load() may fail on PyTorch >= 2.6.
        try:
            self._model = torch.jit.load(ckpt_path, map_location=self.device)
            self._model.to(self.device)
            self._model.eval()
            return
        except Exception:
            # Fall back to raw state-dict checkpoints.
            pass

        # PyTorch >= 2.6 defaults weights_only=True for torch.load.
        # Explicitly set it to False to accept state-dict checkpoints.
        state = torch.load(
            ckpt_path,
            map_location=self.device,
            weights_only=False,
        )

        # Support both raw state-dict and {'model': state_dict} formats
        if isinstance(state, dict) and 'model' in state:
            state = state['model']
        if isinstance(state, dict):
            self._model.load_state_dict(state)
        else:
            self._model = state
        self._model.to(self.device)
        self._model.eval()

    def save(self, path: str | Path) -> None:
        """Save model weights to a checkpoint file."""
        if self._model is None:
            raise RuntimeError('Policy has not been built yet.')
        torch.save(self._model.state_dict(), str(path))

    # ------------------------------------------------------------------
    # Convenience properties
    # ------------------------------------------------------------------

    @property
    def model(self) -> Optional[torch.nn.Module]:
        """Return the underlying torch module (or None if not built)."""
        return self._model

    def to(self, device: str) -> 'BasePolicy':
        """Move the underlying model to a torch device."""
        self.device = torch.device(device)
        if self._model is not None:
            self._model.to(self.device)
        return self

    def set_eval(self) -> 'BasePolicy':
        """Set the underlying model to eval mode."""
        if self._model is not None:
            self._model.eval()
        return self

    # Backward-compatible alias
    eval = set_eval  # noqa: A003

    def __repr__(self) -> str:
        """Return a concise string representation."""
        return (
            f"{self.__class__.__name__}("
            f"obs_dim={self.obs_dim}, "
            f"action_dim={self.action_dim}, "
            f"device={self.device})"
        )
