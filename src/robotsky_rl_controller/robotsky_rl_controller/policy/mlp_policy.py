"""Simple MLP policy — designed to be extended in the future."""

from __future__ import annotations

from typing import List, Optional

import numpy as np
import torch
import torch.nn as nn

from .base_policy import BasePolicy


# ---------------------------------------------------------------------------
# Network building blocks
# ---------------------------------------------------------------------------

_ACTIVATIONS = {
    'elu': nn.ELU,
    'relu': nn.ReLU,
    'tanh': nn.Tanh,
    'selu': nn.SELU,
    'gelu': nn.GELU,
}


class MLP(nn.Module):
    """
    Generic fully-connected MLP.

    Architecture:
        obs_dim → [hidden[0] → act] → [hidden[1] → act] → … → action_dim
    """

    def __init__(
        self,
        input_dim: int,
        output_dim: int,
        hidden_dims: List[int],
        activation: str = 'elu',
        output_activation: Optional[str] = None,
    ) -> None:
        """Create an MLP network."""
        super().__init__()

        act_cls = _ACTIVATIONS.get(activation.lower())
        if act_cls is None:
            raise ValueError(
                f"Unknown activation '{activation}'. "
                f"Choose from: {list(_ACTIVATIONS)}"
            )

        layers: List[nn.Module] = []
        in_dim = input_dim
        for h in hidden_dims:
            layers.append(nn.Linear(in_dim, h))
            layers.append(act_cls())
            in_dim = h
        layers.append(nn.Linear(in_dim, output_dim))

        if output_activation is not None:
            out_act_cls = _ACTIVATIONS.get(output_activation.lower())
            if out_act_cls is None:
                raise ValueError(f"Unknown output activation '{output_activation}'.")
            layers.append(out_act_cls())

        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


# ---------------------------------------------------------------------------
# MLPPolicy
# ---------------------------------------------------------------------------

class MLPPolicy(BasePolicy):
    """
    MLP-based RL policy.

    Default architecture (512 → 256 → 128) can be changed via
    ``hidden_dims`` at construction time. Everything else stays the same —
    just subclass and override ``build()`` for a different architecture.

    Observation layout (57 dims total), matching `robotsky_wq_env.py` actor_obs:
        [0:3]   ang_vel (rad/s)
        [3:6]   projected_gravity (unit vector in body frame)
        [6:9]   cmd (vx, vy, yaw_rate)
        [9:25]  joint_pos deviation (wheel positions are zeroed)
        [25:41] joint_vel deviation (wheel/leg scaled separately by node)
        [41:57] previous action

    Action layout (16 dims):
        joint position / velocity / torque targets,
        ordered: RF_Roll, RF_Hip, RF_Knee, RF_Wheel,
                 LF_Roll, LF_Hip, LF_Knee, LF_Wheel,
                 RB_Roll, RB_Hip, RB_Knee, RB_Wheel,
                 LB_Roll, LB_Hip, LB_Knee, LB_Wheel
    """

    def __init__(
        self,
        obs_dim: int = 57,
        action_dim: int = 16,
        hidden_dims: Optional[List[int]] = None,
        activation: str = 'elu',
        action_scale: float = 1.0,
        device: str = 'cpu',
    ) -> None:
        """Create an MLP policy wrapper."""
        super().__init__(obs_dim=obs_dim, action_dim=action_dim, device=device)
        self.hidden_dims = hidden_dims or [512, 256, 128]
        self.activation = activation
        self.action_scale = action_scale

        # Build immediately so the policy is ready to use / load into
        self.build()

    def build(self) -> None:
        """Construct the MLP. Can be called again to re-initialise weights."""
        self._model = MLP(
            input_dim=self.obs_dim,
            output_dim=self.action_dim,
            hidden_dims=self.hidden_dims,
            activation=self.activation,
        ).to(self.device)
        self._model.eval()

    @torch.no_grad()
    def forward(self, obs: np.ndarray) -> np.ndarray:
        """
        Run one inference step.

        :param obs: 1-D numpy array, shape (obs_dim,)
        :returns:   1-D numpy array, shape (action_dim,)
        """
        x = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        if x.ndim == 1:
            x = x.unsqueeze(0)          # (1, obs_dim)
        action = self._model(x)          # (1, action_dim)
        return (action.squeeze(0) * self.action_scale).cpu().numpy()
