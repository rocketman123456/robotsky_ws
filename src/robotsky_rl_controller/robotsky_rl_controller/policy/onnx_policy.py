"""ONNX Runtime policy for RL inference (no PyTorch at runtime)."""

from __future__ import annotations

from pathlib import Path
from typing import Any, List, Optional, Sequence, Union

import numpy as np


class OnnxPolicy:
    """
    Loads a single-input, single-output ONNX model and runs inference on CPU.

    Expects observation as a flat float32 vector of length ``obs_dim``; feeds
    shape ``(1, obs_dim)`` unless the model declares a different fixed batch layout.
    """

    def __init__(
        self,
        obs_dim: int,
        action_dim: int,
        action_scale: float = 1.0,
        providers: Optional[Sequence[str]] = None,
    ) -> None:
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.action_scale = float(action_scale)
        self._providers: Optional[List[str]] = list(providers) if providers is not None else None
        self._session: Optional[Any] = None
        self._input_name: str = ""

    def load(self, path: Union[str, Path]) -> None:
        """Build an InferenceSession from ``path``."""
        import onnxruntime as ort

        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        prov = list(self._providers) if self._providers is not None else ["CPUExecutionProvider"]
        self._session = ort.InferenceSession(str(path), sess_options=so, providers=prov)

        inputs = self._session.get_inputs()
        if len(inputs) != 1:
            raise ValueError(
                f"ONNX model must have exactly 1 input, got {len(inputs)}: "
                f"{[i.name for i in inputs]}"
            )
        inp = inputs[0]
        self._input_name = inp.name
        shape = inp.shape
        # Resolve symbolic batch dim to 1 for size checks
        fixed_tail = [d for d in shape[1:] if isinstance(d, int)]
        if fixed_tail:
            expected = int(np.prod(fixed_tail))
            if expected != self.obs_dim:
                raise ValueError(
                    f"ONNX input shape {shape} implies size {expected}, "
                    f"but obs_dim={self.obs_dim}"
                )

    def forward(self, obs: np.ndarray) -> np.ndarray:
        if self._session is None:
            raise RuntimeError("OnnxPolicy.load() was not called.")

        x = np.ascontiguousarray(np.asarray(obs, dtype=np.float32).reshape(1, -1))
        if x.shape[1] != self.obs_dim:
            raise ValueError(f"obs shape[1]={x.shape[1]} != obs_dim={self.obs_dim}")

        out = self._session.run(None, {self._input_name: x})
        if not out:
            raise RuntimeError("ONNX model returned no outputs.")
        action = np.asarray(out[0], dtype=np.float32).reshape(-1)
        if action.shape[0] != self.action_dim:
            raise ValueError(
                f"ONNX output length {action.shape[0]} != action_dim={self.action_dim}"
            )
        return (action * np.float32(self.action_scale))
