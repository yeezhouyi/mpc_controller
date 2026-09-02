"""Gymnasium wrapper for the residual tracking env (SB3/PPO training).

Adds the standard ``gym.Env`` protocol (spaces, 5-tuple step, seeding) around
:class:`ResidualTrackingEnv` and normalises observations with the default
stats.  Requires ``gymnasium`` (available in the RL venv); the core env stays
dependency-free so contract tests run anywhere.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

try:  # pragma: no cover - env-specific import
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # pragma: no cover
    gym = None
    spaces = None

from mpc_rl_env.envs.fast_tracking_env import ResidualTrackingEnv
from mpc_rl_env.envs.observation_builder import N_OBS, default_stats, normalize


class GymResidualTrackingEnv(gym.Env):  # type: ignore[misc]
    metadata = {"render_modes": []}

    def __init__(self, track: str = "circle", **env_kwargs) -> None:
        if gym is None:
            raise RuntimeError("gymnasium is required for the gym wrapper")
        super().__init__()
        self.track = track
        self.inner = ResidualTrackingEnv.for_track(track, **env_kwargs)
        self._stats = default_stats()
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(N_OBS,), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)

    def _obs(self, obs: np.ndarray) -> np.ndarray:
        return normalize(obs, self._stats).astype(np.float32)

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        obs, info = self.inner.reset(seed=seed)
        return self._obs(obs), info

    def step(self, action):
        obs, reward, terminated, info = self.inner.step(action)
        return self._obs(obs), float(reward), bool(terminated), False, info
