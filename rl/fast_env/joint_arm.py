"""Joint-space fast env for the RRBot residual-RL line (C4).

Self-contained numpy reference: two-DOF planar arm, joint states
``x = [q1, q2, qd1, qd2]``, inputs ``u = [tau1, tau2]`` (effort), control
period ``Ts``.  The nominal plant is a per-joint double integrator (unit
effective inertia) -- exactly the model family the production MPC uses
(config A/B double integrator) -- with optional domain randomisation
(inertia scale, damping, command delay, measurement noise) for the
residual-RL scenarios (RRBot plan weeks 5-9).

The MDP baseline is a *condensed unconstrained MPC* (finite-horizon LQR
solution) with effort/rate clamp; the policy action is a residual
``delta_tau`` added with gain ``alpha``.  Constrained-OSQP baseline parity
with the production controller is a later alignment step (documented).
"""
from __future__ import annotations

import numpy as np

NX = 4          # [q1, q2, qd1, qd2]
NU = 2          # [tau1, tau2]


class ReferenceProfile:
    """Deterministic joint-space reference (synchronised sines)."""

    def __init__(self, amplitude: float = 0.8, freq: float = 0.3,
                 offset: float = 0.0, duration_s: float = 10.0) -> None:
        self.amplitude = float(amplitude)
        self.freq = float(freq)
        self.offset = float(offset)
        self.duration_s = float(duration_s)

    def q(self, t: float) -> np.ndarray:
        """q_ref(t) for both joints (phase-shifted sine pair)."""
        return self.offset + self.amplitude * np.array([
            np.sin(2 * np.pi * self.freq * t),
            np.sin(2 * np.pi * self.freq * t + 1.3),
        ])

    def qd(self, t: float) -> np.ndarray:
        w = 2 * np.pi * self.freq
        return self.amplitude * w * np.array([
            np.cos(2 * np.pi * self.freq * t),
            np.cos(2 * np.pi * self.freq * t + 1.3),
        ])

    def qdd(self, t: float) -> np.ndarray:
        w = 2 * np.pi * self.freq
        return -self.amplitude * w * w * np.array([
            np.sin(2 * np.pi * self.freq * t),
            np.sin(2 * np.pi * self.freq * t + 1.3),
        ])


class RRBotJointPlant:
    """Two-joint arm plant (double integrator + optional randomisation)."""

    def __init__(self, Ts: float = 0.02, inertia: float = 1.0, damping: float = 0.0,
                 delay_steps: int = 0, seed: int = 0) -> None:
        self.Ts = float(Ts)
        self.inertia = float(inertia)
        self.damping = float(damping)
        self.delay_steps = int(delay_steps)
        self.rng = np.random.default_rng(seed)
        self.x = np.zeros(NX)
        self._cmd_buf: list = []

    def reset(self, x0: np.ndarray | None = None) -> np.ndarray:
        self.x = np.zeros(NX) if x0 is None else np.asarray(x0, dtype=float).copy()
        self._cmd_buf = []
        return self.x.copy()

    def step(self, u: np.ndarray) -> np.ndarray:
        """Semi-implicit Euler: qd += Ts*(tau - damping*qd)/inertia; q += Ts*qd."""
        self._cmd_buf.append(np.asarray(u, dtype=float))
        applied = self._cmd_buf.pop(0) if len(self._cmd_buf) > self.delay_steps else u
        x = self.x
        qd_new = x[2:] + self.Ts * (applied - self.damping * x[2:]) / self.inertia
        q_new = x[:2] + self.Ts * qd_new
        self.x = np.concatenate([q_new, qd_new])
        return self.x.copy()

    def observe(self) -> np.ndarray:
        return self.x.copy()
