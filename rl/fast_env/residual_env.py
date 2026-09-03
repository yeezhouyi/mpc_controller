"""Residual RL environment for joint-space tracking (C4).

MDP (RRBot plan weeks 5-9):
* state/obs : [e_q(2), e_qd(2), q_ref_next(2), prev_tau(2)] (runtime-only),
* action    : normalised residual delta_tau in [-1,1]^2,
* transfer  : u_raw = u_mpc + alpha*scale(delta_tau), clamped to effort and
              rate limits, then RRBotJointPlant.step (with optional
              inertia/damping/delay/noise randomisation),
* reward    : tracking error + progress + effort smoothness - constraint
              violation (see reward()).
* alpha=0   => exactly the pure-MPC baseline (equivalence test, C4 gate).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from rl.fast_env.joint_arm import NX, NU, ReferenceProfile, RRBotJointPlant

OBS_DIM = 8
DELTA_TAU_MAX = 1.0  # Nm per step at full action


@dataclass
class MpcBaselineParams:
    """Unconstrained condensed-MPC baseline parameters (double integrator)."""
    N: int = 10               # horizon steps
    Ts: float = 0.02
    Q_diag: tuple = (60.0, 60.0, 1.0, 1.0)   # [q1,q2,qd1,qd2]
    R_diag: tuple = (0.1, 0.1)               # effort penalty
    QF_diag: tuple = (120.0, 120.0, 2.0, 2.0)


@dataclass
class RewardWeights:
    wq: float = 1.0
    wqd: float = 0.2
    wp: float = 2.0
    wu: float = 0.05
    wc: float = 5.0
    complete_bonus: float = 5.0


class JointResidualEnv:
    """Joint-space residual env (gym-style but dependency-free)."""

    def __init__(
        self,
        profile: Optional[ReferenceProfile] = None,
        Ts: float = 0.02,
        tau_max: float = 5.0,
        alpha_residual: float = 1.0,
        difficulty: float = 0.0,
        reward_w: Optional[RewardWeights] = None,
        baseline: Optional[MpcBaselineParams] = None,
    ) -> None:
        self.profile = profile or ReferenceProfile()
        self.Ts = float(Ts)
        self.tau_max = float(tau_max)
        self.alpha = float(alpha_residual)
        self.difficulty = float(difficulty)
        self.rw = reward_w or RewardWeights()
        self.bp = baseline or MpcBaselineParams(Ts=Ts)
        self.rng = np.random.default_rng(0)
        self.plant: RRBotJointPlant = RRBotJointPlant(Ts=Ts)
        self.t = 0.0
        self.step_count = 0
        self.prev_tau = np.zeros(NU)
        self.prev_err = np.zeros(2)
        self.info: dict = {}

    # ---- helpers -----------------------------------------------------------
    def _sample_plant(self) -> RRBotJointPlant:
        d = self.difficulty
        inertia = float(self.rng.uniform(1.0, 1.0 + 0.3 * d))
        damping = float(self.rng.uniform(0.0, 0.25 * d))
        delay = int(self.rng.integers(0, 1 + int(2 * d)))
        return RRBotJointPlant(Ts=self.Ts, inertia=inertia, damping=damping,
                               delay_steps=delay, seed=int(self.rng.integers(0, 2**31 - 1)))

    def _baseline_command(self, x: np.ndarray, t: float) -> np.ndarray:
        """Unconstrained condensed MPC first action (tau_0)."""
        p = self.bp
        Ts = p.Ts
        N = p.N
        q_ref = self.profile.q(t)
        A = np.eye(4)
        A[0, 2] = Ts
        A[1, 3] = Ts
        B = np.zeros((4, 2))
        B[2, 0] = Ts
        B[3, 1] = Ts
        F = np.zeros((4 * N, 4))
        G = np.zeros((4 * N, 2 * N))
        F[0:4] = A
        G[0:4, 0:2] = B
        for k in range(1, N):
            F[4 * k:4 * k + 4] = A @ F[4 * k - 4:4 * k]
            G[4 * k:4 * k + 4, :2 * k] = A @ G[4 * k - 4:4 * k, :2 * k]
            G[4 * k:4 * k + 4, 2 * k:2 * k + 2] = B
        Q = np.diag(p.Q_diag)
        QF = np.diag(p.QF_diag)
        R = np.diag(p.R_diag)
        Qbar = np.zeros((4 * N, 4 * N))
        for k in range(N):
            Qbar[4 * k:4 * k + 4, 4 * k:4 * k + 4] = QF if k == N - 1 else Q
        Rbar = np.zeros((2 * N, 2 * N))
        for k in range(N):
            Rbar[2 * k:2 * k + 2, 2 * k:2 * k + 2] = R
        Xr = np.zeros(4 * N)
        for k in range(N):
            tk = t + (k + 1) * Ts
            qr = self.profile.q(tk)
            qdr = self.profile.qd(tk)
            Xr[4 * k:4 * k + 4] = np.array([qr[0], qr[1], qdr[0], qdr[1]])
        x_free = F @ x - Xr
        H = G.T @ Qbar @ G + Rbar
        q_vec = G.T @ Qbar @ x_free
        u_seq = np.linalg.solve(H, -q_vec)
        return np.clip(u_seq[0:2], -self.tau_max, self.tau_max)

    # ---- MDP ---------------------------------------------------------------
    def reset(self, seed: Optional[int] = None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.plant = self._sample_plant()
        e0 = self.rng.uniform(0.0, 0.25 * self.difficulty, size=2)
        self.plant.reset(x0=np.array([self.profile.q(0.0)[0] + e0[0],
                                      self.profile.q(0.0)[1] + e0[1],
                                      0.0, 0.0]))
        self.t = 0.0
        self.step_count = 0
        self.prev_tau = np.zeros(NU)
        return self._observe(), {}

    def _observe(self) -> np.ndarray:
        x = self.plant.observe()
        q_ref = self.profile.q(self.t)
        qd_ref = self.profile.qd(self.t)
        e_q = x[0:2] - q_ref
        e_qd = x[2:4] - qd_ref
        q_next = self.profile.q(min(self.t + self.Ts, self.profile.duration_s))
        return np.array([
            e_q[0], e_q[1], e_qd[0], e_qd[1],
            q_next[0], q_next[1], self.prev_tau[0], self.prev_tau[1],
        ])

    def reward(self, e_q: np.ndarray, e_qd: np.ndarray, u_raw: np.ndarray,
               u: np.ndarray, progress: float, done: bool, reason: str) -> float:
        r = -self.rw.wq * float(np.sum(e_q ** 2))
        r -= self.rw.wqd * float(np.sum(e_qd ** 2))
        r += self.rw.wp * float(progress)
        r -= self.rw.wu * float(np.sum((u - self.prev_tau) ** 2))
        viol = float(np.max(np.maximum(np.abs(u_raw) - self.tau_max, 0.0)))
        r -= self.rw.wc * viol
        if reason == "completed":
            r += self.rw.complete_bonus
        return float(r)

    def step(self, action: np.ndarray):
        """action: normalised residual in [-1,1]^2."""
        action = np.clip(np.asarray(action, dtype=float).ravel(), -1.0, 1.0)
        self.step_count += 1
        x = self.plant.observe()

        u_mpc = self._baseline_command(x, self.t)
        delta = action * DELTA_TAU_MAX
        u_raw = u_mpc + self.alpha * delta
        u = np.clip(u_raw, -self.tau_max, self.tau_max)

        x_new = self.plant.step(u)
        self.t += self.Ts

        q_ref = self.profile.q(self.t)
        qd_ref = self.profile.qd(self.t)
        e_q = x_new[0:2] - q_ref
        e_qd = x_new[2:4] - qd_ref

        prev_arc = getattr(self, "_arc_prev", self.t)
        progress = max(0.0, (self.t - prev_arc) / self.profile.duration_s)

        done, reason = self._check_done(e_q)
        rew = self.reward(e_q, e_qd, u_raw, u, progress, done, reason)
        self.prev_tau = u.copy()
        self._arc_prev = self.t
        self.info = {"reason": reason, "t": self.t}
        return self._observe(), rew, bool(done), self.info

    def _check_done(self, e_q: np.ndarray):
        if self.t >= self.profile.duration_s:
            return True, "completed"
        if float(np.max(np.abs(e_q))) > 1.5:
            return True, "out_of_track"
        if self.step_count >= int(self.profile.duration_s / self.Ts) + 200:
            return True, "max_steps"
        return False, ""
