"""Pure (numpy-only) joint model fitting for RRBot system identification.

Discrete per-joint model fitted from effort excitation data:

    q[k+1]   = q[k] + Ts * qd[k]                     (kinematic identity)
    qd[k+1]  = a * qd[k] + b * tau[k-d] + c          (ARX in velocity)

Continuous parameters (effective inertia I, damping D, static bias tau0):
    I    = Ts / b
    D    = (1 - a) / b
    tau0 = c / b

Fitting: OLS on the velocity recursion with a scan over integer delays ``d``;
validation on an independent time split of the same recording (flagged as
such -- a truly separate validation recording is the next protocol step).
"""
from __future__ import annotations

import numpy as np


def _fit_rows(qd: np.ndarray, tau: np.ndarray, d: int,
              k_idx: np.ndarray) -> tuple:
    """Design matrix / target for rows ``k_idx`` (target = qd[k])."""
    A = np.column_stack([
        qd[k_idx - 1],
        tau[k_idx - 1 - d],
        np.ones(len(k_idx)),
    ])
    y = qd[k_idx]
    return A, y


def fit_velocity_arx(qd: np.ndarray, tau: np.ndarray, Ts: float,
                     max_delay: int = 4, train_frac: float = 0.7) -> dict:
    """Fit qd[k+1] = a qd[k] + b tau[k-d] + c by scanning delay d."""
    qd = np.asarray(qd, dtype=float)
    tau = np.asarray(tau, dtype=float)
    n = len(qd)
    results = []
    # all candidate row indices k in [1, n-1] with tau[k-1-d] defined
    for d in range(0, max_delay + 1):
        k_all = np.arange(1, n)
        k_all = k_all[k_all - 1 - d >= 0]
        if len(k_all) < 8:
            continue
        n_tr = int(len(k_all) * train_frac)
        k_tr = k_all[:n_tr]
        k_va = k_all[n_tr:]
        A_tr, y_tr = _fit_rows(qd, tau, d, k_tr)
        coef, *_ = np.linalg.lstsq(A_tr, y_tr, rcond=None)
        a, b, c = float(coef[0]), float(coef[1]), float(coef[2])
        if len(k_va) >= 4:
            A_va, y_va = _fit_rows(qd, tau, d, k_va)
            pred = A_va @ coef
            denom = float(np.sum((y_va - np.mean(y_va)) ** 2))
            vaf = 1.0 - float(np.sum((y_va - pred) ** 2)) / max(denom, 1e-12)
        else:
            vaf = float("nan")
        results.append({
            "d": d, "a": a, "b": b, "c": c,
            "I": float(Ts / b) if abs(b) > 1e-12 else float("inf"),
            "D": float((1.0 - a) / b) if abs(b) > 1e-12 else float("inf"),
            "tau0": float(c / b) if abs(b) > 1e-12 else float("inf"),
            "vaf_val": vaf,
        })
    if not results:
        return {}
    results.sort(key=lambda r: -r["vaf_val"] if np.isfinite(r["vaf_val"]) else -1e9)
    return results[0]


def simulate_joint(I: float, D: float, tau_bias: float, Ts: float,
                   tau: np.ndarray, delay: int = 0, qd0: float = 0.0,
                   noise_std: float = 0.0, seed: int = 0) -> np.ndarray:
    """Semi-implicit Euler plant used as the (synthetic) ground truth."""
    rng = np.random.default_rng(seed)
    n = len(tau)
    qd = np.zeros(n)
    qd[0] = qd0
    for k in range(n - 1):
        applied = tau[k - delay] if k - delay >= 0 else 0.0
        qd[k + 1] = qd[k] + Ts * (applied + tau_bias - D * qd[k]) / I
        if noise_std > 0:
            qd[k + 1] += rng.normal(0.0, noise_std)
    return qd


def prbs(n: int, rng: np.random.Generator, amp: float = 1.0, hold: int = 20) -> np.ndarray:
    """Pseudo-random binary sequence (amplitude +/-amp, hold ``hold`` steps)."""
    u = np.zeros(n)
    v = amp
    for k in range(n):
        if k % hold == 0:
            v = amp if rng.uniform() < 0.5 else -amp
        u[k] = v
    return u
