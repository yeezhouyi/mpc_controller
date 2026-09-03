#!/usr/bin/env python3
"""Evaluate a trained residual policy vs the pure-MPC baseline (U9/C7).

Usage:
    python mpc_rl_env/algorithms/evaluate_policy.py \
        --policy outputs/ppo_residual/checkpoint.zip \
        --seeds 0 1 2 --track s_curve

Reports mean per-episode reward and lateral-error RMS.  Training uses
different seeds/tracks than evaluation (no training-return claims, C7).
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from mpc_core.types import MpcParams  # noqa: E402


def run_episode(env, policy=None, max_steps: int = 600) -> dict:
    obs, _ = env.reset(seed=None)
    total_r = 0.0
    eys = []
    for _ in range(max_steps):
        if policy is None:
            action = np.zeros(2, dtype=np.float32)  # pure MPC (zero residual)
        else:
            action, _ = policy.predict(obs, deterministic=True)
        obs, r, terminated, truncated, info = env.step(action)
        total_r += r
        eys.append(info.get("e_y", 0.0))
        if terminated or truncated:
            break
    return {"reward": total_r, "e_y_rms": float(np.sqrt(np.mean(np.square(eys)))) if eys else 0.0,
            "steps": len(eys)}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default=None, help="SB3 checkpoint; None = pure MPC")
    ap.add_argument("--track", default="s_curve")
    ap.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2])
    ap.add_argument("--episodes-per-seed", type=int, default=2)
    args = ap.parse_args()

    from mpc_rl_env.envs.gym_env import GymResidualTrackingEnv

    env = GymResidualTrackingEnv(track=args.track, mpc_params=MpcParams(N=25))

    policy = None
    if args.policy:
        try:
            from stable_baselines3 import PPO
        except ImportError as e:  # pragma: no cover
            print(f"[eval] sb3 not available: {e}")
            sys.exit(2)
        policy = PPO.load(args.policy)

    results = []
    for seed in args.seeds:
        env.reset(seed=seed)
        for _ in range(args.episodes_per_seed):
            res = run_episode(env, policy)
            results.append(res)
            tag = "policy" if policy is not None else "pure_mpc"
            print(f"[{tag} seed={seed}] reward={res['reward']:.2f} e_y_rms={res['e_y_rms']:.4f} steps={res['steps']}")
    print(f"mean reward={np.mean([r['reward'] for r in results]):.2f} "
          f"mean e_y_rms={np.mean([r['e_y_rms'] for r in results]):.4f} "
          f"({len(results)} episodes)")


if __name__ == "__main__":
    main()
