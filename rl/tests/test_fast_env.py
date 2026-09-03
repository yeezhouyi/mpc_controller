"""C4 fast-env contract tests: determinism, zero-residual==pure MPC,
obs dims, action bounds, reward sanity."""
import numpy as np
import pytest

from rl.fast_env.joint_arm import ReferenceProfile
from rl.fast_env.residual_env import JointResidualEnv, OBS_DIM


def _env(**kw):
    kw.setdefault("profile", ReferenceProfile(amplitude=0.6, freq=0.4, duration_s=4.0))
    kw.setdefault("difficulty", 0.0)
    return JointResidualEnv(**kw)


def test_obs_shape_and_finite():
    env = _env()
    obs, _ = env.reset(seed=0)
    assert obs.shape == (OBS_DIM,)
    assert np.all(np.isfinite(obs))


def test_seed_determinism():
    def run():
        env = _env(difficulty=0.5)
        env.reset(seed=3)
        rng = np.random.default_rng(9)
        obs = []
        for _ in range(60):
            o, r, done, info = env.step(rng.uniform(-1, 1, 2))
            obs.append(o.copy())
            if done:
                break
        return np.array(obs)
    a, b = run(), run()
    assert np.allclose(a, b)


def test_zero_residual_matches_pure_mpc_baseline():
    """alpha=0 + zero action must reproduce the baseline MPC commands 1:1."""
    env = _env(difficulty=0.0)
    env.reset(seed=1)
    for _ in range(120):
        x = env.plant.observe()
        expected = env._baseline_command(x, env.t)
        obs, rew, done, info = env.step(np.zeros(2))
        np.testing.assert_allclose(env.prev_tau, expected, atol=1e-9)
        if done:
            break


def test_tracks_sine_with_small_error_no_randomisation():
    """Baseline MPC alone (alpha=0) should track the slow sine closely."""
    env = _env(profile=ReferenceProfile(amplitude=0.5, freq=0.2, duration_s=8.0))
    env.reset(seed=0)
    max_e = 0.0
    for _ in range(int(8.0 / 0.02)):
        obs, rew, done, info = env.step(np.zeros(2))
        x = env.plant.observe()
        qr = env.profile.q(env.t)
        max_e = max(max_e, float(np.max(np.abs(x[0:2] - qr))))
        if done:
            break
    assert max_e < 0.25, f"baseline tracking error too large: {max_e}"


def test_random_actions_stay_bounded_and_finite():
    env = _env()
    env.reset(seed=2)
    rng = np.random.default_rng(5)
    for _ in range(80):
        obs, rew, done, info = env.step(rng.uniform(-1, 1, 2))
        assert np.all(np.isfinite(obs))
        assert np.isfinite(rew)
        assert np.max(np.abs(env.prev_tau)) <= env.tau_max + 1e-9
        if done:
            env.reset(seed=2)


def test_reward_completion_bonus_and_penalties():
    env = _env()
    env.reset(seed=0)
    r_good = env.reward(np.zeros(2), np.zeros(2), np.zeros(2), np.zeros(2), 0.01, True, "completed")
    r_bad = env.reward(np.array([1.0, 1.0]), np.array([0.5, 0.5]), np.array([9.0, 9.0]),
                       np.array([4.0, 4.0]), 0.0, False, "")
    assert r_good > r_bad
