"""Tests for the pure joint model fitter (synthetic ground truth)."""
import numpy as np
import pytest

from system_identification.fit_joint_model import (
    fit_velocity_arx,
    prbs,
    simulate_joint,
)


def test_recovers_inertia_damping_no_delay():
    Ts = 0.01
    rng = np.random.default_rng(0)
    tau = prbs(3000, rng, amp=1.5, hold=15)
    I0, D0, bias = 0.9, 0.3, 0.1
    qd = simulate_joint(I0, D0, bias, Ts, tau, delay=0)
    fit = fit_velocity_arx(qd, tau, Ts, max_delay=2)
    assert fit["d"] == 0
    assert fit["I"] == pytest.approx(I0, rel=0.02)
    assert fit["D"] == pytest.approx(D0, rel=0.05)
    assert fit["tau0"] == pytest.approx(bias, abs=0.02)
    assert fit["vaf_val"] > 0.99


def test_recovers_delay():
    Ts = 0.01
    rng = np.random.default_rng(1)
    tau = prbs(3000, rng, amp=1.5, hold=10)
    I0, D0 = 1.0, 0.2
    qd = simulate_joint(I0, D0, 0.0, Ts, tau, delay=2)
    fit = fit_velocity_arx(qd, tau, Ts, max_delay=5)
    assert fit["d"] == 2
    assert fit["I"] == pytest.approx(I0, rel=0.05)
    assert fit["vaf_val"] > 0.99


def test_robust_to_measurement_noise():
    Ts = 0.01
    rng = np.random.default_rng(2)
    tau = prbs(4000, rng, amp=2.0, hold=12)
    I0, D0 = 1.2, 0.4
    qd = simulate_joint(I0, D0, 0.0, Ts, tau, delay=1, noise_std=1e-3)
    fit = fit_velocity_arx(qd, tau, Ts, max_delay=4)
    assert fit["d"] == 1
    assert abs(fit["I"] - I0) / I0 < 0.1
    assert fit["vaf_val"] > 0.9


def test_poor_excitation_flagged():
    """Constant command gives no velocity variation: fit must be unreliable
    (low VAF), which the caller must treat as insufficient excitation."""
    Ts = 0.01
    tau = np.full(1000, 0.5)
    qd = simulate_joint(1.0, 0.1, 0.0, Ts, tau)
    fit = fit_velocity_arx(qd, tau, Ts)
    assert fit["vaf_val"] < 0.5 or abs(fit["I"]) > 1.0
