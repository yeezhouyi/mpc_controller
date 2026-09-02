# Changelog

All notable changes to this project will be documented in this file.

## linear_mpc_controller (2026-09-02) — advancement work-in-progress

See `linear_mpc_controller/README.md` for the full status matrix.  Summary:

- U1/C0 audit of the v0.2.1 baseline (`docs/baseline_audit.md`).
- New differential-drive trajectory-tracking package `linear_mpc_controller/`:
  - ROS-free **Python reference core** (`mpc_core/`): Frenet error model,
    analytic LTV linearisation + ZOH discretisation, condensed-QP receding
    horizon MPC with a self-contained dense ADMM solver, health/fallback
    contract (critical states -> zero, QP failure -> deterministic
    degrade-to-stop), offline episode runner; 45+ pytest cases green.
  - Trajectory tools (straight/circle/S/u-turn + pose-only completion),
    benchmark metrics/manifest tooling with an archived reference-core
    baseline (`results/ref_core_baseline`), fast residual-RL environment
    (`mpc_rl_env`) and first-order-lag system identification
    (`system_identification`).
  - C++/Eigen core and ROS2 node/launch/config skeletons written to plan;
    compile + Gazebo gates pending WSL2 (no toolchain on the dev host).
- Status: pure linear MPC tracking baseline works offline on the four formal
  tracks with zero QP failures; residual-RL training and Gazebo closed-loop
  are staged next units (C3/C6-C8).

## v0.2.1 (2026-05-31)

### Performance
- Cache condensed-QP weight matrices and Hessian.
- Rebuild cost matrices only on Q/R/S hot update (cost_matrix_dirty_).
- Preallocate MPCController per-cycle work vectors.
- Reduce controller-layer Eigen heap allocations from 17+ to ~3 per cycle.
- Reuse OSQP wrapper conversion buffers (q, l, u, primal, dual, P).

### Fixed
- Q/R/S hot-update ordering: rebuild P before gradient computation to ensure
  mathematical consistency on the first cycle after a parameter change.
- OSQP wrapper now uses element-wise `static_cast<c_float>` instead of
  `Eigen::Map<Eigen::VectorXd>`, enabling single-precision OSQP builds.

### Changed
- osqp++.h: pre-allocated `q_buffer_`, `l_buffer_`, `u_buffer_`, `px_buffer_`,
  `dy_buffer_` replace per-cycle `std::vector` allocations in UpdateBounds,
  UpdateGradient, and SetPrimalDualWarmStart.

### Benchmark
- WSL2 paired A/B validation (10 runs, 5 alternating pairs) confirms
  ~12% cycle time reduction: v0.2.0 3.06 ms → v0.2.1 2.69 ms (4/5 pairs
  show improvement). All 10 runs passed the ≤10% failed-cycle quality gate.
- Native Linux characterization deferred to v0.2.2.

## v0.2.0 (2026-05-30)

### Added
- Soft velocity constraints via slack variables with L1+L2 penalty
- Partitioned warm-start shifting for z = [U, ε] decision variable layout
- `sol.allFinite()` defense against NaN corruption in warm-start
- `ROADMAP.md` with v0.1.0 → v0.4.0 release plan
- `CHANGELOG.md` for version tracking
- GitHub Actions CI workflow (build + test + lint)

### Fixed
- Warm-start monolithic shift bug (interleaved U and ε blocks)
- Slack diagnostics reading stale invalid data on PRIMAL_INFEASIBLE
- Warm-start reset return value not checked
- Position bounds mismatch with URDF revolute limits (P0)
- Sporadic NaN sentinel burst mitigated (0x7fc00000)

### Changed
- Benchmark results updated with post-fix metrics (97.7% solve rate, 1.13 rad RMS)
- README test section: separated functional tests from lint in-progress
- `package.xml` version bumped to 0.2.0

### Removed
- Placeholder diff-drive launch (did not load MPC controller)

## v0.1.0 (2026-05-15)

### Added
- Initial MPC controller plugin for `ros2_control`
- Linear model with A/B matrices from YAML
- OSQP solver integration via custom C++ wrapper
- Hard state/input/input-rate constraints
- Condensed QP formulation
- Dynamic parameter tuning (Q/R/S weights)
- Diagnostics publishing via Float64MultiArray
- RRBot Gazebo simulation example
- `benchmark_plot.py` for publication-quality visualization
