# Changelog

All notable changes to this project will be documented in this file.

## v0.2.1-rc1 (2026-05-31)

### Performance
- Cache condensed-QP weight matrices and Hessian.
- Rebuild cost matrices only on Q/R/S hot update (cost_matrix_dirty_).
- Preallocate MPCController per-cycle work vectors.
- Reduce controller-layer Eigen heap allocations from 17+ to ~3 per cycle.
- Reuse OSQP wrapper conversion buffers (q, l, u, primal, dual, P).
- **Benchmark results:** Paired A/B (5 alternating pairs) confirms
  cached Hessian cycle time reduction: 3.06 ms → 2.69 ms (−12%, 4/5
  pairs show improvement). All 10 runs passed the ≤10% failed-cycle
  quality gate. See README for dual-reporting with cross-session and
  paired data.

### Fixed
- Q/R/S hot-update ordering: rebuild P before gradient computation to ensure
  mathematical consistency on the first cycle after a parameter change.
- OSQP wrapper now uses element-wise `static_cast<c_float>` instead of
  `Eigen::Map<Eigen::VectorXd>`, enabling single-precision OSQP builds.

### Changed
- osqp++.h: pre-allocated `q_buffer_`, `l_buffer_`, `u_buffer_`, `px_buffer_`,
  `dy_buffer_` replace per-cycle `std::vector` allocations in UpdateBounds,
  UpdateGradient, and SetPrimalDualWarmStart.

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
