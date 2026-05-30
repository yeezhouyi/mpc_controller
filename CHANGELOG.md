# Changelog

All notable changes to this project will be documented in this file.

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
