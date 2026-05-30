# Roadmap

## Released

### v0.1.0 — Hard-Constraint Baseline
- Basic MPC controller plugin for `ros2_control`
- Hard state/input/input-rate constraints via OSQP
- RRBot simulation example
- **Issue:** ~40% primal infeasible rate from conflicting velocity/rate constraints

### v0.2.0 — Soft Velocity Constraints + Warm-Start Hardening
- Soft velocity constraints via slack variables (L1+L2 penalty)
- Partitioned warm-start shifting eliminating sporadic NaN sentinel bursts
- Post-fix benchmarks: 97.7% optimal solve rate, 1.13 rad position RMS, 16.3% deadline miss
- All three priority issues resolved (P0: position bounds, P1: soft constraints, P2: warm-start corruption)

## Planned

### v0.2.1 — Portability & Real-Time Optimization
- `c_float` portability in OSQP wrapper (Eigen::Map assumes double)
- Solver tuning: reduced iterations, cached Hessian, relaxed tolerances
- Native Linux (non-WSL2) benchmark validation
- `ControllerUpdateStats` integration for cycle-accurate deadline tracking

### v0.3.0 — Additional Robot Examples
- Diff-drive base MPC example
- Inverted pendulum / cart-pole example
- Hardware-in-the-loop testing with physical robot interface

### v0.4.0 — Advanced Features
- SQP / nonlinear MPC extensions
- Obstacle avoidance constraints
- CI/CD pipeline with automated benchmark regression testing

---

*See [CHANGELOG](CHANGELOG.md) for version history.*
