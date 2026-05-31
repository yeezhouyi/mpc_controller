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

### v0.2.1-rc1 — Portability & Real-Time Optimization
- [x] Cache condensed-QP weight matrices and Hessian (Q_bar, R_bar, S_bar, P_sparse)
- [x] Preallocate MPCController per-cycle work vectors (x_ref_stacked_, x_free_, rate_offset_, q_vec_, l_, u_)
- [x] Fix hot-update ordering (rebuild P before gradient on Q/R/S change)
- [x] Reuse OSQP wrapper conversion buffers (q_buffer_, l_buffer_, u_buffer_, px_buffer_, dy_buffer_, p_buffer_)
- [x] Add `c_float` portability (element-wise conversion instead of Eigen::Map)
- [x] WSL2 A/B benchmark: clean-run aggregate 3,876 µs solve time, 98.4% optimal rate
- [x] WSL2 paired A/B validation (5 alternating pairs, v0.2.0 vs v0.2.1): 4/5 pairs confirm ~12% cycle time reduction
- [ ] Native Linux benchmark (5+ runs, Ubuntu 24.04)
- [ ] Publish v0.2.1 stable (after experimental validation complete)
- [ ] ControllerUpdateStats integration (deferred to v0.3.0)

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
