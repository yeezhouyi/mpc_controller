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

### v0.2.1 — Real-Time Optimization and Portability Hardening
- Cached condensed-QP weight matrices and Hessian (Q_bar, R_bar, S_bar, P_sparse)
- Preallocated per-cycle work vectors (x_ref_stacked_, x_free_, rate_offset_, q_vec_, l_, u_)
- OSQP wrapper c_float portability (element-wise static_cast)
- Reused conversion buffers (q_buffer_, l_buffer_, u_buffer_, px_buffer_, dy_buffer_, p_buffer_)
- WSL2 paired A/B validation (5 alternating pairs): 4/5 pairs confirm ~12% cycle time reduction
- [x] All features implemented and validated via paired A/B benchmark

## Planned

### v0.2.2 — Runtime Characterization
- [ ] Native Ubuntu 24.04 benchmark (5+ runs)
- [ ] P95 / P99 latency reporting
- [ ] ControllerUpdateStats integration
- [ ] Additional solver parameter tuning
- [ ] Regenerate comparison charts with consolidated data

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
