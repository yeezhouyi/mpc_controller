# Model and QP Derivation — RRBot joint-space linear MPC

This note derives the model and the quadratic program that
`mpc_controller` builds and solves each control cycle.  It documents what
the code and configuration actually express (see
`config/rrbot_mpc.yaml`, `include/mpc_controller/linear_model.hpp`,
`osqp_solver.hpp`, `types.hpp`), and is the companion of the repo snapshot
and interface-contract documents.

## 1. State definition

Per joint of the RRBot planar arm the state is position and velocity, so
for the 2-DOF robot

```
x = [q1, qdot1, q2, qdot2]ᵀ        (nx = 4)
u = [tau1, tau2]ᵀ                  (nu = 2, joint effort commands)
```

The plant model used for prediction is a **configured discrete
double-integrator** (unit mass), not a continuous-time model discretised at
runtime:

```
q[k+1]   = q[k]   + dt * qdot[k]
qdot[k+1] = qdot[k] + dt * u[k]
```

which in state-space form is the discrete LTI model

```
x[k+1] = A x[k] + B u[k]                      (1)
```

with

```
A = [1 dt 0  0 ]      B = [0   0 ]
    [0  1 0  0 ]          [dt  0 ]
    [0  0 1 dt ]          [0   0 ]
    [0  0 0  1 ]          [0   dt]
```

`A`/`B` are supplied row-major in YAML (`A_data`, `B_data`) and are loaded
by `LinearModel::initialize()`, which validates the dimensions
(`A_data == nx*nx`, `B_data == nx*nu`) and throws `std::invalid_argument`
otherwise.  `LinearModel::predict(x0, u_seq)` propagates (1) for
`N = len(u_seq)/nu` steps and returns the `nx × (N+1)` state trajectory.

Because the linearisation point and sampling time are baked into the
configured `A/B`, changing `dt` or the plant requires new model matrices —
the derivation note keeps the same form but the numbers must be
re-validated whenever those change.

## 2. Tracking formulation (errors)

The controller tracks a joint reference trajectory.  With the reference
state stacked over the horizon, define the per-step error

```
e[k] = x[k] − x_ref[k]
```

and the effort `u[k]`.  Input-rate limits are expressed on the *change* of
command per step, `Δu[k] = u[k] − u[k−1]`, which in physical units are
configured as ±10 Nm/s and internally scaled by `dt` (≈ ±0.1 Nm per cycle).

## 3. Cost

Over a prediction horizon of `N = 20` steps:

```
J = Σ_{k=1..N} e[k]ᵀ Q e[k]
  + Σ_{k=0..N−1} u[k]ᵀ R u[k]
  + Σ_{k=1..N−1} Δu[k]ᵀ S Δu[k]
  + soft-velocity terms
```

- `Q` diagonal [100, 1, 100, 1]: position error dominates over velocity.
- `R` diagonal [0.1, 0.1]: effort penalty.
- `S` diagonal [1.0, 1.0]: effort-rate penalty (smoothness).

In condensed form this becomes the standard QP

```
min   (1/2) zᵀ H z + gᵀ z
s.t.  l ≤ C z ≤ u
```

where `z` stacks the decision variables (commands and, with the soft
velocity option, slack variables).  The Hessian `H` is **cached/condensed**
once per configuration (see `rebuildCachedMatrices()` and the v0.2.1+ notes)
and only the gradient and bounds are updated every cycle, which is the main
reason clean-run solve time drops after the structure is built once.

## 4. Constraints

| Constraint | Form | RRBot config |
|---|---|---|
| State (position/velocity) | `x_low ≤ x[k] ≤ x_up` | pos ±3.10 rad (URDF ±3.14 − margin), vel ±5 rad/s |
| Input (effort) | `u_low ≤ u[k] ≤ u_up` | ±3 Nm |
| Input rate | `Δu_low ≤ u[k] − u[k−1] ≤ Δu_up` | ±10 Nm/s → ±0.1 Nm/cycle |
| Soft velocity | slack on velocity indices [1,3] | `slack_rho_1 = 100` (L1), `slack_rho_2 = 10` (L2) |

Soft velocity constraints avoid primal infeasibility when the reference is
aggressive: the velocity limits are allowed to be violated at a large
penalty instead of making the QP infeasible.

## 5. Solver, warm start and status handling

`OSQPSolver` (OSQP 0.6.3):

- `initialize(nx, nu, N)` sizes and (via `setupProblem`) stores the sparse
  constraint structure; subsequent cycles only call the cheap update
  methods (`updateGradient`, `updateCostMatrix`, `updateBounds`) and
  `solve()`.
- Warm start uses the previous solution (`setWarmStart`).
- `SolverDiagnostics` carries `solve_time_us`, `setup_time_us`,
  `cycle_time_us`, `iterations`, `status` (OSQP code), `solved`,
  `solved_approximate`, `objective`, `pri_res`, `dua_res` — these are the
  fields the runtime-characterisation and failure-classification work must
  report per cycle (see README benchmark numbers: all-run optimal solve
  89.9 %, clean-run 98.4 %; mean 5.81 ms all / 3.88 ms clean).

Fallback contract (must hold in code and tests):

1. If OSQP returns `kSolved` → apply `u0` from the solution.
2. If status is non-optimal/infeasible/timeout (`kMaxIter` with
   unacceptable residual → `solved_approximate` false) → **do not** write a
   stale or unvalidated command; go through the deterministic fallback path
   (README documents the NaN sentinel and the P0 infeasibility fix with
   URDF `initial_value=0.0` and bounds ±3.10).
3. NaN/invalid reference or dimension mismatch must never reach the
   command interface (guarded validation; reference topic contract in
   `controller_interface_contract.md`).

## 6. What the numbers do *not* say

- 100 Hz is the `controller_manager` update target; WSL2 diagnostics show
  ~48–83 Hz actual cadence.  Reporting must state target vs measured and
  separate solver failure from WSL2/Gazebo slowdown and ROS2/DDS latency.
- 5.81 ms is a mean, not a worst-case guarantee.
- These figures are Gazebo-only; no Sim2Real claims.

## 7. Suggested next unit coverage (C1)

- `test/test_linear_model.cpp` — dimension validation, row-major loading,
  manual-recursion equivalence of `predict()`, zero-input free response
  (added in this slice).
- QP status/fallback tests on the solver (extend `test/osqp_solver_test.cpp`)
  and reference-validation tests around the controller's realtime buffer
  (needs a small reference-handling seam — next unit).
