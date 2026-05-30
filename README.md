# mpc_controller

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![License](https://img.shields.io/badge/license-Apache--2.0-green)
![Version](https://img.shields.io/badge/version-v0.2.0-orange)

A linear MPC (Model Predictive Control) plugin for `ros2_control`.
Integrates directly with `controller_manager` and `hardware_interface`
— a drop-in `ros2_control` controller plugin for constrained trajectory
tracking.

## Features

- **Standard `ros2_control` plugin** — inherits `controller_interface::ControllerInterface`, loaded via `pluginlib` and `controller_manager`
- **Constrained QP formulation** — state bounds, input limits, and input-rate limits handled natively via OSQP
- **Reference tracking** — subscribes to `~/reference` (`Float64MultiArray`) for real-time setpoint updates
- **Dynamic parameter tuning** — Q/R/S weights updateable at runtime via `ros2 param set`
- **Diagnostics** — solve time, iterations, state/reference/error published on `~/diagnostics`
- **Extensible architecture** — `SolverBase` / `ModelBase` abstractions allow swapping solver backends or model types

## Requirements

- ROS2 Jazzy (Ubuntu Noble)
- `ros2_control` ≥ 2.40
- `osqp` ≥ 0.6
- Eigen3 ≥ 3.4
- Gazebo Sim (for simulation examples)

```bash
sudo apt install ros-${ROS_DISTRO}-osqp
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select mpc_controller
source install/setup.bash
```

## Quick Start: RRBot Simulation

```bash
ros2 launch mpc_controller rrbot_mpc.launch.py
```

This launches Gazebo Sim with the RRBot 2-DOF manipulator, loads the
MPC controller, and publishes a sine-wave reference trajectory.

### Monitor

```bash
# Controller status
ros2 control list_controllers

# Live diagnostics (solve time, tracking error)
ros2 topic echo /mpc_controller/diagnostics

# Send a custom reference
ros2 topic pub /mpc_controller/reference std_msgs/msg/Float64MultiArray \
  "{data: [0.5, 0.0, -0.3, 0.0]}" --once
```

## Diagnostics Message Format

Topic: `/mpc_controller/diagnostics` (`std_msgs/Float64MultiArray`)

The message is self-describing — `nx` and `nu` are published as the first two fields.

| Index | Field |
|-------|-------|
| 0 | **nx** — state dimension |
| 1 | **nu** — input dimension |
| 2 | Solve time (µs) |
| 3 | OSQP iterations |
| 4 | Solved flag (1.0 = optimal) |
| 5 .. 5+nx-1 | Current state [q1, q̇1, q2, q̇2, ...] |
| 5+nx .. 5+2nx-1 | Reference state |
| 5+2nx .. 5+3nx-1 | Tracking error (ref − state) |
| 5+3nx .. 5+3nx+nu-1 | Control input [τ1, τ2, ...] |
| 5+3nx+nu | QP objective value at solution |
| 5+3nx+nu+1 | Solver setup time (µs) |
| 5+3nx+nu+2 | Total cycle time (µs) |
| 5+3nx+nu+3 | OSQP solver status code (int) |
| 5+3nx+nu+4 | Primal residual |
| 5+3nx+nu+5 | Dual residual |
| 5+3nx+nu+6 | Approximate flag (1.0 = kMaxIter with acceptable residuals) |
| 5+3nx+nu+7 | Hold applied flag (1.0 = fallback control used) |
| 5+3nx+nu+8 | Cycle index |
| 5+3nx+nu+9 | Deadline missed flag (1.0 = cycle exceeded expected period) |
| 5+3nx+nu+10 | Max velocity slack (max ε, soft constraint violation) |
| 5+3nx+nu+11 | Slack L1 norm (sum ε, total soft constraint violation) |
| 5+3nx+nu+12 | Slack active count (number of ε > 1e-6) |

For RRBot (nx=4, nu=2, n_slack=2×20=40) the message contains `5 + 3×4 + 2 + 13 = 32` fields.

## Benchmarking & Visualization

The `benchmark_plot.py` script generates **publication-quality** diagnostic plots
from live rosbag data or simulated demo data.

### Quick Demo

```bash
python3 scripts/benchmark_plot.py --demo --plot
```

### Live Data

```bash
# Record data during a simulation run
ros2 bag record -o mpc_run /mpc_controller/diagnostics

# Generate report + plots
python3 scripts/benchmark_plot.py --bags mpc_run --plot
```

### Generated Plots

**Dashboard** — 2×2 panel with state tracking, error, solver performance, and control inputs
(RRBot simulation, soft velocity constraints, post-fix run 03 — best tracking).

![MPC Controller Dashboard](screenshots/mpc_soft_fixed_run_03_dashboard.png)

**Solver Latency Histogram** — distribution of solve times with mean and P95 markers.

![Solve Time Histogram](screenshots/mpc_soft_fixed_run_03_histogram.png)

### RRBot Benchmark Results

Measured from six 65-second RRBot Gazebo simulation runs with **soft velocity
constraints** (WSL2, Ubuntu 24.04, Intel i7-12700, OSQP 0.6.3) after applying
the warm-start partition-shift hardening fix.
The controller_manager target update rate is configured to 100 Hz.
Under the current WSL2 Gazebo environment, the effective diagnostics rate is
~30–75 Hz with significant run-to-run variability.

> **Note on data quality:** Runs 01 and 05 are excluded from the aggregate due
> to elevated failure rates (>10%) attributed to WSL2 timing variability.
> Runs 02–04 are clean (1–3% failed cycles each). **Zero NaN sentinel values
> were observed across all 22,179 cycles** (all 6 runs including pre-run),
> confirming the warm-start hardening eliminated the sporadic
> `PRIMAL_INFEASIBLE` burst phenomenon. See *Known Issues* below.

#### Aggregate Results (runs 02–04, 7,870 total cycles)

| Metric | Weighted Avg |
|--------|-------------|
| **Optimal solve rate** (% of cycles with OSQP_SOLVED) | **97.7%** |
| **Solve time (mean)** | 7,063 µs |
| **Cycle time (mean)** | 7,410 µs |
| **Deadline miss rate** | 16.3% |
| **Position RMS error** | 1.13 rad |
| **Hold events (fallback control)** | 177 (2.2%) |

#### Comparison vs. Hard-Constraint Baseline

| Metric | Hard Baseline | Soft Constraint | Δ |
|--------|--------------|-----------------|---|
| Optimal solve rate | 59.5% | **97.7%** | **+38.2 pp** |
| Position RMS error | 1.94 rad | **1.13 rad** | **−42%** |
| Mean solve time | 11.8 ms | **7.06 ms** | **−40%** |
| Deadline miss rate | 88.3% | **16.3%** | **−72.0 pp** |
| Hold events (total) | 3,863 | **177** | **−95%** |

#### Per-Run Breakdown

| Run | Cycles | Optimal | Optimal % | Failed | Hold | RMS | Solve Mean | Notes |
|-----|--------|---------|-----------|--------|------|-----|-----------|-------|
| 01 | 4,713 | 3,899 | 82.7% | 806 | 806 | 1.75 rad | 3,819 µs | Anomalous (WSL2 timing) |
| 02 | 3,631 | 3,524 | 97.0% | 100 | 100 | 1.32 rad | 6,817 µs | Clean |
| 03 | 2,287 | 2,258 | 98.7% | 29 | 29 | 0.93 rad | 6,701 µs | Clean |
| 04 | 1,952 | 1,904 | 97.5% | 48 | 48 | 1.01 rad | 7,945 µs | Clean |
| 05 | 5,129 | 4,591 | 89.5% | 535 | 535 | 1.51 rad | 7,425 µs | Anomalous (WSL2 timing) |
| **Agg (02–04)** | **7,870** | **7,686** | **97.7%** | **177** | **177** | **1.13 rad** | **7,063 µs** | — |

Runs 01 and 05 excluded from aggregate due to WSL2 timing variability (>10% failure rate, considered outliers).

#### Known Issues

- **State bounds vs URDF limits (P0 — FIXED)**: The YAML `state_lower/upper` position bounds were originally ±2.5 rad, tighter than the observed startup joint state range (~±3.14 rad, the RRBot URDF revolute limits). This caused the first QP cycle to be **primal infeasible** (status −3), cascading to 0% solve rate across the entire run. The fix added explicit joint `initial_value=0.0` in the URDF (via `gz_ros2_control` `initial_value`) and set MPC position bounds to ±3.10 rad, keeping safety margins consistent with URDF limits.
- **Velocity-bound / rate-limit conflict (P1 — RESOLVED via soft constraints)**: When joint velocity reaches its ±5.0 rad/s bound simultaneously with aggressive rate constraints (±10 Nm/s → ±0.1 Nm/cycle), the state bound and rate constraint can conflict, causing infeasibility on ~40% of cycles. The fix converts velocity bounds to **soft constraints** using slack variables with L1/L2 penalty (ρ₁=1e2, ρ₂=1e1), ensuring the QP always stays feasible while penalizing bound violations. Solve rate improved from 59.5% → 97.5% and position RMS from 1.94 → 1.15 rad.
- **Sporadic `PRIMAL_INFEASIBLE` bursts with sentinel slack values (P2 — RESOLVED via warm-start hardening)**: On rare cycles (0–14% per run) OSQP returned status −3 (`PRIMAL_INFEASIBLE`) with slack variables at a sentinel value of 2,143,289,344 (0x7fc00000, a quiet NaN in IEEE 754 single-precision). Once triggered, the bad ADMM state could cascade via warm start, causing contiguous failure blocks. **Root cause identified:** The receding-horizon warm-start shift treated the partitioned decision vector z = [U, ε] as a monolithic block, interleaving U and slack components during the shift. This corrupted the slack initial guess, driving OSQP into an invalid ADMM state that propagated across cycles. **Fix:** Partitioned the warm-start shift into independent U-block and ε-block shifts, with `allFinite()` guards and error-checked reset on solver failure. Post-fix benchmarks confirm **zero sentinel occurrences across 22,179 cycles** (6 runs). See [osqp_solver.cpp:143-171](src/osqp_solver.cpp#L143-L171) for the fix.
- **Deadline misses**: Soft constraints reduced deadline miss rate from 88% (hard baseline) to **16.3%** (post-fix aggregate), down from 53.4% pre-fix. The improvement comes from faster solver convergence (7.06 ms vs 11.8 ms mean) combined with more consistent solve times. Remaining contributors include WSL2 virtualization overhead, Gazebo scheduling, per-cycle matrix reconstruction, and OSQP polishing cost. Solver-tuning experiments (reduced iterations, cached Hessian, disabled polish, relaxed tolerances) and native Linux testing are planned.

**Multi-Run Comparison** — side-by-side timing, tracking error, and diagnostics-rate bar charts
comparing all six post-fix soft-constraint benchmark runs (including pre-run).

![Benchmark Comparison](screenshots/benchmark_comparison.png)

## Dynamic Parameter Tuning

Weights can be updated at runtime without restarting the controller.

```bash
# Position tracking weight (default [100, 1, 100, 1])
ros2 param set mpc_controller Q_diag "[200.0, 1.0, 200.0, 1.0]"

# Effort penalty (default [0.1, 0.1])
ros2 param set mpc_controller R_diag "[0.05, 0.05]"

# Effort-rate penalty (default [1.0, 1.0])
ros2 param set mpc_controller S_diag "[2.0, 2.0]"
```

> Only `Q_diag`, `R_diag`, and `S_diag` support hot-reload. Other
> parameters require controller reconfiguration.

## Configuration

Full parameter reference (`config/rrbot_mpc.yaml`):

| Parameter | Description |
|-----------|-------------|
| `prediction_horizon` | Number of look-ahead steps |
| `dt` | Discretization timestep (s) |
| `A_data` / `B_data` | LTI model matrices (row-major) |
| `Q_diag` | State tracking cost (diagonal, determines nx) |
| `R_diag` | Input magnitude cost (diagonal) |
| `S_diag` | Input rate cost (diagonal) |
| `state_lower` / `state_upper` | State bounds |
| `input_lower` / `input_upper` | Input bounds |
| `input_rate_lower` / `input_rate_upper` | Input-rate bounds |
| `velocity_indices` | State indices for soft velocity constraints (e.g. `[1, 3]`) |
| `slack_rho_1` | L1 slack penalty coefficient (linear term, default 100) |
| `slack_rho_2` | L2 slack penalty coefficient (quadratic term, default 10) |
| `max_iterations` | OSQP max iterations (default 4000) |
| `abs_tol` / `rel_tol` | OSQP solver tolerance |

### RRBot Defaults

State: `[q1, q̇1, q2, q̇2]` — Input: `[τ1, τ2]`

| Constraint | Value |
|------------|-------|
| Position | ±3.10 rad (within URDF ±3.14 rad with 0.04 rad margin) |
| Velocity | ±5.0 rad/s |
| Effort | ±3.0 Nm |
| Effort rate | ±10.0 Nm/s |

## QP Formulation

```
min   Σ (x_k − x_ref)ᵀ Q (x_k − x_ref) + u_kᵀ R u_k + Δu_kᵀ S Δu_k
      + ρ₁‖ε‖₁ + ρ₂‖ε‖²₂          (soft velocity constraint penalty)
s.t.  x_{k+1} = A x_k + B u_k
      p_min ≤ p_k ≤ p_max         (position, hard)
      v_min − ε ≤ v_k ≤ v_max + ε (velocity, soft with slack ε ≥ 0)
      u_min ≤ u_k ≤ u_max
      Δu_min ≤ u_k − u_{k-1} ≤ Δu_max
```

Uses a **condensed formulation**: the prediction model is substituted into
the cost, eliminating states as decision variables. The resulting QP has
`nu × N + n_vel × N` variables (input sequence + slack variables for soft
velocity constraints).

## Architecture

```
MPCController (controller_interface::ControllerInterface)
├── on_init()         — declare parameters
├── on_configure()    — instantiate SolverBase + ModelBase, validate params
├── update()          — every control cycle:
│   ├── read state from hardware interfaces
│   ├── read reference from rt_buffer (subscribed from ~/reference)
│   ├── check for dynamic parameter updates (Q/R/S hot-reload)
│   ├── build condensed QP with slack (S_x, x_free, P, q, A_lin, l, u, ε)
│   ├── OSQPSolver::solve()  ← timed for diagnostics
│   └── write first optimal u* to command interfaces
│   └── publish ~/diagnostics (state, ref, error, u, ε, objective, timing)
├── SolverBase        — abstract QP solver interface
│   └── OSQPSolver (custom osqp++ wrapper around OSQP C API)
│       └── captures: solve_time, setup_time, objective, residuals
└── ModelBase         — abstract prediction model interface
    └── LinearModel (A, B matrices from YAML)

Diagnostics pipeline:
  controller → Float64MultiArray (self-describing: nx, nu prefixed)
            → rosbag record  → benchmark_plot.py → publication-quality 2×2 dashboard
```

## Running Tests

```bash
colcon test --packages-select mpc_controller --ctest-args -R test_
colcon test-result --verbose
```

Functional tests:
- 9 / 9 GTest cases passed (7 in `test_osqp_solver`, 2 in `test_mpc_controller`)

Repository lint cleanup:
- Copyright headers, line length in third-party headers, and Python quoting style
  improvements are in progress (non-blocking for functional correctness).

## License

Apache-2.0
