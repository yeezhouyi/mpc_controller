# Repo Snapshot — 2026-09-03 (mpc_controller audit, plan week 0)

Source plan: `title_ RRBot 关节.docx` (RRBot 关节空间线性 MPC 与残差强化学习进阶计划).

## Repository and baseline

| Field | Value |
|---|---|
| Repository | `yeezhouyi/mpc_controller` (SSH: `git@github.com:yeezhouyi/mpc_controller.git`) |
| Default branch | `main` |
| Plan snapshot commit | `04242b2` (github-snapshot-2026-09-03) |
| **Audited HEAD (this machine)** | `25b0929` — merge of `feature/v0.2.2-runtime-characterization` (PR #2) |
| Local path (WSL2) | `/home/zhouyi/ros2_ws/src/mpc_controller` |
| Working tree | clean |

> Note: `main` on this machine has advanced past the plan snapshot with the
> v0.2.2 runtime-characterization merge (paired A/B benchmark tooling,
> cached condensed Hessian OSQP portability).  All audit numbers below are
> read from HEAD `25b0929`; they supersede the plan's `04242b2` snapshot but
> do not change any interface conclusions.

## Object of the controller (boundary statement)

`mpc_controller` is a **`ros2_control` controller plugin** for the RRBot
2-DOF planar manipulator in Gazebo.  Its I/O contract is joint-space:

- state read: joint positions + velocities (`q, qdot`);
- reference: joint-space arrays on `~/reference` (`std_msgs/Float64MultiArray`);
- command written: joint effort (`effort`) command interface;
- output observable: `~/diagnostics`.

It does **not** emit `cmd_vel` and is not a mobile-base controller.  Do not
bridge it to `ros2_tunnel_explorer`'s TurtleBot3/Nav2 line (plan §1/§3).

## Interfaces and plugin structure

- Plugin: `mpc_controller/MPCController` (`mpc_controller_plugins.xml`),
  class `mpc_controller::MPCController : controller_interface::ControllerInterface`.
- State vector: `x = [q1, qdot1, q2, qdot2]` (nx = 4).
- Input vector: `u = [effort1, effort2]` (nu = 2).
- State interface names: `joint1/position, joint1/velocity, joint2/position, joint2/velocity`.
- Command interface names: `joint1/effort, joint2/effort`.
- Lifecycle entry points seen in `include/mpc_controller/mpc_controller.hpp`:
  `command_interface_configuration()`, `state_interface_configuration()`,
  `on_init()`, `on_configure()`, `on_activate()`, `on_deactivate()`, `update()`
  plus internals `buildQPStructure()`, `rebuildCachedMatrices()`,
  `buildAndSolveQP(x0)`, `readState()`, `writeCommand(u0)`,
  `parameterUpdateCallback()`.
- Core algorithm layers (pure C++ / Eigen, no ROS):
  - `LinearModel` (`linear_model.hpp/cpp`): configured discrete LTI
    `x[k+1] = A x[k] + B u[k]`, row-major `A_data/B_data`.
  - `OSQPSolver` (`osqp_solver.hpp/cpp`): OSQP-backed `SolverBase`
    implementation with `initialize(state_dim, input_dim, horizon)`,
    cached structure setup, gradient/cost/bounds updates, warm start,
    slack variables for soft velocity constraints.
  - `mpc_controller.cpp`: ros2_control glue, realtime reference buffer,
    diagnostics publisher.

## Config and parameters (`config/rrbot_mpc.yaml`)

| Param | Value |
|---|---|
| controller_manager update_rate | 100 Hz (target) |
| prediction_horizon | 20 |
| dt | 0.01 s |
| Model | double integrator per joint (unit mass), A/B as documented in file |
| Q_diag | [100, 1, 100, 1] (position 100, velocity 1) |
| R_diag | [0.1, 0.1] (effort) |
| S_diag | [1.0, 1.0] (effort rate) |
| State bounds | position ±3.10 rad (URDF ±3.14 − 0.04 margin), velocity ±5 rad/s |
| Input bounds | effort ±3 Nm |
| Input rate bounds | ±10 Nm/s (scaled by dt → ±0.1 Nm per cycle) |
| velocity_indices | [1, 3]; slack_rho_1 100, slack_rho_2 10 (soft velocity) |

Known fixed defect (README P0): first-cycle primal infeasible (OSQP −3)
when state bounds were tighter than URDF limits; fix = URDF `initial_value=0.0`
and position bounds ±3.10.

## Benchmark baseline (must reproduce, not claim as improvement)

From README (5 × 65 s RRBot Gazebo runs, soft velocity, OSQP 0.6.3):

| Metric | All runs (01–05) | Clean runs (01–02) |
|---|---|---|
| Optimal solve rate | 89.9 % | 98.4 % |
| Mean solve time | 5.81 ms | 3.88 ms |
| Deadline miss rate | 13.1 % | 5.69 % |
| NaN sentinel | not observed | not observed |

Do not write "100 Hz achieved" or "5.81 ms worst case" — 100 Hz is the
controller_manager *target*; measured update cadence was ~48–83 Hz under
WSL2 diagnostics.

## Environment

- Ubuntu 24.04 (WSL2, distro `Ubuntu-24.04`), ROS2 Jazzy, Gazebo Harmonic,
  OSQP 0.6.3; build via `colcon` at `/home/zhouyi/ros2_ws`.
- RTX 4060 host is not a precondition for QP timing (CPU-bound).

## Explorer companion line (unchanged)

`ros2_tunnel_explorer` fixed at `15fc269` (stage3d-entrance-loop-recovery);
Stage 3D baseline 5/5 explorer completion, recovery probe 4/4.  This line is
tracked separately (see the tunnel cleaning-track docs).
