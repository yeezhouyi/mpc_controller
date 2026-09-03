# Controller Interface Contract — mpc_controller (RRBot joint-space MPC)

Audit of the ros2_control plugin contract on HEAD `25b0929` (plan snapshot
`04242b2`, see `repo_snapshot_2026-09-03.md`).

## 1. Role and ownership

- Only **one** writer may command `joint1/effort`, `joint2/effort`:
  `mpc_controller/MPCController` activated in `controller_manager`.
  A residual-RL training node must never write the joint command interface
  directly; residual actions are delivered into the MPC pipeline (future
  `rl/` work) and the controller remains the single command writer.
- The controller reads joint states and writes effort commands.  It does
  not produce `cmd_vel` (that belongs to a future differential-base MPC
  adapter / `nav2_core::Controller`, a separate project line).

## 2. Plugin and lifecycle

| ros2_control hook | mpc_controller method / behaviour |
|---|---|
| pluginlib registration | `mpc_controller/MPCController` → `mpc_controller::MPCController` (see `mpc_controller_plugins.xml`) |
| `on_init()` | declare parameters (model, cost, bounds, interface names, solver) |
| `on_configure()` | validate parameters, build model + QP structure (`buildQPStructure`, `rebuildCachedMatrices`) |
| `on_activate()` | claim state/command interfaces, reset buffers/warm start |
| `update()` | read state → read reference → build/solve QP → write command |
| `on_deactivate()` | release interfaces; do not write commands |

Managed by `controller_manager`; the launch spawns
`joint_state_broadcaster` and then `mpc_controller` via `spawner`
(`launch/rrbot_mpc.launch.py`) with `--param-file config/rrbot_mpc.yaml`.

## 3. Interfaces (claimed names)

State interfaces (read-only):

```
joint1/position, joint1/velocity, joint2/position, joint2/velocity
```

Command interfaces (write-only, effort):

```
joint1/effort, joint2/effort
```

State vector layout `x = [q1, qdot1, q2, qdot2]ᵀ` and command layout
`u = [effort1, effort2]ᵀ` must stay consistent between YAML interface
names and the internal `Eigen` vectors; `state_interface_names` /
`command_interface_names` in `config/rrbot_mpc.yaml` define the mapping.

## 4. Reference input contract

- Topic: `~/reference` (resolved as `/mpc_controller/reference`).
- Type: `std_msgs/msg/Float64MultiArray`, joint-space reference arrays
  (positions and velocities as consumed by the controller), published by
  `scripts/trajectory_publisher.py` (sine trajectory for Quick Start).
- Delivery: realtime buffer (not direct access from the ROS callback) so
  `update()` never blocks on the subscription.
- Validation requirements (C1 target): dimension must match the controller
  state/reference layout; missing/stale/NaN/invalid-dimension references
  must not reach the command writer — the controller falls back to the
  last valid command or a deterministic safe command and reports it.

## 5. Output / observability

- Commands: `joint1/effort`, `joint2/effort`.
- Diagnostics topic: `~/diagnostics`
  (README: 32 fields per cycle for nx=4, nu=2, n_slack=2×20=40:
  solve/setup/cycle time, iterations, OSQP status, solved flags,
  state/reference/error, objective, residuals, …).  Diagnostics are the
  single source for runtime characterisation and failure classification
  (solver failure vs WSL2 slowdown vs DDS latency).

## 6. Timing contract

- `controller_manager.update_rate` = 100 Hz is the **target**.
- Measured cadence under WSL2 diagnostics ≈ 48–83 Hz; per-cycle
  `cycle_time_us`/`solve_time_us`/`setup_time_us` are recorded.
- Reports must present target-vs-measured, p95/max solve time, deadline
  miss, infeasible, fallback and NaN sentinel counts — never only the mean.

## 7. Safety invariants (to be enforced by tests in C1+)

1. A non-optimal QP outcome (infeasible/timeout/unacceptable residual)
   never writes an unvalidated or stale command.
2. NaN in state/reference/solution never propagates to the command
   interface (NaN sentinel tracked in benchmarks: not observed so far).
3. Only the controller writes the effort command interface.
4. Lifecycle: reference missing, upstream shutdown or deactivation leaves
   the interface unwritten (or at a deterministic safe value).

## 8. Companion documents

- `repo_snapshot_2026-09-03.md` — repository/environment/baseline numbers.
- `model_and_qp_derivation.md` — model and QP derivation.
- `rl_mdp_reward_design.md` (future) — residual-PPO MDP contract, alpha=0
  equivalence to pure MPC.
