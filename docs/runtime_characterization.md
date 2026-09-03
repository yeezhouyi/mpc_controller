# Runtime characterization — RRBot MPC (C2)

Data: `records/c0_repro_2026-09-03` (25 s) and `records/c0_formal_5x65_20260903_232655`
(5x65 s).  Metrics read from `/mpc_controller/diagnostics`.

## Target vs measured

| metric | target | measured (5-run) | note |
|---|---|---|---|
| controller update rate | 100 Hz | 44-66 Hz diagnostics | WSL2 scheduling (README 48-83 Hz) |
| solved rate | - | 97.6-100% | status -2 bursts on load |
| mean solve | <10 ms | 2.2-4.5 ms | |
| p95 solve | - | 4.9-8.8 ms | max 24-28 ms outliers |
| deadline miss | 0 | 3.9-33% | correlated with diagnostics-rate dips |

## Failure classification discipline (R12)

- solver failures: status -2/-3 with hold count (kept, not hidden);
- infrastructure: low diagnostics rate + high deadline miss on the same run
  (WSL2 load) -> classified separately from solver status;
- run5 diagnostics gap retained as an explicit failure sample (controller
  publishing lifecycle check = next C2 item);
- no run removed by a quality gate without reporting it.

## Status
Single-factor sweeps (horizon / Q/R/S / warm start / OSQP time limit) and
controller lifecycle tests are the next C2 work items; the capture +
aggregation tooling above is reusable.
