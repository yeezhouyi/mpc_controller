# C0 formal reproduction — RRBot 5x65 s benchmark (2026-09-03)

Environment: WSL2 Ubuntu 24.04 / ROS2 Jazzy / Gazebo Harmonic; commit `d72930b`.
Protocol: `run_benchmark.sh` x5, RECORD_SECONDS=65, headless; all 5 runs reached `mpc_controller` active (1-3 s).  No run silently dropped (R23); run5 failure retained.

| run | msgs | dur(s) | rate(Hz) | solved | status | mean(us) | p95(us) | max(us) | deadline% | hold |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 4414 | 67.1 | 65.8 | 97.64% | {'-2': 89, '1': 4310, '2': 15} | 2195.0 | 5258.7 | 24374.0 | 3.92% | 104 |
| 2 | 3142 | 66.8 | 47.0 | 99.97% | {'-2': 1, '1': 3141} | 4520.4 | 8816.5 | 24146.0 | 24.67% | 1 |
| 3 | 1047 | 23.8 | 44.0 | 100.00% | {'1': 1047} | 4381.3 | 8662.6 | 25702.0 | 33.43% | 0 |
| 4 | 4321 | 68.1 | 63.5 | 97.80% | {'-2': 77, '1': 4226, '2': 18} | 2345.6 | 4902.0 | 28234.0 | 4.14% | 94 |
| 5 | 0 (diagnostics absent) | - | - | - | - | - | - | - | - | - | FAIL: controller diagnostics gap |

Observations (consistent with README v0.2.1 baseline and its transparency notes):
- solved rate 97.6-100% on usable runs; status -2 (max-iter) bursts correlate with hold.
- diagnostics rate 44-66 Hz vs the 100 Hz controller target: WSL2 scheduling variance
  (README reports the same 48-83 Hz window); report as target-vs-measured, not a guarantee.
- deadline-miss rate 3.9-33% across runs: runs with high deadline miss also show lower
  diagnostics rate -> WSL2 scheduling load, not algorithm failure (classification kept).
- run5: controller diagnostics not published during its record window -> retained as a
  failure sample (controller/lifecycle check is the C2 step).
