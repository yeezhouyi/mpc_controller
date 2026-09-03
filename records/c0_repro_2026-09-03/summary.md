# C0 reproduction — RRBot MPC single-run diagnostics (2026-09-03)

Environment: WSL2 Ubuntu 24.04 / ROS2 Jazzy / Gazebo Harmonic; repo commit `75f03d4`

Flow: `ros2 launch mpc_controller rrbot_mpc.launch.py` -> mpc_controller ACTIVE in 3 s (nx=4, nu=2, horizon=20, dt=0.010, soft velocity constraints rho1=100 rho2=10); recorded `/mpc_controller/diagnostics` for 25.2 s.

| metric | value |
|---|---:|
| messages | 1597 |
| diagnostics rate | 63.4 Hz |
| solved rate (status 1) | 97.68% |
| status histogram | {'-2': 31, '1': 1560, '2': 6} |
| mean solve time | 2586.9 us |
| p95 solve time | 5183.4 us |
| max solve time | 29043.0 us |
| deadline miss rate | 3.94% |
| hold (fallback) count | 37 |

Interpretation: numbers are consistent with the README v0.2.1 baseline (clean-run ~98% solved, solve times ~2.6-5.2 ms, WSL2 diagnostics rate 48-83 Hz).  This is a single-run reproduction; the formal 5x65 s protocol and per-run manifests are the next C0/C2 step.
