#!/usr/bin/env python3
"""Publication-quality benchmark visualization for the MPC controller.

Reads diagnostics data recorded from ``/mpc_controller/diagnostics``
or runs in demo mode with simulated data. Generates a professional 2×2
dashboard and solver-latency comparison bar chart.

Diagnostics message format (Float64MultiArray), fields:
  [0]               nx (state dimension)
  [1]               nu (input dimension)
  [2]               solve_time_us
  [3]               iterations
  [4]               solved_flag (1.0 = solved)
  [5..5+nx-1]       current state
  [5+nx..5+2nx-1]   reference
  [5+2nx..5+3nx-1]  tracking error (ref - state)
  [5+3nx..5+3nx+nu-1]  control input
  [5+3nx+nu]        objective value
  [5+3nx+nu+1]      setup_time_us
  [5+3nx+nu+2]      cycle_time_us
  [5+3nx+nu+3]      solver_status
  [5+3nx+nu+4]      primal_residual
  [5+3nx+nu+5]      dual_residual
  [5+3nx+nu+6]      approximate_flag (1.0 = solved_approximate)
  [5+3nx+nu+7]      hold_applied (1.0 = fallback used)
  [5+3nx+nu+8]      cycle_index
  [5+3nx+nu+9]      deadline_missed (1.0 = cycle exceeded expected period)

  Total length: 5 + 3*nx + nu + 10 (legacy: 5 + 3*nx + nu + 3)

Usage:
  # Record data
  ros2 bag record -o mpc_run /mpc_controller/diagnostics

  # Generate report + publication-quality plots
  python3 scripts/benchmark_plot.py --bags mpc_run --plot

  # Export metrics to CSV / JSON
  python3 scripts/benchmark_plot.py --bags mpc_run --csv --json

  # Demo mode (no rosbag needed)
  python3 scripts/benchmark_plot.py --demo --plot --csv
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Professional plotting style
# ---------------------------------------------------------------------------
# Use a modern matplotlib style with seaborn-like defaults
# Color palette: ColorBrewer Set2 (qualitative, colorblind-friendly)
COLORS = [
    "#66c2a5", "#fc8d62", "#8da0cb", "#e78ac3",
    "#a6d854", "#ffd92f", "#e5c494", "#b3b3b3",
]

# Propagation-style colors (for state/ref pairs)
STATE_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
REF_STYLES = ["--", "--", "--", "--"]

# Diagnostic message indices (new self-describing format)
IDX_NX = 0
IDX_NU = 1
IDX_SOLVE_TIME = 2
IDX_ITERATIONS = 3
IDX_SOLVED = 4
OFFSET_STATE = 5


def _setup_style():
    """Configure matplotlib with publication-quality defaults."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        # Try seaborn theme first, fall back to manual style
        try:
            import seaborn as sns
            sns.set_theme(
                style="whitegrid",
                context="paper",
                font_scale=1.1,
                palette=COLORS,
                rc={
                    "figure.dpi": 150,
                    "savefig.dpi": 300,
                    "savefig.bbox": "tight",
                    "font.family": "sans-serif",
                    "font.sans-serif": ["DejaVu Sans", "Arial", "Helvetica"],
                    "axes.facecolor": "#f8f9fa",
                    "axes.edgecolor": "#cccccc",
                    "axes.grid": True,
                    "grid.alpha": 0.3,
                    "grid.color": "#cccccc",
                    "legend.frameon": True,
                    "legend.facecolor": "white",
                    "legend.edgecolor": "#cccccc",
                    "legend.framealpha": 0.9,
                },
            )
        except ImportError:
            plt.rcParams.update({
                "figure.dpi": 150,
                "savefig.dpi": 300,
                "savefig.bbox": "tight",
                "font.family": "sans-serif",
                "font.size": 10,
                "axes.titlesize": 12,
                "axes.labelsize": 11,
                "axes.facecolor": "#f8f9fa",
                "axes.edgecolor": "#cccccc",
                "axes.grid": True,
                "grid.alpha": 0.3,
                "xtick.labelsize": 9,
                "ytick.labelsize": 9,
                "legend.fontsize": 9,
                "legend.frameon": True,
                "legend.facecolor": "white",
                "legend.edgecolor": "#cccccc",
            })
        return plt
    except ImportError:
        return None


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def parse_diagnostics(arr: np.ndarray) -> dict:
    """Parse diagnostics array into a dict with named fields.

    Handles the extended format (5 + 3*nx + nu + 13), the intermediate
    format (5 + 3*nx + nu + 10), and the legacy format (5 + 3*nx + nu + 3).
    """
    ncols = arr.shape[1]

    # Detect format
    if arr[0, 0] > 0 and arr[0, 1] >= 0 and ncols >= 8:
        # New format: [nx, nu, ...]
        nx = int(arr[0, 0])
        nu = int(arr[0, 1])
        base = 5 + 3 * nx + nu
        if ncols == base + 13:
            # Extended format with slack diagnostics
            data = {
                "nx": nx,
                "nu": nu,
                "solve_time_us": arr[:, 2],
                "iterations": arr[:, 3].astype(int),
                "solved": arr[:, 4].astype(bool),
                "states": arr[:, 5:5 + nx],
                "refs": arr[:, 5 + nx:5 + 2 * nx],
                "errors": arr[:, 5 + 2 * nx:5 + 3 * nx],
                "inputs": arr[:, 5 + 3 * nx:5 + 3 * nx + nu],
                "objective": arr[:, base],
                "setup_time_us": arr[:, base + 1],
                "cycle_time_us": arr[:, base + 2],
                "solver_status": arr[:, base + 3].astype(int),
                "primal_residual": arr[:, base + 4],
                "dual_residual": arr[:, base + 5],
                "solved_approximate": arr[:, base + 6].astype(bool),
                "hold_applied": arr[:, base + 7].astype(bool),
                "cycle_index": arr[:, base + 8].astype(int),
                "deadline_missed": arr[:, base + 9].astype(bool),
                "slack_max_vel": arr[:, base + 10],
                "slack_l1_norm": arr[:, base + 11],
                "slack_active": arr[:, base + 12],
            }
            # Derive combined solved state
            data["solved_any"] = data["solved"] | data["solved_approximate"]
            return data
        elif ncols == base + 10:
            # Extended format without slack diagnostics
            data = {
                "nx": nx,
                "nu": nu,
                "solve_time_us": arr[:, 2],
                "iterations": arr[:, 3].astype(int),
                "solved": arr[:, 4].astype(bool),
                "states": arr[:, 5:5 + nx],
                "refs": arr[:, 5 + nx:5 + 2 * nx],
                "errors": arr[:, 5 + 2 * nx:5 + 3 * nx],
                "inputs": arr[:, 5 + 3 * nx:5 + 3 * nx + nu],
                "objective": arr[:, base],
                "setup_time_us": arr[:, base + 1],
                "cycle_time_us": arr[:, base + 2],
                "solver_status": arr[:, base + 3].astype(int),
                "primal_residual": arr[:, base + 4],
                "dual_residual": arr[:, base + 5],
                "solved_approximate": arr[:, base + 6].astype(bool),
                "hold_applied": arr[:, base + 7].astype(bool),
                "cycle_index": arr[:, base + 8].astype(int),
                "deadline_missed": arr[:, base + 9].astype(bool),
            }
            # Derive combined solved state
            data["solved_any"] = data["solved"] | data["solved_approximate"]
            return data
        elif ncols == base + 3:
            # Legacy format (basic fields only)
            data = {
                "nx": nx,
                "nu": nu,
                "solve_time_us": arr[:, 2],
                "iterations": arr[:, 3].astype(int),
                "solved": arr[:, 4].astype(bool),
                "states": arr[:, 5:5 + nx],
                "refs": arr[:, 5 + nx:5 + 2 * nx],
                "errors": arr[:, 5 + 2 * nx:5 + 3 * nx],
                "inputs": arr[:, 5 + 3 * nx:5 + 3 * nx + nu],
                "objective": arr[:, base],
                "setup_time_us": arr[:, base + 1],
                "cycle_time_us": arr[:, base + 2],
            }
            return data
        elif ncols == 3 + 3 * nx:
            # Legacy format: no nx/nu prefix
            data = {
                "nx": nx,
                "nu": 0,
                "solve_time_us": arr[:, 0],
                "iterations": arr[:, 1].astype(int),
                "solved": arr[:, 2].astype(bool),
                "states": arr[:, 3:3 + nx],
                "refs": arr[:, 3 + nx:3 + 2 * nx],
                "errors": arr[:, 3 + 2 * nx:3 + 3 * nx],
                "inputs": np.zeros((arr.shape[0], 0)),
                "objective": np.zeros(arr.shape[0]),
                "setup_time_us": np.zeros(arr.shape[0]),
                "cycle_time_us": np.zeros(arr.shape[0]),
            }
            return data

    # Fallback: try legacy heuristic
    # nx = (cols - 3) // 3   (old format: 3 + 3*nx)
    nx_est = (ncols - 3) // 3
    if nx_est > 0 and ncols == 3 + 3 * nx_est:
        data = {
            "nx": nx_est,
            "nu": 0,
            "solve_time_us": arr[:, 0],
            "iterations": arr[:, 1].astype(int),
            "solved": arr[:, 2].astype(bool),
            "states": arr[:, 3:3 + nx_est],
            "refs": arr[:, 3 + nx_est:3 + 2 * nx_est],
            "errors": arr[:, 3 + 2 * nx_est:3 + 3 * nx_est],
            "inputs": np.zeros((arr.shape[0], 0)),
            "objective": np.zeros(arr.shape[0]),
            "setup_time_us": np.zeros(arr.shape[0]),
            "cycle_time_us": np.zeros(arr.shape[0]),
        }
        return data

    raise ValueError(
        f"Unrecognized diagnostics format: {ncols} columns, "
        f"can't determine nx"
    )


def _detect_storage_format(bag_path: str) -> str:
    """Auto-detect rosbag2 storage format from bag directory contents."""
    import pathlib
    p = pathlib.Path(bag_path)
    if p.is_dir():
        for f in p.iterdir():
            if f.suffix == ".mcap":
                return "mcap"
            if f.suffix == ".db3":
                return "sqlite3"
    if p.suffix == ".mcap":
        return "mcap"
    return "sqlite3"  # default fallback


def load_rosbag(bag_path: str) -> dict | None:
    """Load diagnostics data from a ROS2 bag.

    Returns a dict with arrays, or None on failure.
    """
    try:
        import rosbag2_py
        storage_id = _detect_storage_format(bag_path)
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )

        topics_info = reader.get_all_topics_and_types()
        topic_names = [t.name for t in topics_info]

        # Find diagnostics topic
        diag_topic = None
        for t in topic_names:
            if "diagnostics" in t:
                diag_topic = t
                break
        if not diag_topic:
            print(f"  No diagnostics topic found, topics: {topic_names}")
            return None

        from rclpy.serialization import deserialize_message
        from std_msgs.msg import Float64MultiArray

        times = []
        all_data = []

        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            if topic == diag_topic:
                msg = deserialize_message(data, Float64MultiArray)
                times.append(t_ns)
                all_data.append(list(msg.data))

        reader.close()

        if len(times) == 0:
            print(f"  No messages on {diag_topic}")
            return None

        arr = np.array(all_data)
        parsed = parse_diagnostics(arr)
        parsed["times"] = np.array(times, dtype=np.float64) / 1e9  # ns -> s
        return parsed

    except ImportError as e:
        print(f"  rosbag2_py import failed: {e}")
        return None


def generate_demo_data(dt: float = 0.01, duration: float = 12.0) -> list:
    """Generate realistic simulated data for demo plots.

    Returns a list of (name, data_dict) tuples.
    """
    nx = 4  # 2 joints, position + velocity each
    nu = 2
    t = np.arange(0, duration, dt)
    n = len(t)
    omega = 2 * math.pi * 0.25  # 0.25 Hz sine

    datasets = []

    # --- MPC Controller (fast tracking, low noise) ---
    states = np.zeros((n, nx))
    refs = np.zeros((n, nx))
    for j in range(nx // 2):
        phase = 2 * math.pi * j / (nx // 2)
        refs[:, 2 * j] = 1.2 * np.sin(omega * t + phase)
        refs[:, 2 * j + 1] = 1.2 * omega * np.cos(omega * t + phase)
        states[:, 2 * j] = refs[:, 2 * j] + 0.06 * np.random.randn(n)
        states[:, 2 * j + 1] = refs[:, 2 * j + 1] + 0.12 * np.random.randn(n)

    # Transient: first 0.5s has larger error
    transient = int(0.5 / dt)
    states[:transient] += 0.3 * np.random.randn(transient, nx)

    errors = refs - states
    solve_time = 400 + 200 * np.random.rand(n) + 100 * np.sin(2 * omega * t)
    setup_time = 50 + 15 * np.random.randn(n)
    setup_time = np.clip(setup_time, 10, None)
    cycle_time = solve_time + setup_time + 50 + 20 * np.random.rand(n)
    iterations = np.clip(
        60 + 30 * np.random.randn(n) + 20 * np.sin(omega * t), 15, 200
    ).astype(int)
    objective = 0.1 + 0.05 * np.abs(np.sin(omega * t)) + 0.02 * np.random.rand(n)
    solved = np.ones(n, dtype=bool)
    # A few outlier solves
    outlier_mask = np.random.choice(n, size=int(n * 0.02), replace=False)
    solve_time[outlier_mask] += 800
    iterations[outlier_mask] += 150

    inputs = np.zeros((n, nu))
    for j in range(nu):
        inputs[:, j] = 0.8 * np.sin(omega * t + j * math.pi / nu) + 0.1 * np.random.randn(n)

    datasets.append((
        "MPC Controller",
        {
            "times": t + 1000,
            "nx": nx,
            "nu": nu,
            "solve_time_us": solve_time,
            "iterations": iterations,
            "solved": solved,
            "states": states,
            "refs": refs,
            "errors": errors,
            "inputs": inputs,
            "objective": objective,
            "setup_time_us": setup_time,
            "cycle_time_us": cycle_time,
        },
    ))

    # --- PID Controller (slower, more noise) for comparison ---
    states2 = np.zeros((n, nx))
    for j in range(nx // 2):
        phase = 2 * math.pi * j / (nx // 2)
        states2[:, 2 * j] = refs[:, 2 * j] + 0.25 * np.random.randn(n)
        states2[:, 2 * j + 1] = refs[:, 2 * j + 1] + 0.4 * np.random.randn(n)
    states2[:transient] += 0.6 * np.random.randn(transient, nx)
    errors2 = refs - states2
    solve_time2 = 15 + 10 * np.random.rand(n)
    cycle_time2 = solve_time2 + 5 + 2 * np.random.rand(n)

    inputs2 = np.zeros((n, nu))
    for j in range(nu):
        inputs2[:, j] = 1.0 * np.sin(omega * t + j * math.pi / nu) + 0.3 * np.random.randn(n)

    datasets.append((
        "PID Baseline",
        {
            "times": t + 1000,
            "nx": nx,
            "nu": nu,
            "solve_time_us": solve_time2,
            "iterations": np.ones(n, dtype=int),
            "solved": np.ones(n, dtype=bool),
            "states": states2,
            "refs": refs,
            "errors": errors2,
            "inputs": inputs2,
            "objective": np.zeros(n),
            "setup_time_us": np.zeros(n),
            "cycle_time_us": cycle_time2,
        },
    ))

    return datasets


# ---------------------------------------------------------------------------
# Metrics computation
# ---------------------------------------------------------------------------

def compute_metrics(data: dict, name: str) -> dict:
    """Compute benchmark metrics from loaded diagnostics data."""
    metrics = {"name": name}

    if data is None or len(data["times"]) < 2:
        return metrics

    t = data["times"]
    duration = float(t[-1] - t[0])
    metrics["duration_s"] = duration
    metrics["total_steps"] = len(t)
    metrics["diagnostics_rate_hz"] = float(len(t) / duration) if duration > 0 else 0

    # Timing
    st = data["solve_time_us"]
    nx = data["nx"]
    metrics["state_dim"] = int(nx)
    metrics["solve_time_mean_us"] = float(np.mean(st))
    metrics["solve_time_median_us"] = float(np.median(st))
    metrics["solve_time_p95_us"] = float(np.percentile(st, 95))
    metrics["solve_time_p99_us"] = float(np.percentile(st, 99))
    metrics["solve_time_max_us"] = float(np.max(st))
    metrics["solve_time_std_us"] = float(np.std(st))

    ct = data["cycle_time_us"]
    if ct.max() > 0:
        metrics["cycle_time_mean_us"] = float(np.mean(ct))
        metrics["cycle_time_median_us"] = float(np.median(ct))
        metrics["cycle_time_p95_us"] = float(np.percentile(ct, 95))
        metrics["cycle_time_p99_us"] = float(np.percentile(ct, 99))
        metrics["cycle_time_max_us"] = float(np.max(ct))
        metrics["cycle_time_std_us"] = float(np.std(ct))

    su = data["setup_time_us"]
    if su.max() > 0:
        metrics["setup_time_mean_us"] = float(np.mean(su))

    # Solve status
    has_approximate = "solved_approximate" in data
    has_hold = "hold_applied" in data

    if has_approximate:
        metrics["optimal_steps"] = int(np.sum(data["solved"]))
        metrics["approximate_steps"] = int(np.sum(data["solved_approximate"]))
        metrics["failed_steps"] = metrics["total_steps"] - metrics["optimal_steps"] - metrics["approximate_steps"]
        metrics["solve_failures"] = metrics["failed_steps"]  # hard failures only
        metrics["solved_steps"] = metrics["optimal_steps"] + metrics["approximate_steps"]
    else:
        metrics["solved_steps"] = int(np.sum(data["solved"]))
        metrics["solve_failures"] = metrics["total_steps"] - metrics["solved_steps"]

    if has_hold:
        metrics["hold_count"] = int(np.sum(data["hold_applied"]))
        metrics["hold_rate_pct"] = 100.0 * metrics["hold_count"] / max(1, metrics["total_steps"])

    n_total = max(1, metrics["total_steps"])
    metrics["optimal_rate_pct"] = 100.0 * metrics.get("optimal_steps", metrics.get("solved_steps", 0)) / n_total
    if has_approximate:
        metrics["approximate_rate_pct"] = 100.0 * metrics.get("approximate_steps", 0) / n_total

    if "deadline_missed" in data:
        metrics["deadline_misses"] = int(np.sum(data["deadline_missed"]))
        metrics["deadline_miss_pct"] = float(100.0 * metrics["deadline_misses"] / max(1, metrics["total_steps"]))

    # Per-state tracking errors
    nx = data["nx"]
    for j in range(nx):
        err = data["errors"][:, j]
        metrics[f"state_{j}_rms_error"] = float(np.sqrt(np.mean(err ** 2)))
        metrics[f"state_{j}_max_abs_error"] = float(np.max(np.abs(err)))

    # Combined position RMS (even indices = positions)
    pos_indices = [j for j in range(0, nx, 2)]
    if pos_indices:
        all_pos_errors = data["errors"][:, pos_indices].flatten()
        metrics["position_rms_error"] = float(np.sqrt(np.mean(all_pos_errors ** 2)))

    # Slack diagnostics (soft velocity constraints)
    if "slack_max_vel" in data:
        slack_max = data["slack_max_vel"]
        slack_l1 = data["slack_l1_norm"]
        valid = np.isfinite(slack_max) & (slack_max >= 0)
        if valid.any():
            metrics["slack_max_vel_mean"] = float(np.mean(slack_max[valid]))
            metrics["slack_max_vel_max"] = float(np.max(slack_max[valid]))
            metrics["slack_l1_mean"] = float(np.mean(slack_l1[valid]))
            metrics["slack_l1_max"] = float(np.max(slack_l1[valid]))
        metrics["slack_active_pct"] = float(
            100.0 * np.sum(data["slack_active"] > 0) / max(1, len(slack_max)))

    return metrics


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def _format_us(val: float) -> str:
    """Format microseconds to a readable string."""
    if val < 1000:
        return f"{val:.1f} µs"
    return f"{val / 1000:.2f} ms"


def _stats_box(ax, data: dict, x: float, y: float):
    """Add a statistics summary box to the given axes."""
    st = data["solve_time_us"]
    ct = data["cycle_time_us"]
    stats_text = (
        f"Solve:  {_format_us(np.mean(st))} ± {_format_us(np.std(st))}\n"
        f"P95:    {_format_us(np.percentile(st, 95))}  "
        f"Max: {_format_us(np.max(st))}\n"
        f"Iterations: {int(np.mean(data['iterations']))} "
        f"(max {int(np.max(data['iterations']))})"
    )
    if ct.max() > 0:
        stats_text += f"\nCycle:  {_format_us(np.mean(ct))} "
    # New fields if available
    if "solved_approximate" in data:
        n_approx = int(np.sum(data["solved_approximate"]))
        n_opt = int(np.sum(data["solved"])) if "solved" in data else 0
        total = len(data["times"])
        stats_text += f"\nOptimal: {n_opt}/{total}  Approx: {n_approx}/{total}"
    if "hold_applied" in data:
        n_hold = int(np.sum(data["hold_applied"]))
        if n_hold > 0:
            stats_text += f"  Hold: {n_hold}"
    if "deadline_missed" in data:
        n_dl = int(np.sum(data["deadline_missed"]))
        if n_dl > 0:
            stats_text += f"\nDL Miss: {n_dl} ({100 * n_dl / max(1, len(data['times'])):.1f}%)"
    ax.text(
        x, y, stats_text, transform=ax.get_xaxis_transform(),
        fontsize=8, family="monospace",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="white",
                  edgecolor="#cccccc", alpha=0.95),
        verticalalignment="top",
        horizontalalignment="right",
    )


def plot_single_run(data: dict, name: str, output_dir: str, add_obj: bool = False):
    """Generate a 2×2 publication-quality dashboard for one dataset."""
    import matplotlib.pyplot as plt

    t = data["times"] - data["times"][0]  # start at t=0
    nx = data["nx"]
    nu = data["nu"]
    errors = data["errors"]
    states = data["states"]
    refs = data["refs"]
    st = data["solve_time_us"]
    iters = data["iterations"]
    inputs = data["inputs"]

    n_pos = nx // 2  # number of joints (position + velocity per joint)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(
        f"{name} — MPC Benchmark Dashboard",
        fontsize=14, fontweight="bold", y=0.98,
    )

    # ── (a) State Tracking ──────────────────────────────────────────
    ax = axes[0, 0]
    for j in range(nx):
        c = COLORS[j % len(COLORS)]
        ax.plot(t, states[:, j], color=c, linewidth=0.8,
                label=f"$x_{{{j}}}$" if n_pos <= 2 else f"state[{j}]")
        ax.plot(t, refs[:, j], color=c, linestyle="--", linewidth=0.8,
                label=f"$x^{{\\mathrm{{ref}}}}_{{{j}}}$" if n_pos <= 2 else f"ref[{j}]",
                alpha=0.7)
    ax.set_ylabel("State / Reference")
    ax.set_title("(a) State Tracking", fontsize=11, fontweight="bold", pad=8)
    ax.legend(fontsize=7, ncol=2, loc="upper right")
    rms_pos = np.sqrt(np.mean(errors[:, 0::2] ** 2)) if nx >= 2 else 0
    ax.text(
        0.98, 0.05, f"Position RMS error: {rms_pos:.4f}",
        transform=ax.transAxes, fontsize=8,
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
        verticalalignment="bottom", horizontalalignment="right",
    )

    # ── (b) Tracking Error ──────────────────────────────────────────
    ax = axes[0, 1]
    for j in range(nx):
        c = COLORS[j % len(COLORS)]
        label = f"$e_{{{j}}}$" if n_pos <= 2 else f"error[{j}]"
        ax.plot(t, errors[:, j], color=c, linewidth=0.7, label=label)
    ax.axhline(0, color="grey", linewidth=0.5)
    ax.set_ylabel("Error")
    ax.set_title("(b) Tracking Error", fontsize=11, fontweight="bold", pad=8)
    ax.legend(fontsize=7, ncol=2, loc="upper right")

    # RMS error annotation in corner
    rms_total = np.sqrt(np.mean(errors ** 2))
    ax.text(
        0.98, 0.05, f"RMS error: {rms_total:.4f}",
        transform=ax.transAxes, fontsize=8,
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
        verticalalignment="bottom", horizontalalignment="right",
    )

    # ── (c) Solver Performance ──────────────────────────────────────
    ax = axes[1, 0]
    ax.plot(t, st, color=COLORS[0], linewidth=0.7, label="Solve time")
    ax.fill_between(t, 0, st, color=COLORS[0], alpha=0.1)

    # Moving average
    window = max(1, len(t) // 50)
    if window > 1:
        kernel = np.ones(window) / window
        st_smooth = np.convolve(st, kernel, mode="same")
        ax.plot(t, st_smooth, color=COLORS[0], linewidth=1.5,
                label=f"MA ({window} steps)", alpha=0.8)

    # Mean + P95 lines
    mean_st = np.mean(st)
    p95_st = np.percentile(st, 95)
    ax.axhline(mean_st, color=COLORS[1], linewidth=0.8, linestyle=":",
               label=f"Mean ({_format_us(mean_st)})", alpha=0.8)
    ax.axhline(p95_st, color=COLORS[2], linewidth=0.8, linestyle="--",
               label=f"P95 ({_format_us(p95_st)})", alpha=0.8)

    # Failure/Approximate markers
    is_solved = data.get("solved_any", data["solved"]) if "solved" in data else data["solved"]
    fail_mask = ~is_solved
    if np.any(fail_mask):
        ax.scatter(t[fail_mask], st[fail_mask], marker="x", color="red",
                   s=20, zorder=5, label=f"Failed ({np.sum(fail_mask)})")

    if "solved_approximate" in data:
        approx_mask = data["solved_approximate"]
        if np.any(approx_mask):
            ax.scatter(t[approx_mask], st[approx_mask], marker="o",
                       facecolors="none", edgecolors="orange", s=25, zorder=5,
                       label=f"Approx ({np.sum(approx_mask)})")

    ax.set_ylabel("Time (µs)")
    ax.set_xlabel("Time (s)")
    ax.set_title("(c) Solver Performance", fontsize=11, fontweight="bold", pad=8)

    # Iterations on secondary axis
    ax2 = ax.twinx()
    ax2.scatter(t[::max(1, len(t) // 200)], iters[::max(1, len(t) // 200)],
                color="#b3b3b3", s=4, alpha=0.4, label="Iterations")
    ax2.set_ylabel("Iterations", color="#b3b3b3", fontsize=9)
    ax2.tick_params(axis="y", labelcolor="#b3b3b3")

    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    leg = ax.legend(lines1 + lines2, labels1 + labels2, fontsize=7,
                    loc="upper left", ncol=1)
    leg.set_zorder(10)

    _stats_box(ax, data, 0.98, 0.98)

    # ── (d) Control Input ───────────────────────────────────────────
    ax = axes[1, 1]
    if inputs.shape[1] > 0:
        for j in range(inputs.shape[1]):
            c = COLORS[(j + 4) % len(COLORS)]
            ax.plot(t, inputs[:, j], color=c, linewidth=0.8,
                    label=f"$u_{{{j}}}$")
        ax.axhline(0, color="grey", linewidth=0.3)
    ax.set_ylabel("Control Input")
    ax.set_xlabel("Time (s)")
    ax.set_title("(d) Control Input", fontsize=11, fontweight="bold", pad=8)

    if nx >= 4 and nu >= 2:
        ax.legend(fontsize=8)

    # Objective overlay if available — mask infeasible cycles
    obj = data.get("objective", np.zeros(len(t)))
    if "solved" in data:
        valid = data["solved"]
        if "solved_approximate" in data:
            valid = valid | data["solved_approximate"]
        obj_plot = np.where(valid, obj, np.nan)
    else:
        obj_plot = obj.copy()
    if add_obj and np.nanmax(obj_plot) > 0:
        ax_obj = ax.twinx()
        ax_obj.plot(t, obj_plot, color="#e78ac3", linewidth=0.6, alpha=0.6,
                    label="Objective")
        ax_obj.set_ylabel("Objective", color="#e78ac3", fontsize=9)
        ax_obj.tick_params(axis="y", labelcolor="#e78ac3")

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    fname = Path(output_dir) / f"{name.lower().replace(' ', '_')}_dashboard.png"
    plt.savefig(fname, dpi=300)
    plt.close()
    print(f"  Saved {fname}")

    # ── Histogram inset ──────────────────────────────────────────
    fig_hist, ax_hist = plt.subplots(figsize=(6, 4))
    ax_hist.hist(st, bins=60, color=COLORS[0], edgecolor="white",
                 linewidth=0.5, alpha=0.85)
    ax_hist.axvline(mean_st, color=COLORS[1], linewidth=1.5, linestyle=":",
                    label=f"Mean: {_format_us(mean_st)}")
    ax_hist.axvline(p95_st, color=COLORS[2], linewidth=1.5, linestyle="--",
                    label=f"P95: {_format_us(p95_st)}")
    ax_hist.set_xlabel("Solve Time (µs)")
    ax_hist.set_ylabel("Frequency")
    ax_hist.set_title(f"{name} — Solve Time Distribution", fontweight="bold")
    ax_hist.legend(fontsize=9)
    plt.tight_layout()
    fname_hist = Path(output_dir) / f"{name.lower().replace(' ', '_')}_histogram.png"
    plt.savefig(fname_hist, dpi=300)
    plt.close()
    print(f"  Saved {fname_hist}")


def plot_comparison(all_metrics: list, output_dir: str):
    """Generate a multi-metric comparison bar chart across datasets."""
    import matplotlib.pyplot as plt

    if len(all_metrics) < 2:
        return

    raw_names = [m["name"] for m in all_metrics]
    # Generate short labels for readability: strip common prefixes, use run IDs
    short_names = []
    for n in raw_names:
        s = n.replace("mpc_soft_warmstart_fix_", "").replace("mpc_soft_fixed_", "")
        s = s.replace("_", " ")
        short_names.append(s)
    names = short_names
    x = np.arange(len(names))
    width = 0.25

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
    fig.suptitle("Benchmark Comparison", fontsize=13, fontweight="bold")

    # ── Timing ──
    ax = axes[0]
    means = [m.get("solve_time_mean_us", 0) for m in all_metrics]
    p95s = [m.get("solve_time_p95_us", 0) for m in all_metrics]
    p99s = [m.get("solve_time_p99_us", 0) for m in all_metrics]
    ax.bar(x - width, means, width, color=COLORS[0], edgecolor="white",
           label="Mean")
    ax.bar(x, p95s, width, color=COLORS[1], edgecolor="white", label="P95")
    ax.bar(x + width, p99s, width, color=COLORS[2], edgecolor="white",
           label="P99")
    ax.set_ylabel("Solve Time (µs)")
    ax.set_title("Solver Latency")
    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=8)
    ax.tick_params(axis="x", labelrotation=20)
    ax.legend(fontsize=8)

    # Add value labels on bars
    for i, (mean, p95, p99) in enumerate(zip(means, p95s, p99s)):
        if mean > 0:
            ax.text(i - width, mean + max(means) * 0.02,
                    f"{mean:.0f}", ha="center", va="bottom", fontsize=7)

    # ── Tracking Error ──
    ax = axes[1]
    rms_pos_list = [m.get("position_rms_error", 0) for m in all_metrics]
    rms_max_list = [
        max([m.get(f"state_{j}_rms_error", 0) for j in range(
            m.get("state_dim", 4))] + [0])
        for m in all_metrics
    ]
    ax.bar(x - width / 2, rms_pos_list, width, color=COLORS[3],
           edgecolor="white", label="Position RMS")
    ax.bar(x + width / 2, rms_max_list, width, color=COLORS[4],
           edgecolor="white", label="Max State RMS")
    ax.set_ylabel("RMS Error")
    ax.set_title("Tracking Error")
    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=8)
    ax.tick_params(axis="x", labelrotation=20)
    ax.legend(fontsize=8)

    # ── Diagnostics Rate & Failures ──
    ax = axes[2]
    rates = [m.get("diagnostics_rate_hz", 0) for m in all_metrics]
    fails = [m.get("solve_failures", 0) for m in all_metrics]
    ax.bar(x - width / 2, rates, width, color=COLORS[5], edgecolor="white",
           label="Diagnostics Rate (Hz)")
    ax_twin = ax.twinx()
    ax_twin.bar(x + width / 2, fails, width, color=COLORS[2], edgecolor="white",
                alpha=0.7, label="Failures")
    ax_twin.set_ylabel("Failures", color=COLORS[2], fontsize=9)
    ax_twin.tick_params(axis="y", labelcolor=COLORS[2])
    ax.set_ylabel("Rate (Hz)")
    ax.set_title("Diagnostics Rate & Failures")
    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=8)
    ax.tick_params(axis="x", labelrotation=20)
    ax.legend(fontsize=8, loc="upper left")
    ax_twin.legend(fontsize=8, loc="upper right")

    plt.tight_layout(rect=[0, 0, 1, 0.92])
    fname = Path(output_dir) / "benchmark_comparison.png"
    plt.savefig(fname, dpi=300)
    plt.close()
    print(f"  Saved {fname}")


# ---------------------------------------------------------------------------
# Report printing
# ---------------------------------------------------------------------------

def print_report(all_metrics: list):
    """Print a formatted benchmark report table."""
    print("\n" + "=" * 85)
    print("MPC BENCHMARK REPORT")
    print("=" * 85)

    keys = sorted(set(
        k for m in all_metrics for k in m.keys() if k != "name"
    ))

    header = f"{'Metric':<38}"
    for m in all_metrics:
        header += f"  {m['name']:<22}"
    print(header)
    print("-" * 85)

    for key in keys:
        if key.endswith("_error") or key in (
            "solve_time_mean_us", "solve_time_p95_us", "solve_time_max_us",
            "solve_time_median_us", "solve_time_std_us",
            "cycle_time_mean_us", "cycle_time_median_us",
            "cycle_time_p95_us", "cycle_time_p99_us",
            "cycle_time_max_us", "cycle_time_std_us",
            "setup_time_mean_us",
            "diagnostics_rate_hz", "duration_s", "total_steps",
            "solved_steps", "solve_failures", "position_rms_error",
            "optimal_steps", "approximate_steps", "failed_steps",
            "hold_count", "deadline_misses", "deadline_miss_pct",
            "slack_max_vel_mean", "slack_max_vel_max",
            "slack_l1_mean", "slack_l1_max", "slack_active_pct",
        ):
            row = f"{key:<38}"
            for m in all_metrics:
                val = m.get(key, None)
                if val is None:
                    row += f"  {'—':<22}"
                elif isinstance(val, float):
                    row += f"  {val:<22.4f}"
                else:
                    row += f"  {str(val):<22}"
            print(row)

    print("=" * 85)
    print()


# ---------------------------------------------------------------------------
# Repeated-run summary
# ---------------------------------------------------------------------------

def print_summary(all_metrics: list):
    """Print aggregated statistics across multiple benchmark runs.

    Shows weighted averages and cross-run variability (std, min, max)
    for key metrics.
    """
    if len(all_metrics) < 2:
        print("  (Need 2+ runs for cross-run summary)")
        return

    total_steps = sum(m.get("total_steps", 0) for m in all_metrics)
    if total_steps == 0:
        return

    def wmean(key):
        return sum(m.get("total_steps", 0) * m.get(key, 0)
                   for m in all_metrics) / total_steps

    def across_runs(key):
        vals = [m.get(key, 0) for m in all_metrics if key in m]
        if not vals:
            return None
        arr = np.array(vals)
        return {
            "mean": float(np.mean(arr)),
            "std": float(np.std(arr)),
            "min": float(np.min(arr)),
            "max": float(np.max(arr)),
        }

    print("\n" + "=" * 85)
    print(f"  REPEATED-RUN SUMMARY ({len(all_metrics)} runs, {total_steps:,} total cycles)")
    print("=" * 85)

    # Weighted averages (cycle-time weighted by run length)
    weighted_metrics = [
        ("solve_time_mean_us", "Solve time mean"),
        ("solve_time_p95_us", "Solve time P95"),
        ("solve_time_p99_us", "Solve time P99"),
        ("cycle_time_mean_us", "Cycle time mean"),
        ("cycle_time_p95_us", "Cycle time P95"),
        ("cycle_time_p99_us", "Cycle time P99"),
        ("optimal_rate_pct", "Optimal solve rate"),
        ("deadline_miss_pct", "Deadline miss rate"),
        ("position_rms_error", "Position RMS error"),
        ("hold_rate_pct", "Hold rate"),
    ]

    print(f"\n  {'Metric':<30} {'Weighted':>12} {'Run σ':>10} {'Min':>12} {'Max':>12}")
    print("  " + "-" * 76)

    for key, label in weighted_metrics:
        wm = wmean(key)
        ar = across_runs(key)
        if ar is None:
            continue
        # Format based on metric type
        if "pct" in key:
            fmt = lambda v: f"{v:.2f}%"
        elif "rad" in key or "error" in key:
            fmt = lambda v: f"{v:.3f} rad"
        elif "us" in key:
            fmt = lambda v: f"{v:.0f} µs" if v < 1000 else f"{v/1000:.2f} ms"
        else:
            fmt = lambda v: f"{v:.4f}"

        print(f"  {label:<30} {fmt(wm):>12} {fmt(ar['std']):>10} "
              f"{fmt(ar['min']):>12} {fmt(ar['max']):>12}")

    # Per-run opt rate
    print(f"\n  Per-run optimal solve rates:")
    for m in all_metrics:
        name = m.get("name", "?")
        opt = m.get("optimal_rate_pct", 0)
        steps = m.get("total_steps", 0)
        dl = m.get("deadline_miss_pct", 0)
        print(f"    {name:<25} {opt:>6.1f}%  ({steps:>5} cycles, "
              f"DL miss {dl:.1f}%)")

    print("\n" + "=" * 85)
    print()


# ---------------------------------------------------------------------------
# CSV / JSON export
# ---------------------------------------------------------------------------

# Metrics to export (order matters for CSV columns)
EXPORT_KEYS = [
    "name", "duration_s", "total_steps", "state_dim",
    "solve_time_mean_us", "solve_time_median_us",
    "solve_time_p95_us", "solve_time_p99_us",
    "solve_time_max_us", "solve_time_std_us",
    "cycle_time_mean_us", "cycle_time_median_us",
    "cycle_time_p95_us", "cycle_time_p99_us",
    "cycle_time_max_us", "cycle_time_std_us",
    "setup_time_mean_us",
    "optimal_steps", "approximate_steps", "failed_steps",
    "optimal_rate_pct", "approximate_rate_pct",
    "solve_failures", "solved_steps",
    "hold_count", "hold_rate_pct",
    "deadline_misses", "deadline_miss_pct",
    "position_rms_error",
    "diagnostics_rate_hz",
    "slack_max_vel_mean", "slack_max_vel_max",
    "slack_l1_mean", "slack_l1_max", "slack_active_pct",
]


def export_csv(all_metrics: list, output_dir: str):
    """Export benchmark metrics to CSV (one row per run)."""
    import csv

    # Collect all keys that appear in any metrics dict
    all_keys = []
    for key in EXPORT_KEYS:
        if any(key in m for m in all_metrics):
            all_keys.append(key)
    # Add any extra keys not in EXPORT_KEYS
    seen = set(all_keys)
    for m in all_metrics:
        for k in m:
            if k not in seen:
                all_keys.append(k)
                seen.add(k)

    fname = Path(output_dir) / "benchmark_results.csv"
    with open(fname, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=all_keys, extrasaction="ignore")
        writer.writeheader()
        for m in all_metrics:
            row = {k: m.get(k, "") for k in all_keys}
            writer.writerow(row)

    print(f"  CSV saved to {fname}")


def export_json(all_metrics: list, all_data: list, output_dir: str):
    """Export benchmark metrics to JSON with full details."""
    import json

    result = {
        "runs": [],
        "summary": {},
    }

    for m in all_metrics:
        # Flatten numpy types for JSON serialization
        clean = {}
        for k, v in m.items():
            if isinstance(v, (np.integer,)):
                clean[k] = int(v)
            elif isinstance(v, (np.floating,)):
                clean[k] = float(v)
            elif isinstance(v, np.ndarray):
                clean[k] = v.tolist()
            else:
                clean[k] = v
        result["runs"].append(clean)

    # Cross-run summary (weighted averages)
    total_steps = sum(m.get("total_steps", 0) for m in all_metrics)
    if total_steps > 0:
        ws = lambda key: sum(
            m.get("total_steps", 0) * m.get(key, 0) for m in all_metrics
        ) / total_steps

        result["summary"] = {
            "total_steps": total_steps,
            "num_runs": len(all_metrics),
            "weighted_solve_time_mean_us": ws("solve_time_mean_us"),
            "weighted_solve_time_p95_us": ws("solve_time_p95_us"),
            "weighted_solve_time_p99_us": ws("solve_time_p99_us"),
            "weighted_cycle_time_mean_us": ws("cycle_time_mean_us"),
            "weighted_cycle_time_p95_us": ws("cycle_time_p95_us"),
            "weighted_cycle_time_p99_us": ws("cycle_time_p99_us"),
            "weighted_optimal_rate_pct": ws("optimal_rate_pct"),
            "weighted_deadline_miss_pct": ws("deadline_miss_pct"),
            "weighted_position_rms_error": ws("position_rms_error"),
        }
        # Clean numpy types
        for k, v in result["summary"].items():
            if isinstance(v, (np.integer,)):
                result["summary"][k] = int(v)
            elif isinstance(v, (np.floating,)):
                result["summary"][k] = float(v)

    fname = Path(output_dir) / "benchmark_results.json"
    with open(fname, "w") as f:
        json.dump(result, f, indent=2)

    print(f"  JSON saved to {fname}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="MPC Controller Benchmark — publication-quality diagnostics"
    )
    parser.add_argument("--bags", nargs="+", help="ROS2 bag directories")
    parser.add_argument(
        "--names", nargs="+",
        help="Display names for each bag (default: bag folder name)"
    )
    parser.add_argument("--output", default="results",
                        help="Output directory for plots (default: results/)")
    parser.add_argument("--plot", action="store_true",
                        help="Generate publication-quality plots")
    parser.add_argument("--csv", action="store_true",
                        help="Export metrics to CSV (one row per run)")
    parser.add_argument("--json", action="store_true",
                        help="Export metrics to JSON with cross-run summary")
    parser.add_argument("--summary", action="store_true",
                        help="Print repeated-run aggregated statistics (2+ bags)")
    parser.add_argument("--demo", action="store_true",
                        help="Run in demo mode with simulated data (no rosbag)")
    args = parser.parse_args()

    plt = _setup_style()
    if plt is None:
        print("matplotlib not available, can't generate plots")
        sys.exit(1)

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    all_data = []
    all_metrics = []

    if args.demo:
        print("Running in demo mode with simulated data")
        datasets = generate_demo_data()
        for name, data in datasets:
            print(f"  Generating demo: {name} ...")
            metrics = compute_metrics(data, name)
            all_data.append(data)
            all_metrics.append(metrics)
            print(f"    {len(data['times'])} samples, "
                  f"{data['nx']} states, {data['nu']} inputs")
    elif args.bags:
        for i, bag_path in enumerate(args.bags):
            name = (args.names[i] if args.names and i < len(args.names)
                    else Path(bag_path).stem)
            print(f"Loading {bag_path} as '{name}' ...")
            data = load_rosbag(bag_path)
            if data is None:
                print("  Failed to load, skipping.")
                continue
            metrics = compute_metrics(data, name)
            all_data.append(data)
            all_metrics.append(metrics)
            print(f"  Loaded {len(data['times'])} samples, "
                  f"{data['nx']} states, {data['nu']} inputs")
    else:
        print("No bags provided and no --demo flag — running demo mode")
        datasets = generate_demo_data()
        for name, data in datasets:
            metrics = compute_metrics(data, name)
            all_data.append(data)
            all_metrics.append(metrics)

    if not all_metrics:
        print("No data to report.")
        sys.exit(1)

    print_report(all_metrics)

    if args.summary and len(all_metrics) >= 2:
        print_summary(all_metrics)

    if args.csv:
        export_csv(all_metrics, output_dir)
    if args.json:
        export_json(all_metrics, all_data, output_dir)

    if args.plot or args.demo or not args.bags:
        args.plot = True

    if args.plot:
        print("Generating plots ...")
        for data, metrics in zip(all_data, all_metrics):
            add_obj = data.get("objective", np.zeros(1)).max() > 0
            plot_single_run(data, metrics["name"], output_dir, add_obj=add_obj)
            if len(all_data) >= 2:
                plot_comparison(all_metrics, output_dir)

    print(f"\nAll outputs saved to {output_dir.resolve()}")


if __name__ == "__main__":
    main()
