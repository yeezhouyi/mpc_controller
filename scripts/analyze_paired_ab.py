#!/usr/bin/env python3
"""Analyze paired A/B benchmark results from 10 rosbag files.

Usage:
  python3 scripts/analyze_paired_ab.py

Output:
  - Paired comparison table (v0.2.0 A vs v0.2.1 B)
  - Per-run breakdown sorted by label
  - Per-pair delta analysis
"""

import re
import subprocess
import sys
from pathlib import Path

BAGS_DIR = Path(__file__).resolve().parent.parent / "records" / "paired_ab"
SCRIPT = Path(__file__).resolve().parent / "benchmark_plot.py"
WS_ROOT = Path(__file__).resolve().parent.parent.parent  # ~/ros2_ws

# All 10 labels in the alternating sequence:
# A1, B2, B3, A4, A5, B6, B7, A8, A9, B10
LABELS = ["A1", "B2", "B3", "A4", "A5", "B6", "B7", "A8", "A9", "B10"]

KEY_METRICS = [
    "total_steps",
    "optimal_steps",
    "failed_steps",
    "hold_count",
    "solve_time_mean_us",
    "solve_time_median_us",
    "solve_time_p95_us",
    "cycle_time_mean_us",
    "deadline_miss_pct",
    "position_rms_error",
    "diagnostics_rate_hz",
    "slack_active_pct",
]


def parse_metrics(text: str) -> dict:
    """Parse the MPI BENCHMARK REPORT output into a dict."""
    metrics = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("=") or line.startswith("-"):
            continue
        parts = line.split()
        if len(parts) >= 2:
            # First word is metric name (may have trailing colon)
            key = parts[0].rstrip(":")
            # Last word is the value
            val_str = parts[-1]
            try:
                if "." in val_str:
                    metrics[key] = float(val_str)
                else:
                    metrics[key] = int(val_str)
            except ValueError:
                pass
    return metrics


def run_analysis(label: str, bag_dir: Path) -> dict:
    """Run benchmark_plot.py on one bag and extract metrics."""
    cmd = [
        sys.executable, str(SCRIPT),
        "--bags", str(bag_dir),
        "--output", str(bag_dir.parent / f"{label}_report"),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    if result.returncode != 0:
        print(f"WARNING: {label} returned code {result.returncode}")
        print(result.stderr[:500])
    return parse_metrics(result.stdout)


def fmt(val, key=""):
    """Format a metric value nicely."""
    if isinstance(val, float):
        if "pct" in key or "rate" in key:
            return f"{val:.2f}"
        if "us" in key or "time" in key:
            if val > 1000:
                return f"{val/1000:.2f} ms"
            return f"{val:.0f} µs"
        if "error" in key and "rms" in key:
            return f"{val:.3f} rad"
        return f"{val:.4f}"
    return str(val)


def main():
    print("=" * 90)
    print("  Paired A/B Benchmark Analysis: v0.2.0 (A) vs v0.2.1-rc1 (B)")
    print("  Sequence: A1, B2, B3, A4, A5, B6, B7, A8, A9, B10")
    print("=" * 90)

    all_metrics = {}
    for label in LABELS:
        bag_dir = BAGS_DIR / f"{label}_bag"
        if not bag_dir.exists():
            print(f"  WARNING: {label}_bag not found at {bag_dir}")
            continue
        print(f"\n  Processing {label} ...", end=" ", flush=True)
        metrics = run_analysis(label, bag_dir)
        all_metrics[label] = metrics
        print(f"done ({metrics.get('total_steps', '?')} cycles)")

    # ── Per-run breakdown ────────────────────────────────────────
    print("\n\n" + "=" * 90)
    print("  Per-Run Breakdown")
    print("=" * 90)

    header = f"{'Label':>6} | {'Version':>10} | {'Cycles':>7} | {'Optimal':>9} | {'Fail':>5} | {'Hold':>5} | {'Solve μ':>9} | {'P95 μ':>7} | {'Cycle μ':>9} | {'Deadline':>9} | {'RMS':>8} | {'Hz':>7}"
    sep = "-" * len(header)
    print(sep)
    print(header)
    print(sep)

    for label in LABELS:
        m = all_metrics.get(label, {})
        version = "v0.2.0" if label.startswith("A") else "v0.2.1"
        total = m.get("total_steps", 0)
        optimal = m.get("optimal_steps", 0)
        failed = m.get("failed_steps", 0)
        hold = m.get("hold_count", 0)
        solve_mean = m.get("solve_time_mean_us", 0)
        solve_p95 = m.get("solve_time_p95_us", 0)
        cycle_mean = m.get("cycle_time_mean_us", 0)
        deadline = m.get("deadline_miss_pct", 0)
        rms = m.get("position_rms_error", 0)
        hz = m.get("diagnostics_rate_hz", 0)

        print(
            f"{label:>6} | {version:>10} | {total:>7} | "
            f"{optimal:>7} ({optimal/total*100:>5.1f}%) | "
            f"{failed:>5} | {hold:>5} | "
            f"{solve_mean/1000:>7.2f}ms | "
            f"{solve_p95/1000:>6.2f}ms | "
            f"{cycle_mean/1000:>8.2f}ms | "
            f"{deadline:>7.2f}% | "
            f"{rms:>6.3f} | "
            f"{hz:>6.1f}"
        )

    print(sep)

    # ── Group statistics ─────────────────────────────────────────
    print("\n\n" + "=" * 90)
    print("  Group Statistics: v0.2.0 (A runs) vs v0.2.1-rc1 (B runs)")
    print("=" * 90)

    a_labels = [l for l in LABELS if l.startswith("A")]
    b_labels = [l for l in LABELS if l.startswith("B")]

    def group_stats(labels_subset):
        """Compute aggregate statistics for a group of runs."""
        total_cycles = sum(all_metrics[l].get("total_steps", 0) for l in labels_subset)
        total_optimal = sum(all_metrics[l].get("optimal_steps", 0) for l in labels_subset)
        total_failed = sum(all_metrics[l].get("failed_steps", 0) for l in labels_subset)
        total_hold = sum(all_metrics[l].get("hold_count", 0) for l in labels_subset)

        # Weighted means
        weighted_solve = sum(
            all_metrics[l].get("total_steps", 0) * all_metrics[l].get("solve_time_mean_us", 0)
            for l in labels_subset
        ) / total_cycles if total_cycles else 0

        weighted_cycle = sum(
            all_metrics[l].get("total_steps", 0) * all_metrics[l].get("cycle_time_mean_us", 0)
            for l in labels_subset
        ) / total_cycles if total_cycles else 0

        weighted_rms = sum(
            all_metrics[l].get("total_steps", 0) * all_metrics[l].get("position_rms_error", 0) ** 2
            for l in labels_subset
        ) / total_cycles if total_cycles else 0
        rms = weighted_rms ** 0.5

        # Deadline miss: weighted by cycles
        weighted_deadline = sum(
            all_metrics[l].get("total_steps", 0) * all_metrics[l].get("deadline_miss_pct", 0)
            for l in labels_subset
        ) / total_cycles if total_cycles else 0

        # Diagnostics rate: weighted by cycles
        weighted_hz = sum(
            all_metrics[l].get("total_steps", 0) * all_metrics[l].get("diagnostics_rate_hz", 0)
            for l in labels_subset
        ) / total_cycles if total_cycles else 0

        return {
            "cycles": total_cycles,
            "optimal": total_optimal,
            "optimal_rate": total_optimal / total_cycles * 100 if total_cycles else 0,
            "failed": total_failed,
            "hold": total_hold,
            "hold_rate": total_hold / total_cycles * 100 if total_cycles else 0,
            "solve_mean_us": weighted_solve,
            "solve_mean_ms": weighted_solve / 1000,
            "cycle_mean_us": weighted_cycle,
            "cycle_mean_ms": weighted_cycle / 1000,
            "rms": rms,
            "deadline_miss_pct": weighted_deadline,
            "hz": weighted_hz,
        }

    a_stats = group_stats(a_labels)
    b_stats = group_stats(b_labels)

    print(f"\n  {'Metric':<35} {'v0.2.0 (A)':>15} {'v0.2.1 (B)':>15} {'Δ':>12} {'Improvement':>12}")
    print("  " + "-" * 89)

    comparisons = [
        ("cycles", "Total cycles", "int", ""),
        ("optimal_rate", "Optimal solve rate", "pct", "pp"),
        ("hold_rate", "Hold rate", "pct", "pp"),
        ("solve_mean_ms", "Mean solve time", "ms", ""),
        ("cycle_mean_ms", "Mean cycle time", "ms", ""),
        ("deadline_miss_pct", "Deadline miss rate", "pct", "pp"),
        ("rms", "Position RMS error", "rad", ""),
        ("hz", "Diagnostics rate", "hz", ""),
    ]

    for key, name, unit, suffix in comparisons:
        a_val = a_stats[key]
        b_val = b_stats[key]
        if unit == "pct":
            delta = b_val - a_val
            delta_str = f"{delta:+.2f} {suffix}".strip()
            impr = "✓" if delta < 0 else "✗"
            if key == "optimal_rate":
                impr = "✓" if delta > 0 else "✗"
            print(f"  {name:<35} {a_val:>13.2f}%  {b_val:>13.2f}%  {delta_str:>12} {impr:>12}")
        elif unit == "int":
            delta = b_val - a_val
            delta_str = f"{delta:+d}"
            print(f"  {name:<35} {a_val:>15,d} {b_val:>15,d} {delta_str:>12} {'':>12}")
        elif unit == "ms":
            delta = b_val - a_val
            delta_pct = delta / a_val * 100 if a_val else 0
            delta_str = f"{delta:+.2f}ms ({delta_pct:+.1f}%)"
            impr = "✓" if delta < 0 else "✗"
            print(f"  {name:<35} {a_val:>13.2f}ms  {b_val:>13.2f}ms  {delta_str:>20} {impr:>12}")
        elif unit == "hz":
            delta = b_val - a_val
            impr = "✓" if delta > 0 else "✗"
            print(f"  {name:<35} {a_val:>13.1f}Hz  {b_val:>13.1f}Hz  {delta:+.1f}Hz {'':>12} {impr:>12}")
        elif unit == "rad":
            delta = b_val - a_val
            delta_pct = delta / a_val * 100 if a_val else 0
            delta_str = f"{delta:+.3f} ({delta_pct:+.1f}%)"
            impr = "✓" if delta < 0 else "✗"
            print(f"  {name:<35} {a_val:>13.3f}  {b_val:>13.3f}  {delta_str:>20} {impr:>12}")

    # ── Pairwise analysis ────────────────────────────────────────
    print("\n\n" + "=" * 90)
    print("  Pairwise Δ Analysis (each A-B pair)")
    print("=" * 90)

    pairs = [
        ("A1", "B2", "Pair 1"),
        ("B3", "A4", "Pair 2"),  # B then A — catch order bias
        ("A5", "B6", "Pair 3"),
        ("B7", "A8", "Pair 4"),  # B then A
        ("A9", "B10", "Pair 5"),
    ]

    for l1, l2, pname in pairs:
        m1 = all_metrics.get(l1, {})
        m2 = all_metrics.get(l2, {})
        cycles1 = m1.get("total_steps", 0)
        cycles2 = m2.get("total_steps", 0)
        if not cycles1 or not cycles2:
            continue
        rms1 = m1.get("position_rms_error", 0)
        rms2 = m2.get("position_rms_error", 0)
        cycle1 = m1.get("cycle_time_mean_us", 0) / 1000
        cycle2 = m2.get("cycle_time_mean_us", 0) / 1000
        solve1 = m1.get("solve_time_mean_us", 0) / 1000
        solve2 = m2.get("solve_time_mean_us", 0) / 1000
        deadline1 = m1.get("deadline_miss_pct", 0)
        deadline2 = m2.get("deadline_miss_pct", 0)

        # For order-consistent comparisons:
        # If pair format is (A, B), delta = B - A
        # If pair format is (B, A), delta = A - B (to get consistent B improvement)
        is_ab_order = l1.startswith("A") and l2.startswith("B")

        if is_ab_order:
            delta_cycle = cycle2 - cycle1
            delta_solve = solve2 - solve1
            delta_rms = rms2 - rms1
            delta_dead = deadline2 - deadline1
        else:
            # Inverted pair: first is B, second is A
            # B improvement from A = l1(cycle) - l2(cycle)
            delta_cycle = cycle1 - cycle2
            delta_solve = solve1 - solve2
            delta_rms = rms1 - rms2
            delta_dead = deadline1 - deadline2

        print(f"\n  {pname} ({l1}↔{l2}):")
        print(f"    Cycle time Δ:   {delta_cycle:+.2f}ms {'✓' if delta_cycle < 0 else '✗'}")
        print(f"    Solve time Δ:   {delta_solve:+.2f}ms {'✓' if delta_solve < 0 else '✗'}")
        print(f"    Deadline miss Δ: {delta_dead:+.2f}pp {'✓' if delta_dead < 0 else '✗'}")
        print(f"    RMS error Δ:    {delta_rms:+.3f} rad {'✓' if delta_rms < 0 else '✗'}")

    # ── Summary judgment ──────────────────────────────────────────
    print("\n\n" + "=" * 90)
    print("  Summary")
    print("=" * 90)

    cycle_improv = (a_stats["cycle_mean_ms"] - b_stats["cycle_mean_ms"]) / a_stats["cycle_mean_ms"] * 100
    solve_improv = (a_stats["solve_mean_ms"] - b_stats["solve_mean_ms"]) / a_stats["solve_mean_ms"] * 100

    better_count = 0
    for l1, l2, pname in pairs:
        m1 = all_metrics.get(l1, {})
        m2 = all_metrics.get(l2, {})
        c1 = m1.get("cycle_time_mean_us", 0)
        c2 = m2.get("cycle_time_mean_us", 0)
        is_ab_order = l1.startswith("A") and l2.startswith("B")
        if is_ab_order:
            if c2 < c1:
                better_count += 1
        else:
            if c1 < c2:
                better_count += 1

    print(f"\n  Total A runs (v0.2.0): {a_stats['cycles']:>6,d} cycles across {len(a_labels)} runs")
    print(f"  Total B runs (v0.2.1): {b_stats['cycles']:>6,d} cycles across {len(b_labels)} runs")
    print(f"\n  Cycle time improvement: {a_stats['cycle_mean_ms']:.2f}ms → {b_stats['cycle_mean_ms']:.2f}ms ({cycle_improv:+.1f}%)")
    print(f"  Solve time improvement: {a_stats['solve_mean_ms']:.2f}ms → {b_stats['solve_mean_ms']:.2f}ms ({solve_improv:+.1f}%)")
    print(f"\n  Pairs with B < A cycle time: {better_count}/{len(pairs)}")

    if better_count >= 4:
        print("  → Cached Hessian benefit confirmed: ≥4/5 pairs show improvement ✓")
    elif better_count >= 3:
        print("  → Cached Hessian benefit likely: 3/5 pairs show improvement")
    else:
        print("  → Cached Hessian benefit not clearly established: <3/5 pairs show improvement")

    print("\n")


if __name__ == "__main__":
    main()
