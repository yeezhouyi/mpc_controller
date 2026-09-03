#!/usr/bin/env python3
"""Fit RRBot joint models from an identification rosbag (C3).

Reads ``/joint_states`` (sensor_msgs/JointState) and the effort command
topic (Float64MultiArray), aligns samples by stamp, and fits the discrete
velocity ARX per joint with delay scan + time-split validation (see
system_identification/fit_joint_model.py).

Usage:
    python3 system_identification/scripts/fit_rrbot_model.py \
        --bag <rosbag dir> --out records/sysid_rrbot_<ts>
"""
import argparse
import collections
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from system_identification.fit_joint_model import fit_velocity_arx  # noqa: E402


def read_bag(bag: str, cmd_topic: str, input_mode: str = "commands"):
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"),
    )
    js_time = []
    js_q = collections.defaultdict(list)
    js_qd = collections.defaultdict(list)
    cmd_time = []
    cmd_tau = []
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic == "/joint_states":
            m = deserialize_message(data, JointState)
            t = ts / 1e9
            js_time.append(t)
            for i, name in enumerate(m.name):
                js_q[name].append(m.position[i] if i < len(m.position) else float("nan"))
                js_qd[name].append(m.velocity[i] if i < len(m.velocity) else float("nan"))
        elif topic == cmd_topic:
            m = deserialize_message(data, Float64MultiArray)
            cmd_time.append(ts / 1e9)
            if input_mode == "diagnostics":
                # /mpc_controller/diagnostics layout: u at idx 5+3*nx .. 5+3*nx+nu
                # nx=4, nu=2 -> u at [17], [18]
                cmd_tau.append([m.data[17], m.data[18]])
            else:
                cmd_tau.append(list(m.data))
    js_time = np.array(js_time)
    cmd_time = np.array(cmd_time)
    cmd_tau = np.array(cmd_tau)
    return js_time, dict(js_q), dict(js_qd), cmd_time, cmd_tau


def align(tau_t, tau, y_t, y):
    """Nearest-time alignment of command samples onto state samples."""
    idx = np.searchsorted(tau_t, y_t)
    idx = np.clip(idx, 0, len(tau_t) - 1)
    tau_a = tau[idx]
    return tau_a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--cmd-topic", default="/mpc_controller/diagnostics")
    ap.add_argument("--input-mode", choices=["commands", "diagnostics"], default="diagnostics")
    ap.add_argument("--out", default=None)
    ap.add_argument("--Ts", type=float, default=0.01)
    ap.add_argument("--max-delay", type=int, default=4)
    args = ap.parse_args()

    js_t, js_q, js_qd, cmd_t, cmd_tau = read_bag(args.bag, args.cmd_topic, args.input_mode)
    names = [n for n in js_q if n in ("joint1", "joint2")]
    print(f"joint_states samples: {len(js_t)}  command samples: {len(cmd_t)}  joints: {names}")
    if len(js_t) < 200 or len(names) < 2:
        print("insufficient data"); sys.exit(2)

    results = {}
    for j, name in enumerate(names):
        y = np.array(js_qd[name])
        tau_j = align(cmd_t, cmd_tau[:, j], js_t, y)
        ok = np.isfinite(y) & np.isfinite(tau_j)
        fit = fit_velocity_arx(y[ok], tau_j[ok], args.Ts, max_delay=args.max_delay)
        results[name] = fit
        print(f"{name}: {fit}")

    outdir = args.out or "records/sysid_rrbot_default"
    os.makedirs(outdir, exist_ok=True)
    meta = {
        "bag": args.bag,
        "cmd_topic": args.cmd_topic,
        "Ts": args.Ts,
        "samples": len(js_t),
        "joints": results,
    }
    with open(os.path.join(outdir, "fitted_model.json"), "w") as f:
        json.dump(meta, f, indent=2)
    print(f"wrote {outdir}/fitted_model.json")


if __name__ == "__main__":
    main()
