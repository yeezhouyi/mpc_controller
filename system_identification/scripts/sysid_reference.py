#!/usr/bin/env python3
"""Multi-sine joint reference for MPC-driven identification (C3).

Publishes Float64MultiArray [q1, q2, qd1, qd2] on /mpc_controller/reference
at 100 Hz so the MPC produces rich effort for fitting.  Duration-limited.

Usage:
    python3 system_identification/scripts/sysid_reference.py --ros-args \
        -p duration_s:=30.0 -p rate:=100.0
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class SysidReference(Node):
    def __init__(self):
        super().__init__("sysid_reference")
        self.declare_parameter("duration_s", 30.0)
        self.declare_parameter("rate", 100.0)
        self.duration = float(self.get_parameter("duration_s").value)
        self.rate = float(self.get_parameter("rate").value)
        self._pub = self.create_publisher(Float64MultiArray, "/mpc_controller/reference", 10)
        self._dt = 1.0 / self.rate
        self._t = 0.0
        self.create_timer(self._dt, self._tick)

    def _ref(self, t: float):
        # per-joint multi-sine (0.4/0.9/1.6 Hz harmonics, amplitude 0.35 rad)
        out = []
        for j, (f0, phi) in enumerate([(0.4, 0.0), (0.9, 1.1)]):
            q = 0.35 * math.sin(2 * math.pi * f0 * t + phi) \
                + 0.12 * math.sin(2 * math.pi * (f0 * 3.1) * t + 2.0 * phi)
            qd = 0.35 * 2 * math.pi * f0 * math.cos(2 * math.pi * f0 * t + phi) \
                + 0.12 * 2 * math.pi * (f0 * 3.1) * math.cos(2 * math.pi * (f0 * 3.1) * t + 2.0 * phi)
            out += [q, qd]
        return out

    def _tick(self):
        if self._t > self.duration:
            self.get_logger().info(f"sysid reference done at t={self._t:.1f}s")
            rclpy.shutdown()
            return
        msg = Float64MultiArray()
        msg.data = self._ref(self._t)
        self._pub.publish(msg)
        self._t += self._dt


def main():
    rclpy.init()
    node = SysidReference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
