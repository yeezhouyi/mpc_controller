#!/usr/bin/env python3
"""Excite the RRBot joints with effort commands for identification (C3).

Publishes per-joint PRBS + multi-sine effort commands on
``/fwd_effort_controller/commands`` (Float64MultiArray) at 100 Hz.  Record
``/joint_states`` and the command topic with rosbag while this runs.

Usage:
    ros2 run mpc_controller excite_rrbot.py --ros-args \
        -p duration_s:=30.0 -p amp:=1.5 -p rate:=100.0
"""
import sys
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class ExciteRRBot(Node):
    def __init__(self):
        super().__init__("excite_rrbot")
        self.declare_parameter("duration_s", 30.0)
        self.declare_parameter("amp", 1.5)
        self.declare_parameter("rate", 100.0)
        self.declare_parameter("topic", "commands")
        self.duration = float(self.get_parameter("duration_s").value)
        self.amp = float(self.get_parameter("amp").value)
        self.rate = float(self.get_parameter("rate").value)
        topic = self.get_parameter("topic").value
        self._pub = self.create_publisher(Float64MultiArray, topic, 10)
        self._dt = 1.0 / self.rate
        self._t = 0.0
        self._timer = self.create_timer(self._dt, self._tick)
        self._count = 0
        self._rng = __import__("random").Random(42)

    def _tick(self):
        t = self._t
        if t > self.duration:
            self.get_logger().info(f"excitation done at t={t:.1f}s")
            rclpy.shutdown()
            return
        # PRBS-ish low-frequency component + shaping sines (decorrelated)
        tau = []
        for j, phase in enumerate([0.0, 1.7]):
            # low-frequency pseudo random steps every ~1.2 s
            seg = int(t // 1.2)
            step = 1.0 if self._rng.random() < 0.5 else -1.0
            prbs_c = step * 0.7 * self.amp
            sine_c = 0.5 * self.amp * math.sin(2 * math.pi * (1.1 + 0.3 * j) * t + phase)
            tau.append(prbs_c + sine_c)
        msg = Float64MultiArray()
        msg.data = [tau[0], tau[1]]
        msg.layout.data_offset = 0
        self._pub.publish(msg)
        self._count += 1
        self._t += self._dt
        if self._count % (int(self.rate) * 5) == 0:
            self.get_logger().info(f"t={t:.1f}s sent={self._count}")


def main():
    rclpy.init()
    node = ExciteRRBot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
