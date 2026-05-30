#!/usr/bin/env python3
"""Publish a reference trajectory for MPC controller testing.

Publishes a multi-DOF reference signal (sine/cosine) for joint position
and velocity tracking.  Configured for RRBot (2 joints, 4 states) by
default; set ``num_joints=1`` for a single-joint test.

Topics:
  - Published: ``<topic>`` (default ``/mpc_controller/reference``)

Usage:
  ros2 run mpc_controller trajectory_publisher.py --ros-args \
    -p amplitude:=1.0 -p frequency:=0.2 -p num_joints:=2
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_publisher")
        self.declare_parameter("amplitude", 1.0)
        self.declare_parameter("frequency", 0.2)
        self.declare_parameter("num_joints", 2)
        self.declare_parameter("topic", "/mpc_controller/reference")

        amplitude = self.get_parameter("amplitude").value
        frequency = self.get_parameter("frequency").value
        num_joints = self.get_parameter("num_joints").value
        topic = self.get_parameter("topic").value

        self.amplitude = amplitude
        self.omega = 2.0 * math.pi * frequency
        self.num_joints = num_joints

        self.pub = self.create_publisher(Float64MultiArray, topic, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.t = 0.0

        self.get_logger().info(
            f"Publishing {num_joints}-joint reference "
            f"{amplitude:.2f} * sin({self.omega:.3f}*t) + phase offset "
            f"on '{topic}'"
        )

    def timer_callback(self):
        self.t += 0.01
        t = self.t

        msg = Float64MultiArray()
        msg.data = []
        for j in range(self.num_joints):
            phase = 2.0 * math.pi * j / self.num_joints
            pos = self.amplitude * math.sin(self.omega * t + phase)
            vel = self.amplitude * self.omega * math.cos(self.omega * t + phase)
            msg.data.append(pos)
            msg.data.append(vel)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
